extends SceneTree

const CampusMultiRouteNetwork = preload("res://scripts/campus_multi_route_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const CampusServiceCoordinator = preload("res://scripts/campus_service_coordinator.gd")
const PassengerJourney = preload("res://scripts/passenger_journey.gd")

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    print("================================================================================")
    print("EASY DRIVE — PHASE 5B.1: MULTI-ROUTE PASSENGER SERVICE FOUNDATION TESTS")
    print("================================================================================")

    # 1. Test Network & Coordinator Compatibility
    var network = CampusMultiRouteNetwork.new()
    _expect(network.is_valid, "CampusMultiRouteNetwork is valid")

    var coordinator = CampusServiceCoordinator.new()
    coordinator.network = network
    coordinator.planner = CampusRoutePlanner.new(network)

    var north_stop = network.get_station_stop("north_gate")
    _expect(north_stop.get("valid", false), "north_gate station stop is valid")

    var launch_segment: Dictionary = network.get_segment("north_to_east")
    var launch_points: Array = launch_segment.get("centerline_points", [])
    var launch_tangent: Vector2 = launch_points[1] - launch_points[0]
    var launch_heading_rad: float = atan2(launch_tangent.x, launch_tangent.y)

    var init_res = coordinator.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)
    _expect(init_res.get("valid", false), "service coordinator initializes successfully with multi-route network")
    _expect(coordinator.journey.state == PassengerJourney.IDLE, "initial coordinator state is IDLE")

    # 2. Test Valid Booking Acceptance (North Gate -> Library)
    var booking_res = coordinator.request_booking("north_gate", "library")
    _expect(booking_res.get("valid", false), "valid booking (north_gate -> library) accepted by coordinator")
    _expect(coordinator.journey.state != PassengerJourney.IDLE, "coordinator state transitions out of IDLE upon booking")

    # 2b. Test Sports Hall -> Innovation Centre on CampusMultiRouteNetwork
    var coord_sports = CampusServiceCoordinator.new()
    coord_sports.network = network
    coord_sports.planner = CampusRoutePlanner.new(network)
    coord_sports.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)
    var booking_sports = coord_sports.request_booking("sports_hall", "innovation_centre")
    _expect(booking_sports.get("valid", false), "booking (sports_hall -> innovation_centre) accepted by coordinator: %s" % booking_sports)
    _expect(coord_sports.journey.state != PassengerJourney.BLOCKED, "coordinator state is not BLOCKED after sports_hall booking, state=%s reason=%s" % [coord_sports.journey.state, coord_sports.blocked_reason])

    # 3. Test Invalid Booking Rejection (Identical pickup/destination)
    var coord_idle = CampusServiceCoordinator.new()
    coord_idle.network = network
    coord_idle.planner = CampusRoutePlanner.new(network)
    coord_idle.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)

    var invalid_booking = coord_idle.request_booking("library", "library")
    _expect(not invalid_booking.get("valid", false), "identical pickup and destination booking rejected")

    # 4. Test Scene Parsing & Instantiation
    var scene = load("res://scenes/multi_route_demo.tscn")
    _expect(scene != null, "multi_route_demo.tscn loads successfully")
    if scene != null:
        var node = scene.instantiate()
        _expect(node != null, "multi_route_demo scene instantiates")
        if node != null:
            node.free()

    print("Multi-route passenger foundation tests complete: %d assertions, %d failures." % [assertions, failures])
    if failures == 0:
        quit(0)
    else:
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1
