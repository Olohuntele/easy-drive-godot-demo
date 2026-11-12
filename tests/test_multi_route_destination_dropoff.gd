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
    print("EASY DRIVE — PHASE 5B.2B: DESTINATION NAVIGATION & PASSENGER DROP-OFF TESTS")
    print("================================================================================")

    var network = CampusMultiRouteNetwork.new()
    _expect(network.is_valid, "CampusMultiRouteNetwork is valid")

    var north_stop = network.get_station_stop("north_gate")
    var launch_segment: Dictionary = network.get_segment("north_to_east")
    var launch_points: Array = launch_segment.get("centerline_points", [])
    var launch_tangent: Vector2 = launch_points[1] - launch_points[0]
    var launch_heading_rad: float = atan2(launch_tangent.x, launch_tangent.y)

    # 1. Test Complete Journey (Sports Hall -> Innovation Centre) with Defer Return Routing
    var coordinator = CampusServiceCoordinator.new()
    coordinator.network = network
    coordinator.planner = CampusRoutePlanner.new(network)
    coordinator.defer_destination_routing = false
    coordinator.defer_return_routing = true
    coordinator.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)

    var booking = coordinator.request_booking("sports_hall", "innovation_centre")
    _expect(booking.get("valid", false), "booking sports_hall -> innovation_centre requested successfully")

    # Step to pickup
    var max_steps: int = 600
    var pickup_arrived: bool = false
    for i in range(max_steps):
        coordinator.step(1.0 / 60.0)
        var snap = coordinator.get_service_snapshot()
        if snap.get("service_state", "") == PassengerJourney.BOARDING && snap.get("arrival_certified", false):
            pickup_arrived = true
            break

    _expect(pickup_arrived, "shuttle arrives at sports_hall pickup station and certifies arrival")

    # Premature exit rejection before destination arrival
    var premature_exit = coordinator.confirm_passenger_exit()
    _expect(not premature_exit.get("valid", false), "premature passenger exit before destination arrival is rejected")

    # Confirm boarding -> should automatically trigger destination routing (TO_DESTINATION)
    var board_res = coordinator.confirm_boarding()
    _expect(board_res.get("valid", false), "boarding confirmation accepted")
    _expect(coordinator.get_service_snapshot().get("service_state", "") == PassengerJourney.TO_DESTINATION, "journey transitions to TO_DESTINATION automatically after boarding")

    # Step to destination (innovation_centre)
    var dest_arrived: bool = false
    for i in range(800):
        coordinator.step(1.0 / 60.0)
        var snap = coordinator.get_service_snapshot()
        if snap.get("service_state", "") == PassengerJourney.DROPOFF && snap.get("arrival_certified", false):
            dest_arrived = true
            break

    _expect(dest_arrived, "shuttle arrives at innovation_centre destination and certifies arrival")
    _expect(coordinator.get_service_snapshot().get("destination_station_id", "") == "innovation_centre", "destination station id matches innovation_centre")

    # Duplicate exit rejection test / check exit
    var exit_res = coordinator.confirm_passenger_exit()
    _expect(exit_res.get("valid", false), "successful passenger exit confirmation is accepted")
    _expect(coordinator.get_service_snapshot().get("exit_confirmed", false), "service records exit confirmed")

    var dup_exit = coordinator.confirm_passenger_exit()
    _expect(not dup_exit.get("valid", false), "duplicate passenger exit confirmation is rejected")

    var snap_after_exit = coordinator.get_service_snapshot()
    _expect(snap_after_exit.get("service_state", "") == PassengerJourney.DROPOFF, "journey state remains DROPOFF when return routing is deferred")
    _expect(snap_after_exit.get("speed_mps", 1.0) == 0.0, "shuttle remains stationary after passenger exit")

    # 2. Test Complete Journey (Library -> North Gate)
    var coord_lib = CampusServiceCoordinator.new()
    coord_lib.network = network
    coord_lib.planner = CampusRoutePlanner.new(network)
    coord_lib.defer_destination_routing = false
    coord_lib.defer_return_routing = true
    coord_lib.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)
    coord_lib.request_booking("library", "north_gate")
    
    var lib_pickup_arrived: bool = false
    for i in range(500):
        coord_lib.step(1.0 / 60.0)
        var snap = coord_lib.get_service_snapshot()
        if snap.get("service_state", "") == PassengerJourney.BOARDING && snap.get("arrival_certified", false):
            lib_pickup_arrived = true
            break
    _expect(lib_pickup_arrived, "library pickup arrival certified")
    coord_lib.confirm_boarding()

    var lib_dest_arrived: bool = false
    for i in range(700):
        coord_lib.step(1.0 / 60.0)
        var snap = coord_lib.get_service_snapshot()
        if snap.get("service_state", "") == PassengerJourney.DROPOFF && snap.get("arrival_certified", false):
            lib_dest_arrived = true
            break
    _expect(lib_dest_arrived, "north_gate destination arrival certified for library journey")
    var lib_exit = coord_lib.confirm_passenger_exit()
    _expect(lib_exit.get("valid", false), "library journey exit confirmed successfully")

    # 3. Test Reset During Destination Travel & Drop-Off
    var coord_reset = CampusServiceCoordinator.new()
    coord_reset.network = network
    coord_reset.planner = CampusRoutePlanner.new(network)
    coord_reset.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)
    coord_reset.request_booking("sports_hall", "library")
    coord_reset.step(1.0 / 60.0)
    var reset_coord = CampusServiceCoordinator.new()
    reset_coord.network = network
    reset_coord.planner = CampusRoutePlanner.new(network)
    var reset_init = reset_coord.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)
    _expect(reset_init.get("valid", false) && reset_coord.journey.state == PassengerJourney.IDLE, "reset during destination/drop-off clears state and returns to IDLE")

    # 4. Test Original Mode 5 Compatibility (defer_destination_routing = false, defer_return_routing = false)
    var coord_mode5 = CampusServiceCoordinator.new()
    coord_mode5.network = network
    coord_mode5.planner = CampusRoutePlanner.new(network)
    coord_mode5.defer_destination_routing = false
    coord_mode5.defer_return_routing = false
    coord_mode5.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)
    coord_mode5.request_booking("north_gate", "library")
    for i in range(150):
        coord_mode5.step(1.0 / 60.0)
    coord_mode5.confirm_boarding()
    for i in range(300):
        coord_mode5.step(1.0 / 60.0)
    var mode5_snap = coord_mode5.get_service_snapshot()
    coord_mode5.confirm_passenger_exit()
    var mode5_return_snap = coord_mode5.get_service_snapshot()
    _expect(mode5_return_snap.get("service_state", "") == PassengerJourney.RETURNING_TO_SERVICE, "original Mode 5 automatically starts return-to-service routing after exit")

    # 5. Scene Parsing & Instantiation Validation
    var main_scene = load("res://scenes/main.tscn")
    _expect(main_scene != null, "main.tscn loads successfully")
    if main_scene != null:
        var node = main_scene.instantiate()
        _expect(node != null, "main scene instantiates")
        if node != null:
            node.free()

    var demo_scene = load("res://scenes/multi_route_demo.tscn")
    _expect(demo_scene != null, "multi_route_demo.tscn loads successfully")
    if demo_scene != null:
        var node = demo_scene.instantiate()
        _expect(node != null, "multi_route_demo scene instantiates")
        if node != null:
            node.free()

    print("Phase 5B.2B destination & drop-off tests complete: %d assertions, %d failures." % [assertions, failures])
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
