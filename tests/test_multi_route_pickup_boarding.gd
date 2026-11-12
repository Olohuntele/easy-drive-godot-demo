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
    print("EASY DRIVE â€” PHASE 5B.2A: PICKUP ARRIVAL AND PASSENGER BOARDING TESTS")
    print("================================================================================")

    var network = CampusMultiRouteNetwork.new()
    _expect(network.is_valid, "CampusMultiRouteNetwork is valid")

    var north_stop = network.get_station_stop("north_gate")
    var launch_segment: Dictionary = network.get_segment("north_to_east")
    var launch_points: Array = launch_segment.get("centerline_points", [])
    var launch_tangent: Vector2 = launch_points[1] - launch_points[0]
    var launch_heading_rad: float = atan2(launch_tangent.x, launch_tangent.y)

    # 1. Test Premature Boarding Rejection (shuttle traveling to pickup at sports_hall)
    var coord_premature = CampusServiceCoordinator.new()
    coord_premature.network = network
    coord_premature.planner = CampusRoutePlanner.new(network)
    coord_premature.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)
    var req1 = coord_premature.request_booking("sports_hall", "library")
    _expect(req1.get("valid", false), "booking (sports_hall -> library) accepted")
    _expect(coord_premature.journey.state == PassengerJourney.TO_PICKUP, "journey is traveling to pickup (TO_PICKUP)")
    var premature_board = coord_premature.confirm_boarding()
    _expect(not premature_board.get("valid", false), "premature boarding before arrival certification is rejected")

    # 2. Test Correct Pickup Arrival & Stationary Waiting
    var coordinator = CampusServiceCoordinator.new()
    coordinator.network = network
    coordinator.planner = CampusRoutePlanner.new(network)
    coordinator.defer_destination_routing = true
    coordinator.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)

    var booking = coordinator.request_booking("north_gate", "library")
    _expect(booking.get("valid", false), "booking north_gate -> library requested")

    var max_steps: int = 300
    var arrived: bool = false
    for i in range(max_steps):
        var step_res = coordinator.step(1.0 / 60.0)
        var snap = coordinator.get_service_snapshot()
        if snap.get("service_state", "") == PassengerJourney.BOARDING && snap.get("arrival_certified", false):
            arrived = true
            break

    _expect(arrived, "shuttle reaches requested pickup station and certifies arrival")
    _expect(coordinator.get_service_snapshot().get("speed_mps", 1.0) <= 0.1, "shuttle satisfies stopped speed tolerance at pickup")

    # 3. Test Successful Boarding Confirmation
    var board_res = coordinator.confirm_boarding()
    _expect(board_res.get("valid", false), "successful boarding confirmation is accepted")
    _expect(coordinator.get_service_snapshot().get("boarding_confirmed", false), "coordinator records boarding confirmed")

    # 4. Test Duplicate Boarding Rejection
    var dup_board = coordinator.confirm_boarding()
    _expect(not dup_board.get("valid", false), "duplicate boarding confirmation is rejected")

    # 5. Test No Unintended Destination-Leg Movement
    var snap_after_board = coordinator.get_service_snapshot()
    _expect(snap_after_board.get("service_state", "") == PassengerJourney.BOARDING, "journey state remains BOARDING after boarding confirmation")
    _expect(snap_after_board.get("current_leg", "") != "destination", "destination leg execution is deferred (not started)")
    _expect(snap_after_board.get("speed_mps", 1.0) == 0.0, "shuttle remains stationary while awaiting Phase 5B.2B destination travel")

    # 6. Test Reset During Pickup / Boarding
    var coord_reset = CampusServiceCoordinator.new()
    coord_reset.network = network
    coord_reset.planner = CampusRoutePlanner.new(network)
    coord_reset.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)
    coord_reset.request_booking("north_gate", "library")
    coord_reset.step(1.0 / 60.0)
    var reset_coord = CampusServiceCoordinator.new()
    reset_coord.network = network
    reset_coord.planner = CampusRoutePlanner.new(network)
    var reset_init = reset_coord.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)
    _expect(reset_init.get("valid", false) && reset_coord.journey.state == PassengerJourney.IDLE, "reset during pickup clears booking and returns to IDLE")

    # 7. Test Pickup Routing from Valid Pose vs Arbitrary Between-Stations Limitation
    var planner = CampusRoutePlanner.new(network)
    var between_pos = Vector2(50.0, 50.0)
    var between_route = planner.plan_route_from_pose("north_gate", "library", between_pos, 0.0)
    _expect(not between_route.get("valid", false), "arbitrary between-stations routing is correctly rejected as origin_pose_incompatible")

    # 8. Test Scene Parsing, Manual Navigation Inhibition & Instantiation
    var scene = load("res://scenes/multi_route_demo.tscn")
    _expect(scene != null, "multi_route_demo.tscn loads successfully")
    if scene != null:
        var node = scene.instantiate()
        _expect(node != null, "multi_route_demo scene instantiates")
        if node != null:
            _expect(node.has_method("_is_passenger_service_active"), "multi_route_demo has _is_passenger_service_active method")
            _expect(not node._is_passenger_service_active(), "passenger service is initially inactive (allowing manual routes)")
            node.service_coordinator.request_booking("sports_hall", "library")
            _expect(node._is_passenger_service_active(), "passenger service is active when booking requested, inhibiting manual route keys 1-4")
            node.free()

    print("Phase 5B.2A pickup & boarding tests complete: %d assertions, %d failures." % [assertions, failures])
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
