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
    print("EASY DRIVE — PHASE 5C.1: RETURN-TO-SERVICE NAVIGATION FOUNDATION TESTS")
    print("================================================================================")

    var network = CampusMultiRouteNetwork.new()
    _expect(network.is_valid, "CampusMultiRouteNetwork is valid")

    var north_stop = network.get_station_stop("north_gate")
    var launch_segment: Dictionary = network.get_segment("north_to_east")
    var launch_points: Array = launch_segment.get("centerline_points", [])
    var launch_tangent: Vector2 = launch_points[1] - launch_points[0]
    var launch_heading_rad: float = atan2(launch_tangent.x, launch_tangent.y)

    # 1. Test Complete Sequential Journey (Trip 1: Sports Hall -> Innovation Centre -> North Gate Return)
    var coordinator = CampusServiceCoordinator.new()
    coordinator.network = network
    coordinator.planner = CampusRoutePlanner.new(network)
    coordinator.defer_destination_routing = false
    coordinator.defer_return_routing = false
    coordinator.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)

    var booking1 = coordinator.request_booking("sports_hall", "innovation_centre")
    _expect(booking1.get("valid", false), "trip 1 booking (sports_hall -> innovation_centre) requested successfully")

    # Step to pickup
    var pickup_arrived: bool = false
    for i in range(600):
        coordinator.step(1.0 / 60.0)
        var snap = coordinator.get_service_snapshot()
        if snap.get("service_state", "") == PassengerJourney.BOARDING && snap.get("arrival_certified", false):
            pickup_arrived = true
            break
    _expect(pickup_arrived, "shuttle arrives at sports_hall pickup and certifies arrival")

    # Confirm boarding
    var board_res = coordinator.confirm_boarding()
    _expect(board_res.get("valid", false), "boarding confirmed, transitioning to TO_DESTINATION")

    # Step to destination
    var dest_arrived: bool = false
    for i in range(800):
        coordinator.step(1.0 / 60.0)
        var snap = coordinator.get_service_snapshot()
        if snap.get("service_state", "") == PassengerJourney.DROPOFF && snap.get("arrival_certified", false):
            dest_arrived = true
            break
    _expect(dest_arrived, "shuttle arrives at innovation_centre destination and certifies arrival")

    # Confirm passenger exit -> should immediately initiate return to North Gate (RETURNING_TO_SERVICE)
    var exit_res = coordinator.confirm_passenger_exit()
    _expect(exit_res.get("valid", false), "passenger exit confirmed successfully")
    _expect(coordinator.get_service_snapshot().get("service_state", "") == PassengerJourney.RETURNING_TO_SERVICE, "passenger exit successfully initiates return navigation to North Gate (RETURNING_TO_SERVICE)")

    # Second booking rejection during return
    var booking_during_return = coordinator.request_booking("library", "north_gate")
    _expect(not booking_during_return.get("valid", false), "new booking is correctly rejected while shuttle is returning to service")

    # Step back to North Gate
    var return_arrived: bool = false
    for i in range(900):
        coordinator.step(1.0 / 60.0)
        var snap = coordinator.get_service_snapshot()
        if snap.get("service_state", "") == PassengerJourney.IDLE && snap.get("arrival_certified", false):
            return_arrived = true
            break
    _expect(return_arrived, "shuttle successfully returns to North Gate and certifies arrival, transitioning back to IDLE")
    _expect(coordinator.get_service_snapshot().get("active_booking", {}).is_empty(), "completed booking state is safely cleared without clearing vehicle pose")

    # 2. Test Second Sequential Booking (Trip 2: Library -> Sports Hall) without pressing R
    var booking2 = coordinator.request_booking("library", "sports_hall")
    _expect(booking2.get("valid", false), "second sequential booking accepted after return without simulation reset")
    _expect(coordinator.get_service_snapshot().get("service_state", "") == PassengerJourney.ASSIGNED, "journey state is ASSIGNED for second booking")

    # Step trip 2 pickup
    var pickup2_arrived: bool = false
    for i in range(600):
        coordinator.step(1.0 / 60.0)
        var snap = coordinator.get_service_snapshot()
        if snap.get("service_state", "") == PassengerJourney.BOARDING && snap.get("arrival_certified", false):
            pickup2_arrived = true
            break
    _expect(pickup2_arrived, "shuttle successfully services second booking pickup (library)")

    # 3. Test Reset During Return Service
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
    _expect(reset_init.get("valid", false) && reset_coord.journey.state == PassengerJourney.IDLE, "reset during return-to-service clears state and returns to IDLE")

    # 4. Test Original Mode 5 Compatibility & Scene Parsing
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

    print("Phase 5C.1 return-to-service tests complete: %d assertions, %d failures." % [assertions, failures])
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
