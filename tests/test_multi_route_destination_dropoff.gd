extends SceneTree

const CampusMultiRouteNetwork = preload("res://scripts/campus_multi_route_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const CampusServiceCoordinator = preload("res://scripts/campus_service_coordinator.gd")
const PassengerJourney = preload("res://scripts/passenger_journey.gd")
const MAX_TICKS_PER_LEG: int = 12000

var failures: int = 0
var assertions: int = 0
var next_tick_id: int = 1

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
    var pickup_arrived: bool = _step_until_state(
        coordinator,
        PassengerJourney.BOARDING,
        "sports_hall pickup"
    )

    _expect(pickup_arrived, "shuttle arrives at sports_hall pickup station and certifies arrival")

    # Premature exit rejection before destination arrival
    var premature_exit = coordinator.confirm_passenger_exit()
    _expect(not premature_exit.get("valid", false), "premature passenger exit before destination arrival is rejected")

    # Confirm boarding -> should automatically trigger destination routing (TO_DESTINATION)
    var board_res = coordinator.confirm_boarding()
    _expect(board_res.get("valid", false), "boarding confirmation accepted")
    _expect(coordinator.get_service_snapshot().get("service_state", "") == PassengerJourney.TO_DESTINATION, "journey transitions to TO_DESTINATION automatically after boarding")

    # Step to destination (innovation_centre)
    var dest_arrived: bool = _step_until_state(
        coordinator,
        PassengerJourney.DROPOFF,
        "innovation_centre destination"
    )

    _expect(dest_arrived, "shuttle arrives at innovation_centre destination and certifies arrival")
    var destination_snapshot = coordinator.get_service_snapshot()
    _expect(
        destination_snapshot.get("service_state", "") == PassengerJourney.DROPOFF
            && destination_snapshot.get("arrival_certified", false),
        "destination arrival is certified in DROPOFF state"
    )

    # Duplicate exit rejection test / check exit
    var exit_res = coordinator.confirm_passenger_exit()
    _expect(exit_res.get("valid", false), "successful passenger exit confirmation is accepted")
    _expect(
        exit_res.get("valid", false)
            && coordinator.get_service_snapshot().get("service_state", "") == PassengerJourney.DROPOFF,
        "service records exit confirmed while return routing is deferred"
    )

    var dup_exit = coordinator.confirm_passenger_exit()
    _expect(not dup_exit.get("valid", false), "duplicate passenger exit confirmation is rejected")

    var snap_after_exit = coordinator.get_service_snapshot()
    _expect(snap_after_exit.get("service_state", "") == PassengerJourney.DROPOFF, "journey state remains DROPOFF when return routing is deferred")
    _expect(
        snap_after_exit.get("speed_mps", 1.0) <= PassengerJourney.ARRIVAL_SPEED_TOLERANCE_MPS,
        "shuttle speed is within arrival tolerance at drop-off"
    )

    # 2. Test Complete Journey (Library -> North Gate)
    var coord_lib = CampusServiceCoordinator.new()
    coord_lib.network = network
    coord_lib.planner = CampusRoutePlanner.new(network)
    coord_lib.defer_destination_routing = false
    coord_lib.defer_return_routing = true
    coord_lib.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)
    coord_lib.request_booking("library", "north_gate")

    var lib_pickup_arrived: bool = _step_until_state(
        coord_lib,
        PassengerJourney.BOARDING,
        "library pickup"
    )
    _expect(lib_pickup_arrived, "library pickup arrival certified")
    coord_lib.confirm_boarding()

    var lib_dest_arrived: bool = _step_until_state(
        coord_lib,
        PassengerJourney.DROPOFF,
        "north_gate destination"
    )
    _expect(lib_dest_arrived, "north_gate destination arrival certified for library journey")
    var lib_exit = coord_lib.confirm_passenger_exit()
    _expect(lib_exit.get("valid", false), "library journey exit confirmed successfully")

    # 3. Test Reset During Destination Travel & Drop-Off
    var coord_reset = CampusServiceCoordinator.new()
    coord_reset.network = network
    coord_reset.planner = CampusRoutePlanner.new(network)
    coord_reset.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)
    coord_reset.request_booking("sports_hall", "library")
    _step_coordinator(coord_reset)
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
    _step_until_state(coord_mode5, PassengerJourney.BOARDING, "Mode 5 pickup")
    coord_mode5.confirm_boarding()
    _step_until_state(coord_mode5, PassengerJourney.DROPOFF, "Mode 5 destination")
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

func _step_coordinator(coordinator: RefCounted) -> Dictionary:
    var result: Dictionary = coordinator.step(1.0 / 60.0, next_tick_id)
    next_tick_id += 1
    return result

func _step_until_state(
    coordinator: RefCounted,
    expected_state: String,
    description: String
) -> bool:
    for _tick in range(MAX_TICKS_PER_LEG):
        if coordinator.get_service_snapshot().get("service_state", "") == expected_state:
            return true
        var result := _step_coordinator(coordinator)
        if not result.get("valid", false) && result.get("status", "") != "ARRIVED":
            break
    var actual_state: String = str(
        coordinator.get_service_snapshot().get("service_state", "")
    )
    if actual_state != expected_state:
        push_error(
            "FAIL: %s expected state %s but reached %s within %d ticks."
            % [description, expected_state, actual_state, MAX_TICKS_PER_LEG]
        )
        failures += 1
        return false
    return true
