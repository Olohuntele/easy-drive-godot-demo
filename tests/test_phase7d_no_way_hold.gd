extends SceneTree

const CampusMultiRouteNetwork = preload("res://scripts/campus_multi_route_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const CampusServiceCoordinator = preload("res://scripts/campus_service_coordinator.gd")
const PassengerJourney = preload("res://scripts/passenger_journey.gd")

const FIXED_STEP: float = 1.0 / 60.0
const MAX_TICKS: int = 18000

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    await _test_no_way_hold_and_release()
    if failures == 0:
        print("Phase 7D no-way hold tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d Phase 7D assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _new_coordinator() -> RefCounted:
    var network := CampusMultiRouteNetwork.new()
    var coordinator := CampusServiceCoordinator.new()
    coordinator.network = network
    coordinator.planner = CampusRoutePlanner.new(network)
    var north_stop: Dictionary = network.get_station_stop("north_gate")
    var launch: Dictionary = network.get_segment("north_to_east")
    var points: Array = launch.centerline_points
    var tangent: Vector2 = points[1] - points[0]
    coordinator.initialize_service(
        "north_gate",
        north_stop.position_xz,
        atan2(tangent.x, tangent.y)
    )
    return coordinator

func _test_no_way_hold_and_release() -> void:
    var coordinator := _new_coordinator()
    _expect(
        coordinator.request_booking("sports_hall", "innovation_centre").get("valid", false),
        "no-way scenario installs a pickup route"
    )
    for tick in range(90):
        coordinator.step(FIXED_STEP, tick + 1)

    var total_length: float = coordinator.navigator.total_length_m
    var no_way_obstacle_distance: float = total_length - 8.0
    coordinator.place_obstacle(no_way_obstacle_distance, 3.0)
    var held := false
    for tick in range(MAX_TICKS):
        coordinator.step(FIXED_STEP, 1000 + tick)
        if coordinator.no_way_hold_active:
            held = true
            break
        if coordinator.journey.state == PassengerJourney.BLOCKED:
            break
    _expect(held, "no clear path produces a persistent hold")
    _expect(
        coordinator.journey.state == PassengerJourney.TO_PICKUP
            && coordinator.get_service_snapshot().blocked == false,
        "no-way hold keeps the booking alive without BLOCKED"
    )

    coordinator.clear_obstacle()
    _expect(
        not coordinator.no_way_hold_active,
        "clearing the obstacle releases the no-way hold flag"
    )
    coordinator.place_obstacle(no_way_obstacle_distance, 3.0)
    var held_again := false
    for tick in range(3600):
        coordinator.step(FIXED_STEP, 10000 + tick)
        if coordinator.no_way_hold_active:
            held_again = true
            break
    _expect(held_again, "repeated hold and release cycles remain stable")

    coordinator.clear_obstacle()
    _expect(
        _drive_until(coordinator, PassengerJourney.BOARDING, 8000, 20000),
        "clearing the obstacle resumes normal journey completion"
    )

func _drive_until(
    coordinator: RefCounted,
    target_state: String,
    max_ticks: int,
    tick_offset: int
) -> bool:
    for tick in range(max_ticks):
        coordinator.step(FIXED_STEP, tick_offset + tick)
        if coordinator.journey.state == target_state:
            return true
        if coordinator.journey.state == PassengerJourney.BLOCKED:
            return false
    return false
