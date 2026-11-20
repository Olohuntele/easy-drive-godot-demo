extends SceneTree

const CampusServiceCoordinator = preload("res://scripts/campus_service_coordinator.gd")
const PassengerJourney = preload("res://scripts/passenger_journey.gd")

const FIXED_STEP: float = 1.0 / 60.0
const MAX_TICKS: int = 12000

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    await _test_midleg_swap_and_obstacle_reprojection()
    _test_rejection_states()
    if failures == 0:
        print("Phase 7B route swap tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d Phase 7B assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _new_coordinator() -> RefCounted:
    var coordinator := CampusServiceCoordinator.new()
    var north_stop: Dictionary = coordinator.network.get_station_stop("north_gate")
    var launch_segment: Dictionary = coordinator.network.get_segment("north_to_east")
    var launch_points: Array = launch_segment.centerline_points
    var tangent: Vector2 = launch_points[1] - launch_points[0]
    coordinator.initialize_service(
        "north_gate",
        north_stop.position_xz,
        atan2(tangent.x, tangent.y)
    )
    return coordinator

func _test_midleg_swap_and_obstacle_reprojection() -> void:
    var coordinator: RefCounted = _new_coordinator()
    _expect(
        coordinator.request_booking("sports_hall", "innovation_centre").get("valid", false),
        "swap scenario installs an active pickup leg"
    )
    for tick in range(90):
        coordinator.step(FIXED_STEP, tick + 1)
    var before: Dictionary = coordinator.get_service_snapshot()
    var old_progress: float = coordinator.navigator.find_nearest_point(
        before.position_xz,
        true
    ).progress_m
    var obstacle_distance: float = minf(
        old_progress + 8.0,
        coordinator.navigator.total_length_m - 7.0
    )
    coordinator.place_obstacle(obstacle_distance, 4.0)
    var obstacle_world: Vector2 = coordinator.active_obstacle.world_position_xz
    var replacement: Array[Vector2] = _remaining_route_from_current(coordinator)
    var swapped: Dictionary = coordinator.replace_active_route(
        replacement,
        20,
        "replacement-pickup",
        PackedStringArray(["replacement-segment"])
    )
    var after: Dictionary = coordinator.get_service_snapshot()
    _expect(swapped.get("valid", false), "moving pickup leg accepts a compatible route swap")
    _expect(
        before.position_xz == after.position_xz
            && is_equal_approx(before.heading_rad, after.heading_rad)
            && is_equal_approx(before.speed_mps, after.speed_mps),
        "route swap preserves pose, heading, and speed"
    )
    _expect(
        swapped.get("obstacle_reprojected", false)
            && coordinator.active_obstacle.world_position_xz == obstacle_world
            && is_finite(coordinator.active_obstacle.distance_along_route_m),
        "active obstacle is re-projected from its world position after swap"
    )

    for tick in range(60):
        coordinator.step(FIXED_STEP, 1000 + tick)
    var repeated: Dictionary = coordinator.replace_active_route(
        _remaining_route_from_current(coordinator),
        20,
        "replacement-pickup-2"
    )
    _expect(repeated.get("valid", false), "repeated swaps in one active leg are accepted")
    coordinator.clear_obstacle()
    _expect(
        _drive_until(coordinator, PassengerJourney.BOARDING, 2000, 2000),
        "swapped pickup route is followed to certified arrival"
    )
    _expect(
        coordinator.confirm_boarding().get("valid", false),
        "the following destination leg still installs correctly"
    )
    _expect(
        _drive_until(coordinator, PassengerJourney.DROPOFF, MAX_TICKS, 5000),
        "destination leg completes after pickup route swap"
    )
    _expect(
        coordinator.confirm_passenger_exit().get("valid", false),
        "return leg still installs after swapped destination handoff"
    )

func _test_rejection_states() -> void:
    var idle := _new_coordinator()
    var idle_route: Array[Vector2] = [Vector2.ZERO, Vector2(0.0, 10.0)]
    var idle_result: Dictionary = idle.replace_active_route(
        idle_route
    )
    _expect(
        not idle_result.get("valid", false)
            && idle_result.get("failure_reason", "") == "route_swap_not_allowed_in_current_state",
        "route swap is rejected while idle"
    )

    var blocked := _new_coordinator()
    blocked.request_booking("sports_hall", "innovation_centre")
    blocked.interrupt_service("test_block")
    var blocked_route: Array[Vector2] = _remaining_route_from_current(blocked)
    var blocked_result: Dictionary = blocked.replace_active_route(
        blocked_route
    )
    _expect(
        not blocked_result.get("valid", false)
            && blocked_result.get("failure_reason", "")
                == "route_swap_not_allowed_in_current_state",
        "route swap is rejected while blocked"
    )

func _remaining_route_from_current(coordinator: RefCounted) -> Array[Vector2]:
    var position := Vector2(
        coordinator.dynamics.position_x_m,
        coordinator.dynamics.position_z_m
    )
    var current_navigation: Dictionary = coordinator.navigator.find_nearest_point(
        position,
        true
    )
    var progress: float = current_navigation.progress_m
    var route_length: float = coordinator.navigator.total_length_m
    var heading: float = coordinator.dynamics.heading_rad
    var forward := Vector2(sin(heading), cos(heading))
    var points: Array[Vector2] = [position, position + forward * 1.0]
    for fraction in [0.2, 0.45, 0.7, 0.9, 1.0]:
        var distance: float = lerpf(progress, route_length, float(fraction))
        var point: Vector2 = coordinator.navigator.get_point_at_distance(distance)
        if point.distance_to(points[points.size() - 1]) > 0.1:
            points.append(point)
    return points

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
