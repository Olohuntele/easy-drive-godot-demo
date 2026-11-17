extends SceneTree

const CampusMultiRouteNetwork = preload("res://scripts/campus_multi_route_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const CampusServiceCoordinator = preload("res://scripts/campus_service_coordinator.gd")
const OffsetPathBuilder = preload("res://scripts/offset_path_builder.gd")
const PassengerJourney = preload("res://scripts/passenger_journey.gd")

const FIXED_STEP: float = 1.0 / 60.0
const MAX_TICKS: int = 18000

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    await _test_valid_lane_change()
    _test_builder_rejections()
    await _test_default_road_rejects_bypass()
    await _test_destination_and_return_bypasses()
    await _test_hold_cancellation_and_repeat_booking()
    if failures == 0:
        print("Phase 7C lane change tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d Phase 7C assertions failed." % [failures, assertions])
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

func _test_valid_lane_change() -> void:
    var coordinator := _new_coordinator()
    coordinator.road_width_m = 7.0
    _expect(
        coordinator.request_booking("sports_hall", "innovation_centre").get("valid", false),
        "lane-change scenario installs a pickup route"
    )
    for tick in range(90):
        coordinator.step(FIXED_STEP, tick + 1)
    var progress: float = coordinator.navigator.find_nearest_point(
        Vector2(coordinator.dynamics.position_x_m, coordinator.dynamics.position_z_m),
        true
    ).progress_m
    var obstacle_distance: float = minf(
        progress + 18.0,
        coordinator.navigator.total_length_m - 11.0 - 0.5
    )
    coordinator.place_obstacle(obstacle_distance)
    var obstacle_center: Vector2 = coordinator.navigator.get_point_at_distance(
        obstacle_distance
    )
    var obstacle_tangent: Vector2 = _route_tangent(
        coordinator.navigator,
        obstacle_distance
    )

    var changed_lane := false
    for tick in range(MAX_TICKS):
        coordinator.step(FIXED_STEP, 1000 + tick)
        if coordinator.lane_change_active:
            changed_lane = true
            break
        if coordinator.journey.state == PassengerJourney.BLOCKED:
            break
    _expect(changed_lane, "a clear offset path is installed after the hold")
    _expect(
        coordinator.lane_change_active
            && is_equal_approx(
                coordinator.offset_path_builder.LATERAL_MARGIN_M,
                OffsetPathBuilder.LATERAL_MARGIN_M
            ),
        "lane-change state exposes the configured offset"
    )
    var minimum_clearance_m: float = INF
    var maximum_tracking_error_m: float = 0.0
    for tick in range(6000):
        coordinator.step(FIXED_STEP, 50000 + tick)
        minimum_clearance_m = minf(
            minimum_clearance_m,
            _footprint_clearance(
                Vector2(coordinator.dynamics.position_x_m, coordinator.dynamics.position_z_m),
                coordinator.dynamics.heading_rad,
                obstacle_center,
                obstacle_tangent,
                4.0,
                coordinator.active_obstacle.width_m
            )
        )
        maximum_tracking_error_m = maxf(
            maximum_tracking_error_m,
            coordinator.maximum_cross_track_error_m
        )
        if coordinator.journey.state == PassengerJourney.BOARDING:
            break
    coordinator.clear_obstacle()
    print(
        "closed-loop bypass: min clearance=%.3f m, max lateral tracking error=%.3f m"
        % [minimum_clearance_m, maximum_tracking_error_m]
    )
    _expect(
        minimum_clearance_m >= OffsetPathBuilder.LATERAL_MARGIN_M - 0.05,
        "closed-loop bypass footprint preserves lateral clearance"
    )
    _expect(
        _drive_until(coordinator, PassengerJourney.BOARDING, 6000, 60000),
        "valid bypass rejoins and reaches pickup"
    )

func _test_default_road_rejects_bypass() -> void:
    var coordinator := _new_coordinator()
    coordinator.road_width_m = 4.0
    _expect(
        coordinator.request_booking("sports_hall", "innovation_centre").get("valid", false),
        "default-width scenario installs a pickup route"
    )
    for tick in range(90):
        coordinator.step(FIXED_STEP, 70000 + tick)
    var progress: float = coordinator.navigator.find_nearest_point(
        Vector2(coordinator.dynamics.position_x_m, coordinator.dynamics.position_z_m),
        true
    ).progress_m
    coordinator.place_obstacle(minf(
        progress + 18.0,
        coordinator.navigator.total_length_m - 11.5
    ))
    var held := false
    for tick in range(MAX_TICKS):
        coordinator.step(FIXED_STEP, 71000 + tick)
        held = held || coordinator.no_way_hold_active
        if held:
            break
    _expect(held, "4 m road rejects the bypass and produces a no-way hold")
    _expect(not coordinator.lane_change_active, "4 m road never installs a bypass")
    var held_position := Vector2(
        coordinator.dynamics.position_x_m,
        coordinator.dynamics.position_z_m
    )
    var held_speed: float = coordinator.dynamics.speed_mps
    var held_stable := true
    for tick in range(600):
        coordinator.step(FIXED_STEP, 72000 + tick)
        held_stable = held_stable \
            && held_position.distance_to(Vector2(
                coordinator.dynamics.position_x_m,
                coordinator.dynamics.position_z_m
            )) <= 0.05 \
            && absf(coordinator.dynamics.speed_mps) <= 0.1
    _expect(
        held_stable && held_speed <= 0.1,
        "no-way hold keeps position and speed stable over a long interval"
    )
    coordinator.clear_obstacle()

func _test_destination_and_return_bypasses() -> void:
    var coordinator := _new_coordinator()
    coordinator.road_width_m = 7.0
    _expect(
        coordinator.request_booking("sports_hall", "innovation_centre").get("valid", false),
        "destination/return scenario installs a booking"
    )
    for tick in range(90):
        coordinator.step(FIXED_STEP, 80000 + tick)
    _drive_until(coordinator, PassengerJourney.BOARDING, 6000, 81000)
    _expect(coordinator.confirm_boarding().get("valid", false), "boarding installs destination leg")
    var destination_bypass := _force_stationary_bypass(coordinator, 82000)
    _expect(destination_bypass, "destination leg accepts a bypass")
    coordinator.clear_obstacle()
    _drive_until(coordinator, PassengerJourney.DROPOFF, 8000, 83000)
    _expect(coordinator.confirm_passenger_exit().get("valid", false), "exit installs return leg")
    var return_bypass := _force_stationary_bypass(coordinator, 84000)
    _expect(return_bypass, "return leg accepts a bypass")
    coordinator.clear_obstacle()

func _test_hold_cancellation_and_repeat_booking() -> void:
    var coordinator := _new_coordinator()
    coordinator.road_width_m = 7.0
    coordinator.request_booking("sports_hall", "innovation_centre")
    for tick in range(90):
        coordinator.step(FIXED_STEP, 90000 + tick)
    var progress: float = coordinator.navigator.find_nearest_point(
        Vector2(coordinator.dynamics.position_x_m, coordinator.dynamics.position_z_m),
        true
    ).progress_m
    coordinator.place_obstacle(minf(progress + 18.0, coordinator.navigator.total_length_m - 11.5))
    for tick in range(90):
        coordinator.step(FIXED_STEP, 90100 + tick)
    coordinator.clear_obstacle()
    _expect(
        not coordinator.no_way_hold_active && not coordinator.lane_change_active,
        "clearing during the two-second wait cancels the bypass"
    )
    _expect(
        _drive_until(coordinator, PassengerJourney.BOARDING, 6000, 90200),
        "wait cancellation resumes the unobstructed pickup"
    )
    coordinator.confirm_boarding()
    _drive_until(coordinator, PassengerJourney.DROPOFF, 8000, 97000)
    coordinator.confirm_passenger_exit()
    _expect(
        _drive_until(coordinator, PassengerJourney.IDLE, 8000, 105000),
        "first bypass booking returns shuttle to service"
    )
    _expect(
        coordinator.request_booking("library", "north_gate").get("valid", false),
        "repeat booking after a bypass is accepted"
    )
    _expect(
        _drive_until(coordinator, PassengerJourney.BOARDING, 6000, 100000),
        "valid bypass rejoins and reaches pickup"
    )

func _test_builder_rejections() -> void:
    var coordinator := _new_coordinator()
    coordinator.request_booking("sports_hall", "innovation_centre")
    for tick in range(90):
        coordinator.step(FIXED_STEP, 20000 + tick)
    var builder := OffsetPathBuilder.new()
    var position := Vector2(coordinator.dynamics.position_x_m, coordinator.dynamics.position_z_m)
    var moving_obstacle: Dictionary = coordinator.place_obstacle(
        coordinator.navigator.find_nearest_point(position, true).progress_m + 10.0,
        3.0
    )
    var moving_result: Dictionary = builder.build(
        coordinator.navigator,
        coordinator.network,
        position,
        coordinator.dynamics.heading_rad,
        1.0,
        coordinator.active_obstacle
    )
    _expect(
        moving_obstacle.get("valid", false)
            && not moving_result.get("valid", false)
            && moving_result.get("failure_reason", "") == "vehicle_not_stationary",
        "bypass is rejected while the shuttle is moving"
    )

    var endpoint_obstacle_distance: float = coordinator.navigator.total_length_m - 8.0
    coordinator.place_obstacle(endpoint_obstacle_distance, 3.0)
    var endpoint_result: Dictionary = builder.build(
        coordinator.navigator,
        coordinator.network,
        position,
        coordinator.dynamics.heading_rad,
        0.0,
        coordinator.active_obstacle
    )
    _expect(
        not endpoint_result.get("valid", false)
            && endpoint_result.get("failure_reason", "")
                == "rejoin_too_close_to_leg_end",
        "bypass is rejected when rejoin would be too close to the leg end"
    )
    var junction: Dictionary = coordinator.network.get_node("junction_bypass_mid")
    var junction_progress: float = coordinator.navigator.find_nearest_point(
        junction.position_xz,
        true
    ).progress_m
    coordinator.dynamics.speed_mps = 0.0
    coordinator.place_obstacle(junction_progress - 8.5, 3.0)
    var junction_result: Dictionary = builder.build(
        coordinator.navigator,
        coordinator.network,
        position,
        coordinator.dynamics.heading_rad,
        0.0,
        coordinator.active_obstacle,
        7.0
    )
    _expect(
        not junction_result.get("valid", false)
            && junction_result.get("failure_reason", "")
                == "rejoin_inside_restricted_zone",
        "bypass is rejected when rejoin is near a junction"
    )
    coordinator.place_obstacle(
        coordinator.navigator.find_nearest_point(position, true).progress_m + 18.0,
        3.0
    )
    var road_result: Dictionary = builder.build(
        coordinator.navigator,
        coordinator.network,
        position,
        coordinator.dynamics.heading_rad,
        0.0,
        coordinator.active_obstacle,
        4.0
    )
    _expect(
        not road_result.get("valid", false)
            && road_result.get("failure_reason", "")
                == "offset_exceeds_road_clearance",
        "bypass is rejected when the path would leave the road"
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

func _force_stationary_bypass(coordinator: RefCounted, tick_offset: int) -> bool:
    var position := Vector2(coordinator.dynamics.position_x_m, coordinator.dynamics.position_z_m)
    var progress: float = coordinator.navigator.find_nearest_point(position, true).progress_m
    var obstacle_distance: float = minf(
        progress + 18.0,
        coordinator.navigator.total_length_m - 11.5
    )
    coordinator.dynamics.speed_mps = 0.0
    coordinator.place_obstacle(obstacle_distance)
    var build: Dictionary = coordinator.offset_path_builder.build(
        coordinator.navigator,
        coordinator.network,
        position,
        coordinator.dynamics.heading_rad,
        0.0,
        coordinator.active_obstacle,
        coordinator.road_width_m,
        coordinator.dynamics.wheelbase_m,
        coordinator.dynamics.steering_limit_rad
    )
    if not build.get("valid", false):
        return false
    var swap: Dictionary = coordinator.replace_active_route(
        build.centerline_points,
        coordinator.ROUTE_SAMPLES_PER_SEGMENT,
        "lane-bypass-test",
        PackedStringArray(["lane-bypass-test"])
    )
    if swap.get("valid", false):
        coordinator.lane_change_active = true
    return swap.get("valid", false)

func _route_tangent(navigator: RefCounted, distance_m: float) -> Vector2:
    return (
        navigator.get_point_at_distance(minf(distance_m + 0.25, navigator.total_length_m))
        - navigator.get_point_at_distance(maxf(distance_m - 0.25, 0.0))
    ).normalized()

func _footprint_clearance(
    center: Vector2,
    heading_rad: float,
    obstacle_center: Vector2,
    obstacle_tangent: Vector2,
    obstacle_length_m: float,
    obstacle_width_m: float
) -> float:
    var forward := Vector2(sin(heading_rad), cos(heading_rad))
    var right := Vector2(forward.y, -forward.x)
    var shuttle := _rectangle(center, forward, right, 3.7, 1.8)
    var obstacle_right := Vector2(obstacle_tangent.y, -obstacle_tangent.x)
    var obstacle := _rectangle(
        obstacle_center,
        obstacle_tangent,
        obstacle_right,
        obstacle_length_m,
        obstacle_width_m
    )
    var minimum_overlap := INF
    var maximum_gap := 0.0
    for axis in [
        (shuttle[1] - shuttle[0]).normalized(),
        (shuttle[0] - shuttle[3]).normalized(),
        (obstacle[1] - obstacle[0]).normalized(),
        (obstacle[0] - obstacle[3]).normalized()
    ]:
        var a := _project(shuttle, axis)
        var b := _project(obstacle, axis)
        var gap := maxf(b.x - a.y, a.x - b.y)
        if gap > 0.0:
            maximum_gap = maxf(maximum_gap, gap)
        else:
            minimum_overlap = minf(minimum_overlap, minf(a.y, b.y) - maxf(a.x, b.x))
    return maximum_gap if maximum_gap > 0.0 else -minimum_overlap

func _rectangle(
    center: Vector2,
    forward: Vector2,
    right: Vector2,
    length_m: float,
    width_m: float
) -> Array[Vector2]:
    return [
        center + forward * length_m * 0.5 + right * width_m * 0.5,
        center + forward * length_m * 0.5 - right * width_m * 0.5,
        center - forward * length_m * 0.5 - right * width_m * 0.5,
        center - forward * length_m * 0.5 + right * width_m * 0.5
    ]

func _project(points: Array[Vector2], axis: Vector2) -> Vector2:
    var minimum := INF
    var maximum := -INF
    for point in points:
        var value: float = point.dot(axis)
        minimum = minf(minimum, value)
        maximum = maxf(maximum, value)
    return Vector2(minimum, maximum)
