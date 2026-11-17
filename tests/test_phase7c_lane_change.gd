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
        progress + 12.0,
        coordinator.navigator.total_length_m - 18.0
    )
    coordinator.place_obstacle(obstacle_distance, 3.0)

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
                coordinator.offset_path_builder.LANE_OFFSET_M,
                OffsetPathBuilder.LANE_OFFSET_M
            ),
        "lane-change state exposes the configured offset"
    )
    coordinator.clear_obstacle()
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
