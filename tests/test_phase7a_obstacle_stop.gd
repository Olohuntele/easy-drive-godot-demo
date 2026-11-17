extends SceneTree

const CampusServiceCoordinator = preload("res://scripts/campus_service_coordinator.gd")
const PassengerJourney = preload("res://scripts/passenger_journey.gd")
const RouteFollowingController = preload("res://scripts/route_following_controller.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const RouteObstacle = preload("res://scripts/route_obstacle.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")

const FIXED_STEP: float = 1.0 / 60.0
const TOLERANCE_M: float = 0.2
const MAX_TICKS: int = 12000

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    _test_controller_obstacle_hold_and_resume()
    _test_coordinator_booking_with_obstacle_clear()
    if failures == 0:
        print("Phase 7A obstacle tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d Phase 7A assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _test_controller_obstacle_hold_and_resume() -> void:
    var navigator := RouteNavigator.new()
    navigator.set_route([Vector2.ZERO, Vector2(0.0, 100.0)], 20)
    var dynamics := VehicleDynamics.new()
    dynamics.max_speed = 10.0
    dynamics.max_acceleration = 3.5
    dynamics.max_braking = 6.0
    dynamics.reset()
    var controller := RouteFollowingController.new()
    var obstacle := RouteObstacle.new(35.0, 4.0, "test-route", "test-segment")

    var hold_seen := false
    var minimum_gap := INF
    var stop_progress := 0.0
    for tick in range(MAX_TICKS):
        var result: Dictionary = controller.update(
            dynamics,
            navigator,
            FIXED_STEP,
            false,
            obstacle
        )
        hold_seen = hold_seen || bool(result.get("obstacle_hold", false))
        dynamics.step(
            float(result.requested_acceleration_mps2),
            FIXED_STEP,
            float(result.requested_steering_rad)
        )
        if hold_seen:
            var progress: float = navigator.find_nearest_point(
                Vector2(dynamics.position_x_m, dynamics.position_z_m)
            ).progress_m
            minimum_gap = minf(minimum_gap, obstacle.distance_along_route_m - progress)
            if dynamics.speed_mps <= 0.1:
                stop_progress = progress
                break

    _expect(hold_seen, "active obstacle ahead creates a controller hold")
    _expect(
        dynamics.speed_mps <= 0.1,
        "obstacle hold brakes the shuttle to the arrival speed tolerance"
    )
    _expect(
        obstacle.distance_along_route_m - stop_progress
            >= RouteFollowingController.STOP_GAP_M - TOLERANCE_M,
        "shuttle stops at least STOP_GAP_M before the obstacle"
    )
    _expect(
        dynamics.max_speed * dynamics.max_speed / (2.0 * dynamics.max_braking)
            + RouteFollowingController.STOP_GAP_M
            < RouteFollowingController.DETECTION_RANGE_M,
        "detection range exceeds maximum-speed braking distance plus stop gap"
    )

    obstacle.active = false
    var clear_result := controller.update(
        dynamics,
        navigator,
        FIXED_STEP,
        false,
        obstacle
    )
    _expect(
        not clear_result.get("obstacle_hold", false)
            && clear_result.get("desired_speed_mps", 0.0) > 0.0,
        "clearing the obstacle restores the normal route target"
    )

    var behind := RouteObstacle.new(5.0, 4.0)
    var behind_result := controller.update(
        dynamics,
        navigator,
        FIXED_STEP,
        false,
        behind
    )
    _expect(
        not behind_result.get("obstacle_hold", false),
        "an obstacle behind the shuttle is ignored"
    )
    var no_obstacle_result := controller.update(
        dynamics,
        navigator,
        FIXED_STEP,
        false,
        null
    )
    _expect(
        not no_obstacle_result.get("obstacle_hold", false),
        "no obstacle preserves normal route-following behavior"
    )

func _test_coordinator_booking_with_obstacle_clear() -> void:
    var coordinator := CampusServiceCoordinator.new()
    var north_stop: Dictionary = coordinator.network.get_station_stop("north_gate")
    var launch_segment: Dictionary = coordinator.network.get_segment("north_to_east")
    var launch_points: Array = launch_segment.centerline_points
    var launch_tangent: Vector2 = launch_points[1] - launch_points[0]
    var initialized := coordinator.initialize_service(
        "north_gate",
        north_stop.position_xz,
        atan2(launch_tangent.x, launch_tangent.y)
    )
    _expect(initialized.get("valid", false), "coordinator initializes for obstacle journey")
    var booking := coordinator.request_booking("sports_hall", "innovation_centre")
    _expect(booking.get("valid", false), "booking installs a route for obstacle journey")

    var navigation: Dictionary = coordinator.navigator.find_nearest_point(
        Vector2(coordinator.dynamics.position_x_m, coordinator.dynamics.position_z_m)
    )
    var obstacle_distance: float = minf(
        navigation.progress_m + 12.0,
        coordinator.navigator.total_length_m - RouteFollowingController.STOP_GAP_M - 0.5
    )
    coordinator.place_obstacle(obstacle_distance, 4.0)
    var hold_seen := false
    for tick in range(MAX_TICKS):
        var step_result := coordinator.step(FIXED_STEP, tick + 1)
        hold_seen = hold_seen || bool(
            step_result.get("controller_result", {}).get("obstacle_hold", false)
        )
        if hold_seen && coordinator.dynamics.speed_mps <= 0.1:
            break
    _expect(hold_seen, "coordinator forwards obstacle hold without entering BLOCKED")
    _expect(
        coordinator.journey.state != PassengerJourney.BLOCKED
            && coordinator.dispatcher.has_active_booking(),
        "obstacle hold keeps the booking alive"
    )
    coordinator.clear_obstacle()
    var pickup_reached := _drive_until_state(
        coordinator,
        PassengerJourney.BOARDING,
        MAX_TICKS,
        100000
    )
    _expect(pickup_reached, "cleared obstacle permits pickup arrival")
    _expect(
        coordinator.confirm_boarding().get("valid", false),
        "boarding still installs the next leg after obstacle hold"
    )
    _expect(
        _drive_until_state(coordinator, PassengerJourney.DROPOFF, MAX_TICKS, 120000),
        "destination leg completes after obstacle hold"
    )
    _expect(
        coordinator.confirm_passenger_exit().get("valid", false),
        "passenger exit still installs the return leg"
    )
    _expect(
        _drive_until_state(coordinator, PassengerJourney.IDLE, MAX_TICKS, 140000),
        "full booking journey completes after obstacle clear"
    )

func _drive_until_state(
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
