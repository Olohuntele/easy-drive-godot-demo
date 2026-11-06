extends SceneTree

const RouteFollowingController = preload("res://scripts/route_following_controller.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")

const FIXED_STEP: float = 1.0 / 60.0
const MIRROR_WAYPOINTS: Array[Vector2] = [
    Vector2(0.0, 0.0),
    Vector2(0.0, 20.0),
    Vector2(-0.8, 40.0),
    Vector2(-4.5, 60.0),
    Vector2(-13.0, 78.0),
    Vector2(-27.0, 94.0)
]

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    _test_nominal_route_following()
    _test_endpoint_and_overshoot_safety()
    _test_invalid_route_localization_and_steering()
    _test_stationary_low_speed_and_manual_certification()
    _test_timestep_stability()
    _test_mirrored_route_steering()

    if failures == 0:
        print("RouteFollowingController tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d RouteFollowingController assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _new_broad_route() -> RouteNavigator:
    var navigator = RouteNavigator.new()
    navigator.set_route(RouteNavigator.create_broad_bend_waypoints(), 24)
    return navigator

func _test_nominal_route_following() -> void:
    var initial_navigator: RouteNavigator = _new_broad_route()
    initial_navigator.find_nearest_point(Vector2.ZERO, true)
    var initial_lookahead: Dictionary = initial_navigator.get_lookahead_point(
        RouteFollowingController.DEFAULT_LOOKAHEAD_DISTANCE_M
    )
    var initial_expected_lateral_m: float = initial_lookahead.position_xz.x
    var result: Dictionary = _run_closed_loop(
        RouteNavigator.create_broad_bend_waypoints(),
        FIXED_STEP
    )
    print(
        "NOMINAL METRICS: route_length=%.4f m, max_cte_after_5m=%.4f m, endpoint_error=%.4f m, final_speed=%.4f m/s, max_speed=%.4f m/s, brake_progress=%.4f m, steering_sign_changes=%d"
        % [
            result.route_length_m,
            result.max_cross_track_error_after_convergence_m,
            result.endpoint_error_m,
            result.final_speed_mps,
            result.max_speed_mps,
            result.braking_start_progress_m,
            result.steering_sign_changes
        ]
    )
    _expect(result.initial_status == "EN ROUTE", "route controller starts in EN ROUTE state")
    _expect(
        signf(result.initial_steering_rad) == signf(initial_expected_lateral_m),
        "initial steering sign matches the actual sampled route lookahead"
    )
    _expect(result.positive_steering_seen, "right-bending route requests positive steering")
    _expect(result.progress_monotonic, "nominal route progress never moves backward")
    _expect(
        result.max_cross_track_error_after_convergence_m <= 0.75,
        "nominal route cross-track error after initial convergence stays within 0.75 m"
    )
    _expect(result.steering_sign_changes <= 8, "nominal steering does not exhibit excessive sign oscillation")
    _expect(result.max_speed_mps <= 4.01, "route speed stays within the 4 m/s navigation target")
    _expect(result.braking_started, "endpoint braking begins before route completion")
    _expect(
        result.braking_start_progress_m < result.route_length_m,
        "endpoint braking begins with positive route distance remaining"
    )
    _expect(result.status == "ARRIVED", "nominal route reaches certified ARRIVED state")
    _expect(result.endpoint_error_m <= 1.0, "nominal terminal position is within 1 m of endpoint")
    _expect(result.final_speed_mps <= 0.1, "nominal final speed is at most 0.1 m/s")

func _test_endpoint_and_overshoot_safety() -> void:
    var navigator = RouteNavigator.new()
    navigator.set_route([Vector2.ZERO, Vector2(0.0, 10.0)], 20)
    var dynamics = VehicleDynamics.new()
    dynamics.reset(10.0, 0.2)
    var controller = RouteFollowingController.new()
    var endpoint_moving: Dictionary = controller.update(dynamics, navigator, FIXED_STEP)
    _expect(endpoint_moving.status != "ARRIVED", "endpoint proximity at nonzero speed is not arrival")
    _expect(endpoint_moving.arrival_certified == false, "moving endpoint state is not certified")

    var overshoot_dynamics = VehicleDynamics.new()
    overshoot_dynamics.reset(11.25, 0.0)
    var overshoot_navigator = RouteNavigator.new()
    overshoot_navigator.set_route([Vector2.ZERO, Vector2(0.0, 10.0)], 20)
    var overshoot_controller = RouteFollowingController.new()
    var overshoot: Dictionary = overshoot_controller.update(
        overshoot_dynamics,
        overshoot_navigator,
        FIXED_STEP
    )
    _expect(overshoot.status == "MISSED DESTINATION", "endpoint overshoot is reported as missed")
    _expect(not overshoot.arrival_certified, "overshoot cannot be reported as successful arrival")
    _expect(overshoot.requested_acceleration_mps2 < 0.0, "overshoot requests braking rather than position clamping")
    _expect(is_equal_approx(overshoot_dynamics.position_z_m, 11.25), "controller does not teleport or clamp vehicle position")

func _test_invalid_route_localization_and_steering() -> void:
    var dynamics = VehicleDynamics.new()
    var controller = RouteFollowingController.new()
    var empty_route = RouteNavigator.new()
    var invalid_route: Dictionary = controller.update(dynamics, empty_route, FIXED_STEP)
    _expect(invalid_route.status == "SAFE STOP / INVALID NAVIGATION", "missing route requests safe stop")
    _expect(invalid_route.requested_acceleration_mps2 <= 0.0, "invalid route never requests acceleration")

    var lost_navigator = _new_broad_route()
    var lost_dynamics = VehicleDynamics.new()
    lost_dynamics.reset(30.0, 0.0, 20.0)
    var lost_result: Dictionary = RouteFollowingController.new().update(
        lost_dynamics, lost_navigator, FIXED_STEP
    )
    _expect(lost_result.reason == "route_localization_lost", "off-route vehicle is detected as lost localization")
    _expect(lost_result.status == "SAFE STOP / INVALID NAVIGATION", "lost localization requests safe-stop status")
    _expect(lost_result.requested_acceleration_mps2 < 0.0, "lost localization requests available braking")

    var reverse_facing_navigator = RouteNavigator.new()
    reverse_facing_navigator.set_route([Vector2.ZERO, Vector2(0.0, 10.0)], 20)
    var reverse_facing_dynamics = VehicleDynamics.new()
    reverse_facing_dynamics.reset(5.0, 1.0, 0.0, PI)
    var invalid_steering: Dictionary = RouteFollowingController.new().update(
        reverse_facing_dynamics, reverse_facing_navigator, FIXED_STEP
    )
    _expect(
        invalid_steering.reason.begins_with("invalid_pure_pursuit:"),
        "invalid behind-vehicle Pure Pursuit target is rejected"
    )
    _expect(invalid_steering.requested_steering_rad == 0.0, "invalid steering request is neutral")
    _expect(invalid_steering.requested_acceleration_mps2 < 0.0, "invalid steering request brakes safely")

func _test_stationary_low_speed_and_manual_certification() -> void:
    var dynamics = VehicleDynamics.new()
    var navigator = _new_broad_route()
    var controller = RouteFollowingController.new()
    var initial_position := Vector2(dynamics.position_x_m, dynamics.position_z_m)
    var initial_heading: float = dynamics.heading_rad
    var result: Dictionary = controller.update(dynamics, navigator, FIXED_STEP)
    _expect(is_finite(result.requested_steering_rad), "stationary steering output is finite")
    _expect(is_finite(result.requested_acceleration_mps2), "stationary longitudinal output is finite")
    _expect(
        Vector2(dynamics.position_x_m, dynamics.position_z_m).is_equal_approx(initial_position),
        "controller update does not directly move the vehicle"
    )
    _expect(is_equal_approx(dynamics.heading_rad, initial_heading), "controller update does not directly rotate the vehicle")
    _expect(result.requested_acceleration_mps2 > 0.0, "controller requests acceleration from a safe stop")

    var manual_navigator = RouteNavigator.new()
    manual_navigator.set_route([Vector2.ZERO, Vector2(0.0, 5.0)], 20)
    var manual_dynamics = VehicleDynamics.new()
    manual_dynamics.reset(5.0, 0.0)
    var manual_controller = RouteFollowingController.new()
    var manual_result: Dictionary = manual_controller.update(
        manual_dynamics, manual_navigator, FIXED_STEP, true
    )
    _expect(manual_result.status == "EN ROUTE", "manual intervention prevents certified arrival")
    _expect(not manual_result.arrival_certified, "manual intervention latch prevents arrival certification")
    _expect(manual_result.reason == "manual_intervention_seen", "manual intervention reason appears at endpoint")

    var overspeed_dynamics = VehicleDynamics.new()
    overspeed_dynamics.reset(0.0, 6.0)
    var overspeed_navigator = RouteNavigator.new()
    overspeed_navigator.set_route([Vector2.ZERO, Vector2(0.0, 100.0)], 20)
    var overspeed_result: Dictionary = RouteFollowingController.new().update(
        overspeed_dynamics, overspeed_navigator, FIXED_STEP
    )
    _expect(
        overspeed_result.requested_acceleration_mps2 < 0.0,
        "vehicle above the navigation speed target is actively slowed"
    )

func _test_timestep_stability() -> void:
    var at_60: Dictionary = _run_closed_loop(
        RouteNavigator.create_broad_bend_waypoints(),
        1.0 / 60.0
    )
    var at_120: Dictionary = _run_closed_loop(
        RouteNavigator.create_broad_bend_waypoints(),
        1.0 / 120.0
    )
    _expect(at_60.status == "ARRIVED" && at_120.status == "ARRIVED", "both fixed timesteps complete the route")
    _expect(
        absf(at_60.endpoint_error_m - at_120.endpoint_error_m) <= 0.25,
        "terminal position remains stable between 1/60 and 1/120 seconds"
    )
    _expect(
        absf(at_60.max_cross_track_error_after_convergence_m - at_120.max_cross_track_error_after_convergence_m) <= 0.25,
        "lateral tracking remains stable between 1/60 and 1/120 seconds"
    )

func _test_mirrored_route_steering() -> void:
    var mirrored: Dictionary = _run_closed_loop(MIRROR_WAYPOINTS, FIXED_STEP)
    _expect(mirrored.negative_steering_seen, "left-bending mirrored route requests negative steering")
    _expect(mirrored.status == "ARRIVED", "mirrored left-bending route reaches ARRIVED state")
    _expect(mirrored.endpoint_error_m <= 1.0, "mirrored route stops within 1 m of endpoint")
    _expect(mirrored.final_speed_mps <= 0.1, "mirrored route final speed is at most 0.1 m/s")

func _run_closed_loop(waypoints: Array[Vector2], delta: float) -> Dictionary:
    var navigator = RouteNavigator.new()
    var route_valid: bool = navigator.set_route(waypoints, 36)
    var dynamics = VehicleDynamics.new()
    dynamics.reset(waypoints[0].y, 0.0, waypoints[0].x)
    var controller = RouteFollowingController.new()
    var max_cross_track_error_after_convergence_m: float = 0.0
    var max_speed_mps: float = 0.0
    var initial_steering_rad: float = 0.0
    var positive_steering_seen: bool = false
    var negative_steering_seen: bool = false
    var previous_steering_sign: int = 0
    var steering_sign_changes: int = 0
    var previous_progress_m: float = 0.0
    var progress_monotonic: bool = true
    var braking_started: bool = false
    var braking_start_progress_m: float = -1.0
    var final_result: Dictionary = {}
    var max_steps: int = int(180.0 / delta)

    if not route_valid:
        return {"status": "INVALID ROUTE"}

    for step_index in range(max_steps):
        var result: Dictionary = controller.update(dynamics, navigator, delta)
        if step_index == 0:
            initial_steering_rad = result.requested_steering_rad
        if result.requested_steering_rad > deg_to_rad(0.5):
            positive_steering_seen = true
        elif result.requested_steering_rad < -deg_to_rad(0.5):
            negative_steering_seen = true
        var steering_sign: int = 1 if result.requested_steering_rad > deg_to_rad(0.5) else (
            -1 if result.requested_steering_rad < -deg_to_rad(0.5) else 0
        )
        if steering_sign != 0:
            if previous_steering_sign != 0 && steering_sign != previous_steering_sign:
                steering_sign_changes += 1
            previous_steering_sign = steering_sign

        var progress_m: float = result.route_progress_m
        if progress_m + 0.00001 < previous_progress_m:
            progress_monotonic = false
        previous_progress_m = progress_m
        if progress_m >= 5.0:
            max_cross_track_error_after_convergence_m = maxf(
                max_cross_track_error_after_convergence_m,
                absf(result.cross_track_error_m)
            )
        if result.braking && not braking_started:
            braking_started = true
            braking_start_progress_m = progress_m
        dynamics.step(
            result.requested_acceleration_mps2,
            delta,
            result.requested_steering_rad
        )
        max_speed_mps = maxf(max_speed_mps, dynamics.speed_mps)
        final_result = result
        if result.status == "ARRIVED" || result.status == "MISSED DESTINATION" \
            || result.status == "SAFE STOP / INVALID NAVIGATION":
            break

    var endpoint: Vector2 = waypoints[waypoints.size() - 1]
    return {
        "initial_status": "EN ROUTE" if route_valid else "INVALID ROUTE",
        "initial_steering_rad": initial_steering_rad,
        "positive_steering_seen": positive_steering_seen,
        "negative_steering_seen": negative_steering_seen,
        "progress_monotonic": progress_monotonic,
        "max_cross_track_error_after_convergence_m": max_cross_track_error_after_convergence_m,
        "steering_sign_changes": steering_sign_changes,
        "max_speed_mps": max_speed_mps,
        "braking_started": braking_started,
        "braking_start_progress_m": braking_start_progress_m,
        "status": final_result.get("status", "TIMEOUT"),
        "endpoint_error_m": Vector2(dynamics.position_x_m, dynamics.position_z_m).distance_to(endpoint),
        "final_speed_mps": dynamics.speed_mps,
        "final_position_xz": Vector2(dynamics.position_x_m, dynamics.position_z_m),
        "route_length_m": navigator.total_length_m
    }
