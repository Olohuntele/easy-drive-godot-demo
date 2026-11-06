extends SceneTree

const PurePursuitController = preload("res://scripts/pure_pursuit_controller.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    _test_straight_target()
    _test_right_and_left_targets()
    _test_rotated_vehicle_heading()
    _test_steering_saturation()
    _test_curved_route_lookahead()
    _test_near_zero_target()
    _test_target_behind_policy()
    _test_invalid_inputs()
    _test_deterministic_calculations()
    _test_zero_speed_behavior()
    _test_no_state_mutation()

    if failures == 0:
        print("PurePursuitController tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d PurePursuitController assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _new_controller():
    return PurePursuitController.new()

func _calculate(target: Vector2, heading: float = 0.0, position: Vector2 = Vector2.ZERO) -> Dictionary:
    var dynamics = VehicleDynamics.new()
    return _new_controller().calculate_steering_angle(
        position,
        heading,
        target,
        dynamics.wheelbase_m,
        dynamics.steering_limit_rad
    )

func _test_straight_target() -> void:
    var result: Dictionary = _calculate(Vector2(0.0, 8.0))
    _expect(result.valid, "target straight ahead produces a valid request")
    _expect(is_equal_approx(result.steering_angle_rad, 0.0), "target straight ahead produces zero steering")

func _test_right_and_left_targets() -> void:
    var right: Dictionary = _calculate(Vector2(2.0, 8.0))
    var left: Dictionary = _calculate(Vector2(-2.0, 8.0))
    _expect(right.valid && right.steering_angle_rad > 0.0, "target right produces positive steering")
    _expect(left.valid && left.steering_angle_rad < 0.0, "target left produces negative steering")

func _test_rotated_vehicle_heading() -> void:
    var heading: float = PI / 2.0
    var right_target: Vector2 = Vector2(cos(heading), -sin(heading)) + \
        Vector2(sin(heading), cos(heading)) * 5.0
    var result: Dictionary = _calculate(right_target, heading)
    _expect(result.valid, "rightward target is valid for a rotated vehicle")
    _expect(result.lateral_offset_m > 0.0, "rotated-frame transform identifies a rightward target")
    _expect(result.forward_offset_m > 0.0, "rotated-frame transform identifies a forward target")
    _expect(result.steering_angle_rad > 0.0, "rotated vehicle receives a positive right-turn request")

func _test_steering_saturation() -> void:
    var dynamics = VehicleDynamics.new()
    var controller = _new_controller()
    var right: Dictionary = controller.calculate_steering_angle(
        Vector2.ZERO, 0.0, Vector2(2.0, 0.1), dynamics.wheelbase_m, dynamics.steering_limit_rad
    )
    var left: Dictionary = controller.calculate_steering_angle(
        Vector2.ZERO, 0.0, Vector2(-2.0, 0.1), dynamics.wheelbase_m, dynamics.steering_limit_rad
    )
    _expect(is_equal_approx(right.steering_angle_rad, dynamics.steering_limit_rad), "positive steering saturates at VehicleDynamics limit")
    _expect(is_equal_approx(left.steering_angle_rad, -dynamics.steering_limit_rad), "negative steering saturates at VehicleDynamics limit")

func _test_curved_route_lookahead() -> void:
    var navigator = RouteNavigator.new()
    var route_set: bool = navigator.set_route(RouteNavigator.create_broad_bend_waypoints(), 24)
    var vehicle_position: Vector2 = Vector2(0.0, 30.0)
    var nearest: Dictionary = navigator.find_nearest_point(vehicle_position, true)
    var lookahead: Dictionary = navigator.get_lookahead_point(20.0)
    var dynamics = VehicleDynamics.new()
    var result: Dictionary = _new_controller().calculate_steering_angle(
        vehicle_position,
        0.0,
        lookahead.position_xz,
        dynamics.wheelbase_m,
        dynamics.steering_limit_rad
    )
    _expect(route_set && nearest.valid && lookahead.valid, "broad-bend navigator supplies a lookahead target")
    _expect(lookahead.position_xz.x > vehicle_position.x, "broad-bend lookahead lies to the right")
    _expect(result.valid && result.steering_angle_rad > 0.0, "broad-bend lookahead requests a positive turn")

func _test_near_zero_target() -> void:
    var near_target: Dictionary = _calculate(Vector2(0.0005, 0.0))
    var small_forward_target: Dictionary = _calculate(Vector2(0.0005, 0.01))
    _expect(not near_target.valid, "coincident near-zero target returns invalid neutral status")
    _expect(is_equal_approx(near_target.steering_angle_rad, 0.0), "near-zero target returns neutral steering")
    _expect(small_forward_target.valid, "small but resolvable forward target calculates safely")
    _expect(is_finite(small_forward_target.steering_angle_rad), "near-zero valid steering remains finite")

func _test_target_behind_policy() -> void:
    var behind: Dictionary = _calculate(Vector2(2.0, -5.0))
    var lateral_only: Dictionary = _calculate(Vector2(2.0, 0.0))
    _expect(not behind.valid && behind.status == "target_not_ahead", "target behind vehicle is rejected")
    _expect(is_equal_approx(behind.steering_angle_rad, 0.0), "behind target returns neutral steering")
    _expect(not lateral_only.valid, "target with no forward component is rejected")

func _test_invalid_inputs() -> void:
    var dynamics = VehicleDynamics.new()
    var controller = _new_controller()
    var invalid_wheelbase: Dictionary = controller.calculate_steering_angle(
        Vector2.ZERO, 0.0, Vector2(0.0, 5.0), 0.0, dynamics.steering_limit_rad
    )
    var invalid_limit: Dictionary = controller.calculate_steering_angle(
        Vector2.ZERO, 0.0, Vector2(0.0, 5.0), dynamics.wheelbase_m, INF
    )
    var invalid_position: Dictionary = controller.calculate_steering_angle(
        Vector2(INF, 0.0), 0.0, Vector2(0.0, 5.0), dynamics.wheelbase_m, dynamics.steering_limit_rad
    )
    var invalid_heading: Dictionary = controller.calculate_steering_angle(
        Vector2.ZERO, INF, Vector2(0.0, 5.0), dynamics.wheelbase_m, dynamics.steering_limit_rad
    )
    _expect(not invalid_wheelbase.valid && invalid_wheelbase.status == "invalid_wheelbase", "zero wheelbase is rejected safely")
    _expect(not invalid_limit.valid && invalid_limit.status == "invalid_steering_limit", "non-finite steering limit is rejected safely")
    _expect(not invalid_position.valid && invalid_position.status == "non_finite_position", "non-finite position is rejected safely")
    _expect(not invalid_heading.valid && invalid_heading.status == "non_finite_heading", "non-finite heading is rejected safely")

func _test_deterministic_calculations() -> void:
    var controller = _new_controller()
    var dynamics = VehicleDynamics.new()
    var first: Dictionary = controller.calculate_steering_angle(
        Vector2(1.0, 2.0), deg_to_rad(12.0), Vector2(4.0, 12.0),
        dynamics.wheelbase_m, dynamics.steering_limit_rad
    )
    var second: Dictionary = controller.calculate_steering_angle(
        Vector2(1.0, 2.0), deg_to_rad(12.0), Vector2(4.0, 12.0),
        dynamics.wheelbase_m, dynamics.steering_limit_rad
    )
    _expect(first.valid && second.valid, "repeated calculation inputs are valid")
    _expect(first.steering_angle_rad == second.steering_angle_rad, "identical inputs produce identical steering")
    _expect(first.curvature_per_m == second.curvature_per_m, "identical inputs produce identical curvature")

func _test_zero_speed_behavior() -> void:
    var dynamics = VehicleDynamics.new()
    var initial_position: Vector2 = Vector2(dynamics.position_x_m, dynamics.position_z_m)
    var initial_heading: float = dynamics.heading_rad
    var result: Dictionary = _new_controller().calculate_steering_angle(
        initial_position, initial_heading, Vector2(1.0, 5.0),
        dynamics.wheelbase_m, dynamics.steering_limit_rad
    )
    _expect(result.valid && is_finite(result.steering_angle_rad), "stationary vehicle gets a finite steering request")
    _expect(
        Vector2(dynamics.position_x_m, dynamics.position_z_m).is_equal_approx(initial_position),
        "steering calculation does not move a stopped vehicle"
    )
    _expect(is_equal_approx(dynamics.heading_rad, initial_heading), "steering calculation does not rotate a stopped vehicle")
    dynamics.step(0.0, 1.0 / 60.0, result.steering_angle_rad)
    _expect(is_equal_approx(dynamics.position_x_m, initial_position.x), "VehicleDynamics does not move laterally at zero speed")
    _expect(is_equal_approx(dynamics.position_z_m, initial_position.y), "VehicleDynamics does not move forward at zero speed")
    _expect(is_equal_approx(dynamics.heading_rad, initial_heading), "VehicleDynamics does not rotate at zero speed")

func _test_no_state_mutation() -> void:
    var dynamics = VehicleDynamics.new()
    dynamics.reset(4.0, 2.0, 1.0, deg_to_rad(15.0), deg_to_rad(3.0))
    var route = RouteNavigator.new()
    route.set_route(RouteNavigator.create_broad_bend_waypoints(), 12)
    route.find_nearest_point(Vector2(0.0, 10.0), true)
    var before_dynamics: Array[float] = [
        dynamics.position_x_m, dynamics.position_z_m, dynamics.heading_rad,
        dynamics.speed_mps, dynamics.actual_acceleration_mps2, dynamics.steering_rad
    ]
    var before_route_progress: float = route.progress_m
    var controller = _new_controller()
    var result: Dictionary = controller.calculate_steering_angle(
        Vector2(dynamics.position_x_m, dynamics.position_z_m),
        dynamics.heading_rad,
        Vector2(4.0, 12.0),
        dynamics.wheelbase_m,
        dynamics.steering_limit_rad
    )
    _expect(result.valid, "state-isolation request calculates successfully")
    _expect(dynamics.position_x_m == before_dynamics[0], "controller does not mutate dynamics X position")
    _expect(dynamics.position_z_m == before_dynamics[1], "controller does not mutate dynamics Z position")
    _expect(dynamics.heading_rad == before_dynamics[2], "controller does not mutate dynamics heading")
    _expect(dynamics.speed_mps == before_dynamics[3], "controller does not mutate dynamics speed")
    _expect(dynamics.actual_acceleration_mps2 == before_dynamics[4], "controller does not mutate dynamics acceleration")
    _expect(dynamics.steering_rad == before_dynamics[5], "controller returns a request without mutating actual steering")
    _expect(route.progress_m == before_route_progress, "controller does not mutate RouteNavigator progress")
