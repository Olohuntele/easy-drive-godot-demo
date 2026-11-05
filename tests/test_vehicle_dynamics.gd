extends SceneTree

const Dynamics = preload("res://scripts/vehicle_dynamics.gd")

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    _test_positive_acceleration()
    _test_speed_limit()
    _test_braking_and_no_reverse()
    _test_zero_acceleration()
    _test_reset()
    _test_fixed_timestep_stability()
    _test_straight_motion()
    _test_left_and_right_turn_signs()
    _test_steering_saturation()
    _test_zero_speed_steering()
    _test_braking_while_turning()
    _test_steering_timestep_stability()
    _test_full_state_reset()

    if failures == 0:
        print("VehicleDynamics tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d VehicleDynamics assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _test_positive_acceleration() -> void:
    var dynamics = Dynamics.new()
    dynamics.step(2.0, 0.5)
    _expect(is_equal_approx(dynamics.speed_mps, 1.0), "positive acceleration increases speed")
    _expect(is_equal_approx(dynamics.position_m, 0.25), "acceleration integrates position")
    _expect(is_equal_approx(dynamics.actual_acceleration_mps2, 2.0), "actual acceleration is reported")

func _test_speed_limit() -> void:
    var dynamics = Dynamics.new()
    for i in range(600):
        dynamics.step(3.5, 1.0 / 60.0)
    _expect(dynamics.speed_mps <= 10.0, "speed never exceeds 10 m/s")
    _expect(is_equal_approx(dynamics.speed_mps, 10.0), "continuous acceleration reaches the speed limit")
    _expect(is_equal_approx(dynamics.actual_acceleration_mps2, 0.0), "actual acceleration is zero at a steady speed limit")

func _test_braking_and_no_reverse() -> void:
    var dynamics = Dynamics.new()
    dynamics.reset(0.0, 4.0)
    dynamics.step(-6.0, 0.1)
    _expect(is_equal_approx(dynamics.speed_mps, 3.4), "braking reduces speed")
    dynamics.step(-6.0, 1.0)
    _expect(is_equal_approx(dynamics.speed_mps, 0.0), "braking stops the vehicle")
    _expect(dynamics.position_m >= 0.0, "braking does not move backwards")

func _test_zero_acceleration() -> void:
    var dynamics = Dynamics.new()
    dynamics.reset(2.0, 3.0)
    dynamics.step(0.0, 0.5)
    _expect(is_equal_approx(dynamics.speed_mps, 3.0), "zero acceleration does not increase speed")
    _expect(is_equal_approx(dynamics.position_m, 3.5), "zero acceleration preserves constant-speed motion")

func _test_reset() -> void:
    var dynamics = Dynamics.new()
    dynamics.step(2.0, 0.5)
    dynamics.reset(12.0, 4.0)
    _expect(is_equal_approx(dynamics.position_m, 12.0), "reset restores initial position")
    _expect(is_equal_approx(dynamics.speed_mps, 4.0), "reset restores initial speed")
    _expect(is_equal_approx(dynamics.actual_acceleration_mps2, 0.0), "reset clears actual acceleration")

func _test_fixed_timestep_stability() -> void:
    var reference = _run_acceleration_for_two_seconds(1.0 / 30.0)
    _expect(is_equal_approx(reference.position_m, 4.0), "fixed-step run reaches expected position")
    _expect(is_equal_approx(reference.speed_mps, 4.0), "fixed-step run reaches expected speed")
    for timestep in [1.0 / 60.0, 1.0 / 120.0]:
        var result = _run_acceleration_for_two_seconds(timestep)
        _expect(is_equal_approx(result.position_m, reference.position_m), "position is stable at timestep %.6f" % timestep)
        _expect(is_equal_approx(result.speed_mps, reference.speed_mps), "speed is stable at timestep %.6f" % timestep)

func _test_straight_motion() -> void:
    var dynamics = Dynamics.new()
    dynamics.reset(0.0, 4.0)
    dynamics.step(0.0, 1.0, 0.0)
    _expect(is_equal_approx(dynamics.position_x_m, 0.0), "straight motion keeps X constant")
    _expect(is_equal_approx(dynamics.position_z_m, 4.0), "straight motion advances along +Z")
    _expect(is_equal_approx(dynamics.position_m, dynamics.position_z_m), "legacy longitudinal position aliases Z")
    _expect(is_equal_approx(dynamics.heading_rad, 0.0), "straight motion preserves heading")

func _test_left_and_right_turn_signs() -> void:
    var left = Dynamics.new()
    left.reset(0.0, 4.0)
    left.step(0.0, 1.0, -deg_to_rad(20.0))
    _expect(left.position_x_m < 0.0, "negative steering turns toward -X")
    _expect(left.heading_rad < 0.0, "negative steering produces negative heading")

    var right = Dynamics.new()
    right.reset(0.0, 4.0)
    right.step(0.0, 1.0, deg_to_rad(20.0))
    _expect(right.position_x_m > 0.0, "positive steering turns toward +X")
    _expect(right.heading_rad > 0.0, "positive steering produces positive heading")

func _test_steering_saturation() -> void:
    var dynamics = Dynamics.new()
    dynamics.step(0.0, 0.1, deg_to_rad(90.0))
    _expect(is_equal_approx(dynamics.steering_rad, dynamics.steering_limit_rad), "positive steering saturates at the configured limit")
    dynamics.step(0.0, 0.1, deg_to_rad(-90.0))
    _expect(is_equal_approx(dynamics.steering_rad, -dynamics.steering_limit_rad), "negative steering saturates at the configured limit")

func _test_zero_speed_steering() -> void:
    var dynamics = Dynamics.new()
    dynamics.step(0.0, 0.5, deg_to_rad(25.0))
    _expect(is_equal_approx(dynamics.position_x_m, 0.0), "steering at zero speed does not move X")
    _expect(is_equal_approx(dynamics.position_z_m, 0.0), "steering at zero speed does not move Z")
    _expect(is_equal_approx(dynamics.heading_rad, 0.0), "steering at zero speed does not rotate the vehicle")
    _expect(is_equal_approx(dynamics.steering_rad, deg_to_rad(25.0)), "requested steering is represented at zero speed")

func _test_braking_while_turning() -> void:
    var dynamics = Dynamics.new()
    dynamics.reset(0.0, 4.0)
    var steering: float = deg_to_rad(15.0)
    for i in range(10):
        dynamics.step(-6.0, 0.1, steering)
    _expect(is_equal_approx(dynamics.speed_mps, 0.0), "braking while turning stops without reverse")
    _expect(dynamics.position_x_m > 0.0, "braking while turning follows the +X arc")
    _expect(dynamics.heading_rad > 0.0, "heading changes while the vehicle is moving under braking")
    var stopped_position: Vector2 = Vector2(dynamics.position_x_m, dynamics.position_z_m)
    var stopped_heading: float = dynamics.heading_rad
    dynamics.step(-6.0, 0.1, steering)
    _expect(is_equal_approx(dynamics.position_x_m, stopped_position.x), "stopped vehicle does not continue moving laterally")
    _expect(is_equal_approx(dynamics.position_z_m, stopped_position.y), "stopped vehicle does not continue moving longitudinally")
    _expect(is_equal_approx(dynamics.heading_rad, stopped_heading), "stopped vehicle does not continue turning")

func _test_steering_timestep_stability() -> void:
    var reference = _run_turn_for_two_seconds(1.0 / 30.0)
    for timestep in [1.0 / 60.0, 1.0 / 120.0]:
        var result = _run_turn_for_two_seconds(timestep)
        _expect(absf(result.position_x_m - reference.position_x_m) <= 0.0001, "X position is stable at timestep %.6f" % timestep)
        _expect(absf(result.position_z_m - reference.position_z_m) <= 0.0001, "Z position is stable at timestep %.6f" % timestep)
        _expect(absf(result.heading_rad - reference.heading_rad) <= 0.0001, "heading is stable at timestep %.6f" % timestep)

func _test_full_state_reset() -> void:
    var dynamics = Dynamics.new()
    dynamics.step(2.0, 0.5, deg_to_rad(10.0))
    dynamics.reset(12.0, 4.0, -3.0, deg_to_rad(20.0), deg_to_rad(-5.0))
    _expect(is_equal_approx(dynamics.position_x_m, -3.0), "reset restores X position")
    _expect(is_equal_approx(dynamics.position_z_m, 12.0), "reset restores Z position")
    _expect(is_equal_approx(dynamics.speed_mps, 4.0), "reset restores speed")
    _expect(is_equal_approx(dynamics.heading_rad, deg_to_rad(20.0)), "reset restores heading")
    _expect(is_equal_approx(dynamics.steering_rad, deg_to_rad(-5.0)), "reset restores steering")
    _expect(is_equal_approx(dynamics.actual_acceleration_mps2, 0.0), "reset clears actual acceleration")

func _run_acceleration_for_two_seconds(timestep: float):
    var dynamics = Dynamics.new()
    var step_count: int = roundi(2.0 / timestep)
    for i in range(step_count):
        dynamics.step(2.0, timestep)
    return dynamics

func _run_turn_for_two_seconds(timestep: float):
    var dynamics = Dynamics.new()
    dynamics.reset(0.0, 2.0)
    var step_count: int = roundi(2.0 / timestep)
    for i in range(step_count):
        dynamics.step(1.0, timestep, deg_to_rad(20.0))
    return dynamics
