extends SceneTree

const Dynamics = preload("res://scripts/vehicle_dynamics.gd")

var failures: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    _test_positive_acceleration()
    _test_speed_limit()
    _test_braking_and_no_reverse()
    _test_zero_acceleration()
    _test_reset()
    _test_fixed_timestep_stability()

    if failures == 0:
        print("VehicleDynamics tests passed.")
        quit(0)
    else:
        push_error("%d VehicleDynamics test(s) failed." % failures)
        quit(1)

func _expect(condition: bool, message: String) -> void:
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

func _run_acceleration_for_two_seconds(timestep: float):
    var dynamics = Dynamics.new()
    var step_count: int = roundi(2.0 / timestep)
    for i in range(step_count):
        dynamics.step(2.0, timestep)
    return dynamics
