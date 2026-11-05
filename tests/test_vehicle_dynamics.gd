extends SceneTree

const Dynamics = preload("res://scripts/vehicle_dynamics.gd")
const DrivingController = preload("res://scripts/driving_controller.gd")

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
    _test_autonomous_clear_road_arrival()
    _test_large_overshoot_is_missed()
    _test_manual_coast_down_is_not_autonomous_arrival()
    _test_temporary_manual_override_disqualifies_autonomous_arrival()
    _test_reset_restores_autonomous_mode()
    _test_blocked_road_still_stops_safely()
    _test_steering_test_control_still_turns()

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

func _test_autonomous_clear_road_arrival() -> void:
    var dynamics = Dynamics.new()
    var controller = DrivingController.new()
    controller.reset()
    var status: String = "NOT YET ARRIVED"
    for i in range(3000):
        var decision: Dictionary = controller.request_clear_road_acceleration(
            dynamics.position_z_m, dynamics.speed_mps, 100.0, 1.0 / 60.0, 0.0,
            dynamics.max_acceleration, dynamics.max_braking, dynamics.max_speed
        )
        dynamics.step(decision.acceleration, 1.0 / 60.0)
        status = controller.clear_road_arrival_status(
            100.0, dynamics.position_z_m, dynamics.speed_mps, 0.0
        )
        if dynamics.speed_mps == 0.0:
            break

    _expect(dynamics.speed_mps <= 0.1, "autonomous clear-road controller stops within the speed tolerance")
    _expect(absf(100.0 - dynamics.position_z_m) <= 1.0, "autonomous clear-road stop is within 1 m of target")
    _expect(status == "ARRIVED", "controller reports ARRIVED only for the in-tolerance stop")

func _test_large_overshoot_is_missed() -> void:
    var controller = DrivingController.new()
    controller.reset()
    var status: String = controller.clear_road_arrival_status(100.0, 191.84, 0.0, 0.0)
    _expect(status == "MISSED DESTINATION", "stopped vehicle at Z=191.84 is a missed destination")

func _test_manual_coast_down_is_not_autonomous_arrival() -> void:
    var dynamics = Dynamics.new()
    var controller = DrivingController.new()
    controller.auto_drive_enabled = false
    var saw_arrived: bool = false
    for i in range(3000):
        var manual_input: float = 1.0 if dynamics.position_z_m < 90.0 else 0.0
        var decision: Dictionary = controller.request_clear_road_acceleration(
            dynamics.position_z_m, dynamics.speed_mps, 100.0, 1.0 / 60.0, manual_input,
            dynamics.max_acceleration, dynamics.max_braking, dynamics.max_speed
        )
        dynamics.step(decision.acceleration, 1.0 / 60.0)
        var status: String = controller.clear_road_arrival_status(
            100.0, dynamics.position_z_m, dynamics.speed_mps, manual_input
        )
        saw_arrived = saw_arrived || status == "ARRIVED"
        if dynamics.speed_mps == 0.0 && dynamics.position_z_m > 100.0:
            break

    _expect(not saw_arrived, "manual acceleration and coast-down never report autonomous arrival")
    _expect(dynamics.position_z_m > 101.0, "manual coast-down demonstrates destination overshoot")
    _expect(
        controller.clear_road_arrival_status(100.0, dynamics.position_z_m, dynamics.speed_mps, 0.0)
            == "MISSED DESTINATION",
        "manual coast-down overshoot is reported as missed"
    )

func _test_temporary_manual_override_disqualifies_autonomous_arrival() -> void:
    var dynamics = Dynamics.new()
    var controller = DrivingController.new()
    controller.reset()

    for i in range(10):
        var manual_decision: Dictionary = controller.request_clear_road_acceleration(
            dynamics.position_z_m, dynamics.speed_mps, 100.0, 1.0 / 60.0, 1.0,
            dynamics.max_acceleration, dynamics.max_braking, dynamics.max_speed
        )
        dynamics.step(manual_decision.acceleration, 1.0 / 60.0)

    var status: String = "NOT YET ARRIVED"
    for i in range(3000):
        var autonomous_decision: Dictionary = controller.request_clear_road_acceleration(
            dynamics.position_z_m, dynamics.speed_mps, 100.0, 1.0 / 60.0, 0.0,
            dynamics.max_acceleration, dynamics.max_braking, dynamics.max_speed
        )
        dynamics.step(autonomous_decision.acceleration, 1.0 / 60.0)
        status = controller.clear_road_arrival_status(
            100.0, dynamics.position_z_m, dynamics.speed_mps, 0.0
        )
        if dynamics.speed_mps == 0.0:
            break

    _expect(status == "AUTONOMOUS ARRIVAL NOT CERTIFIED - MANUAL INPUT USED", "temporary W/S override cannot be certified as autonomous arrival")

func _test_reset_restores_autonomous_mode() -> void:
    var controller = DrivingController.new()
    controller.auto_drive_enabled = false
    controller.request_clear_road_acceleration(0.0, 0.0, 100.0, 1.0 / 60.0, 0.0, 3.5, 6.0, 10.0)
    controller.reset()
    _expect(controller.auto_drive_enabled, "reset restores autonomous control mode")
    _expect(not controller.manual_intervention_seen, "reset clears the manual-intervention latch")

    var dynamics = Dynamics.new()
    var status: String = "NOT YET ARRIVED"
    for i in range(3000):
        var decision: Dictionary = controller.request_clear_road_acceleration(
            dynamics.position_z_m, dynamics.speed_mps, 100.0, 1.0 / 60.0, 0.0,
            dynamics.max_acceleration, dynamics.max_braking, dynamics.max_speed
        )
        dynamics.step(decision.acceleration, 1.0 / 60.0)
        status = controller.clear_road_arrival_status(100.0, dynamics.position_z_m, dynamics.speed_mps, 0.0)
        if dynamics.speed_mps == 0.0:
            break
    _expect(status == "ARRIVED", "autonomous destination arrival works after reset")

func _test_blocked_road_still_stops_safely() -> void:
    var dynamics = Dynamics.new()
    var controller = DrivingController.new()
    controller.reset()
    for i in range(3000):
        var decision: Dictionary = controller.request_blocked_road_acceleration(
            dynamics.position_z_m, dynamics.speed_mps, 60.0, 4.0, 4.0, 1.0 / 60.0, 0.0,
            dynamics.max_acceleration, dynamics.max_braking, dynamics.max_speed
        )
        dynamics.step(decision.acceleration, 1.0 / 60.0)
        if dynamics.speed_mps == 0.0 && dynamics.position_z_m > 0.0:
            break
    var bumper_clearance: float = 60.0 - 4.0 / 2.0 - (dynamics.position_z_m + 4.0 / 2.0)
    _expect(dynamics.speed_mps == 0.0, "blocked-road controller brings the vehicle to a stop")
    _expect(bumper_clearance >= 2.0, "blocked-road stop retains at least 2 m bumper clearance")

func _test_steering_test_control_still_turns() -> void:
    var dynamics = Dynamics.new()
    var controller = DrivingController.new()
    controller.reset()
    var decision: Dictionary = controller.request_clear_road_acceleration(
        dynamics.position_z_m, dynamics.speed_mps, 100.0, 1.0 / 60.0, 0.0,
        dynamics.max_acceleration, dynamics.max_braking, dynamics.max_speed
    )
    dynamics.reset(0.0, 2.0)
    dynamics.step(decision.acceleration, 0.5, dynamics.steering_limit_rad)
    _expect(dynamics.position_x_m > 0.0, "steering-test positive steering still turns toward +X")
    _expect(dynamics.heading_rad > 0.0, "steering-test positive steering still changes heading")

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
