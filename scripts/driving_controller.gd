extends RefCounted

const ARRIVAL_TOLERANCE_M: float = 1.0
const ARRIVAL_MAX_SPEED_MPS: float = 0.1

var auto_drive_enabled: bool = true
var manual_intervention_seen: bool = false

func reset() -> void:
    auto_drive_enabled = true
    manual_intervention_seen = false

func request_clear_road_acceleration(
    position_z_m: float,
    speed_mps: float,
    target_z_m: float,
    delta: float,
    manual_input: float,
    max_acceleration: float,
    max_braking: float,
    max_speed: float
) -> Dictionary:
    if not auto_drive_enabled || manual_input != 0.0:
        manual_intervention_seen = true
        if manual_input > 0.0:
            return {"acceleration": max_acceleration, "braking_state": "MANUAL ACCEL"}
        if manual_input < 0.0:
            return {"acceleration": -max_braking, "braking_state": "MANUAL BRAKE"}
        return {"acceleration": -0.5, "braking_state": "COASTING"}

    var distance_to_target: float = target_z_m - position_z_m
    var braking_distance: float = speed_mps * speed_mps / (2.0 * max_braking) + speed_mps * delta
    if distance_to_target <= braking_distance + 1.0 || position_z_m >= target_z_m:
        return {"acceleration": -max_braking, "braking_state": "STOPPING AT DESTINATION B"}
    if speed_mps < max_speed:
        return {"acceleration": max_acceleration, "braking_state": "ACCELERATING TO B"}
    return {"acceleration": 0.0, "braking_state": "CRUISING TO B"}

func request_blocked_road_acceleration(
    position_z_m: float,
    speed_mps: float,
    lead_position_z_m: float,
    ego_length_m: float,
    lead_length_m: float,
    delta: float,
    manual_input: float,
    max_acceleration: float,
    max_braking: float,
    max_speed: float
) -> Dictionary:
    if auto_drive_enabled && manual_input == 0.0:
        var lead_rear: float = lead_position_z_m - lead_length_m / 2.0
        var ego_front: float = position_z_m + ego_length_m / 2.0
        var bumper_separation: float = lead_rear - ego_front
        var braking_distance: float = speed_mps * speed_mps / (2.0 * max_braking) + 2.5 + speed_mps * delta

        if bumper_separation <= braking_distance || position_z_m >= lead_position_z_m - 12.0:
            return {
                "acceleration": -max_braking,
                "braking_state": "AEB / PLANNED BRAKING",
                "obstacle_state": "OBSTACLE DETECTED (APPROACHING)"
            }
        if speed_mps < max_speed:
            return {
                "acceleration": max_acceleration,
                "braking_state": "ACCELERATING",
                "obstacle_state": "OBSTACLE AHEAD (Z=60)"
            }
        return {
            "acceleration": 0.0,
            "braking_state": "CRUISING",
            "obstacle_state": "OBSTACLE AHEAD (Z=60)"
        }

    if manual_input > 0.0:
        return {"acceleration": manual_input * max_acceleration, "braking_state": "MANUAL ACCEL"}
    if manual_input < 0.0:
        return {"acceleration": manual_input * max_braking, "braking_state": "MANUAL BRAKE"}
    return {"acceleration": -0.5, "braking_state": "COASTING"}

func clear_road_arrival_status(
    target_z_m: float,
    position_z_m: float,
    speed_mps: float,
    manual_input: float
) -> String:
    if position_z_m > target_z_m + ARRIVAL_TOLERANCE_M:
        return "MISSED DESTINATION"
    if not auto_drive_enabled || manual_input != 0.0:
        return "MANUAL CONTROL - AUTO DESTINATION CONTROL DISABLED"
    if manual_intervention_seen:
        return "AUTONOMOUS ARRIVAL NOT CERTIFIED - MANUAL INPUT USED"
    if absf(target_z_m - position_z_m) <= ARRIVAL_TOLERANCE_M && speed_mps <= ARRIVAL_MAX_SPEED_MPS:
        return "ARRIVED"
    return "NOT YET ARRIVED"
