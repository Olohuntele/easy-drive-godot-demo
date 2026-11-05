extends RefCounted

var max_acceleration: float = 3.5
var max_braking: float = 6.0
var max_speed: float = 10.0

var position_m: float:
    get:
        return _position_m

var speed_mps: float:
    get:
        return _speed_mps

var actual_acceleration_mps2: float:
    get:
        return _actual_acceleration_mps2

var _position_m: float = 0.0
var _speed_mps: float = 0.0
var _actual_acceleration_mps2: float = 0.0

func reset(initial_position: float = 0.0, initial_speed: float = 0.0) -> void:
    _position_m = initial_position
    _speed_mps = clampf(initial_speed, 0.0, max_speed)
    _actual_acceleration_mps2 = 0.0

func step(requested_acceleration_mps2: float, delta: float) -> void:
    if delta <= 0.0:
        push_error("VehicleDynamics.step requires a positive timestep.")
        return
    if max_acceleration < 0.0 || max_braking < 0.0 || max_speed <= 0.0:
        push_error("VehicleDynamics limits must be non-negative and max_speed must be positive.")
        return

    var initial_speed: float = _speed_mps
    var acceleration: float = clampf(requested_acceleration_mps2, -max_braking, max_acceleration)
    if (initial_speed >= max_speed && acceleration > 0.0) || (initial_speed <= 0.0 && acceleration < 0.0):
        acceleration = 0.0

    var unconstrained_speed: float = initial_speed + acceleration * delta
    var final_speed: float = clampf(unconstrained_speed, 0.0, max_speed)
    var distance_m: float

    if acceleration > 0.0 && unconstrained_speed >= max_speed:
        var time_to_limit: float = (max_speed - initial_speed) / acceleration
        distance_m = (
            initial_speed * time_to_limit
            + 0.5 * acceleration * time_to_limit * time_to_limit
            + max_speed * (delta - time_to_limit)
        )
    elif acceleration < 0.0 && unconstrained_speed <= 0.0:
        var time_to_stop: float = -initial_speed / acceleration
        distance_m = (
            initial_speed * time_to_stop
            + 0.5 * acceleration * time_to_stop * time_to_stop
        )
    else:
        distance_m = initial_speed * delta + 0.5 * acceleration * delta * delta

    _position_m += distance_m
    _speed_mps = final_speed
    _actual_acceleration_mps2 = (_speed_mps - initial_speed) / delta
