extends RefCounted

var max_acceleration: float = 3.5
var max_braking: float = 6.0
var max_speed: float = 10.0
var wheelbase_m: float = 2.5
var steering_limit_rad: float = deg_to_rad(30.0)

var position_x_m: float:
    get:
        return _position_x_m

var position_z_m: float:
    get:
        return _position_z_m

var position_m: float:
    get:
        return _position_z_m

var heading_rad: float:
    get:
        return _heading_rad

var speed_mps: float:
    get:
        return _speed_mps

var actual_acceleration_mps2: float:
    get:
        return _actual_acceleration_mps2

var steering_rad: float:
    get:
        return _steering_rad

var _position_x_m: float = 0.0
var _position_z_m: float = 0.0
var _heading_rad: float = 0.0
var _speed_mps: float = 0.0
var _actual_acceleration_mps2: float = 0.0
var _steering_rad: float = 0.0

func reset(
    initial_position_z_m: float = 0.0,
    initial_speed_mps: float = 0.0,
    initial_position_x_m: float = 0.0,
    initial_heading_rad: float = 0.0,
    initial_steering_rad: float = 0.0
) -> void:
    _position_x_m = initial_position_x_m
    _position_z_m = initial_position_z_m
    _heading_rad = initial_heading_rad
    _speed_mps = clampf(initial_speed_mps, 0.0, max_speed)
    _actual_acceleration_mps2 = 0.0
    _steering_rad = clampf(initial_steering_rad, -steering_limit_rad, steering_limit_rad)

func step(
    requested_acceleration_mps2: float,
    delta: float,
    requested_steering_rad: float = 0.0
) -> void:
    if delta <= 0.0:
        push_error("VehicleDynamics.step requires a positive timestep.")
        return
    if max_acceleration < 0.0 || max_braking < 0.0 || max_speed <= 0.0:
        push_error("VehicleDynamics acceleration and braking limits must be non-negative and max_speed must be positive.")
        return
    if wheelbase_m <= 0.0 || steering_limit_rad < 0.0:
        push_error("VehicleDynamics wheelbase must be positive and steering_limit_rad must be non-negative.")
        return

    _steering_rad = clampf(requested_steering_rad, -steering_limit_rad, steering_limit_rad)
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

    # Kinematic bicycle model: curvature = tan(steering) / wheelbase.
    # Godot yaw is positive toward +X for a vehicle whose local forward is +Z.
    # Integrating the circular arc exactly keeps motion stable across fixed steps.
    var curvature: float = tan(_steering_rad) / wheelbase_m
    var heading_change: float = distance_m * curvature
    if absf(curvature) < 0.000001:
        _position_x_m += distance_m * sin(_heading_rad)
        _position_z_m += distance_m * cos(_heading_rad)
    else:
        var final_heading: float = _heading_rad + heading_change
        _position_x_m += (cos(_heading_rad) - cos(final_heading)) / curvature
        _position_z_m += (sin(final_heading) - sin(_heading_rad)) / curvature
    _heading_rad += heading_change
    _speed_mps = final_speed
    _actual_acceleration_mps2 = (_speed_mps - initial_speed) / delta
