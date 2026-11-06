extends RefCounted

const MIN_TARGET_DISTANCE_M: float = 0.001

## Calculates a front-wheel steering request for a world-space lookahead target.
## Vector2 coordinates are (X, Z); heading zero faces +Z and positive steering turns toward +X.
## The target should be selected ahead along the route using the caller's configured lookahead distance.
## Targets at or behind the vehicle produce an invalid neutral command rather than reverse-looking steering.
func calculate_steering_angle(
    vehicle_position_xz: Vector2,
    vehicle_heading_rad: float,
    target_position_xz: Vector2,
    wheelbase_m: float,
    steering_limit_rad: float
) -> Dictionary:
    if not vehicle_position_xz.is_finite() || not target_position_xz.is_finite():
        return _invalid_result("non_finite_position")
    if not is_finite(vehicle_heading_rad):
        return _invalid_result("non_finite_heading")
    if not is_finite(wheelbase_m) || wheelbase_m <= 0.0:
        return _invalid_result("invalid_wheelbase")
    if (
        not is_finite(steering_limit_rad)
        || steering_limit_rad < 0.0
        || steering_limit_rad >= PI / 2.0
    ):
        return _invalid_result("invalid_steering_limit")

    var displacement: Vector2 = target_position_xz - vehicle_position_xz
    var distance_squared: float = displacement.length_squared()
    if not is_finite(distance_squared) || distance_squared <= MIN_TARGET_DISTANCE_M * MIN_TARGET_DISTANCE_M:
        return _invalid_result("target_too_close")

    var sin_heading: float = sin(vehicle_heading_rad)
    var cos_heading: float = cos(vehicle_heading_rad)
    var lateral_offset_m: float = displacement.x * cos_heading - displacement.y * sin_heading
    var forward_offset_m: float = displacement.x * sin_heading + displacement.y * cos_heading
    if not is_finite(lateral_offset_m) || not is_finite(forward_offset_m):
        return _invalid_result("non_finite_local_target")
    if forward_offset_m <= 0.0:
        return _invalid_result("target_not_ahead")

    # Pure Pursuit curvature uses the actual target distance, not just its forward component.
    var curvature_per_m: float = 2.0 * lateral_offset_m / distance_squared
    var requested_steering_rad: float = atan(wheelbase_m * curvature_per_m)
    if not is_finite(requested_steering_rad):
        return _invalid_result("non_finite_steering")

    var steering_angle_rad: float = clampf(
        requested_steering_rad,
        -steering_limit_rad,
        steering_limit_rad
    )
    return {
        "valid": true,
        "status": "ok",
        "steering_angle_rad": steering_angle_rad,
        "curvature_per_m": curvature_per_m,
        "lateral_offset_m": lateral_offset_m,
        "forward_offset_m": forward_offset_m
    }

func _invalid_result(status: String) -> Dictionary:
    return {
        "valid": false,
        "status": status,
        "steering_angle_rad": 0.0,
        "curvature_per_m": 0.0,
        "lateral_offset_m": 0.0,
        "forward_offset_m": 0.0
    }
