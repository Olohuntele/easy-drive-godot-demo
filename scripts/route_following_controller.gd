extends RefCounted

const PurePursuitController = preload("res://scripts/pure_pursuit_controller.gd")

const DEFAULT_NAVIGATION_SPEED_MPS: float = 4.0
const DEFAULT_LOOKAHEAD_DISTANCE_M: float = 6.0
const DEFAULT_MAX_LOCALIZATION_ERROR_M: float = 2.5
const DEFAULT_ENDPOINT_TOLERANCE_M: float = 1.0
const DEFAULT_ENDPOINT_MAX_SPEED_MPS: float = 0.1
const DEFAULT_BRAKING_MARGIN_M: float = 0.25
## Maximum route distance at which an active obstacle creates a temporary hold.
const DETECTION_RANGE_M: float = 30.0
## Required route gap between the shuttle and the obstacle's leading edge.
const STOP_GAP_M: float = 6.0

var navigation_speed_mps: float = DEFAULT_NAVIGATION_SPEED_MPS
var lookahead_distance_m: float = DEFAULT_LOOKAHEAD_DISTANCE_M
var max_localization_error_m: float = DEFAULT_MAX_LOCALIZATION_ERROR_M
var endpoint_tolerance_m: float = DEFAULT_ENDPOINT_TOLERANCE_M
var endpoint_max_speed_mps: float = DEFAULT_ENDPOINT_MAX_SPEED_MPS
var braking_margin_m: float = DEFAULT_BRAKING_MARGIN_M
var manual_intervention_seen: bool = false

var _pure_pursuit = PurePursuitController.new()

func reset() -> void:
    manual_intervention_seen = false

func update(
    vehicle_dynamics: RefCounted,
    route_navigator: RefCounted,
    delta: float,
    manual_intervention: bool = false,
    obstacle: RefCounted = null
) -> Dictionary:
    if manual_intervention:
        manual_intervention_seen = true
    var safe_braking_mps2: float = 0.0
    if vehicle_dynamics != null && is_finite(vehicle_dynamics.max_braking):
        safe_braking_mps2 = maxf(vehicle_dynamics.max_braking, 0.0)
    if not _configuration_is_valid() || not is_finite(delta) || delta <= 0.0:
        return _safe_stop_result("invalid_controller_configuration", {}, safe_braking_mps2)
    if vehicle_dynamics == null || route_navigator == null:
        return _safe_stop_result("missing_dynamics_or_route", {}, safe_braking_mps2)
    if not route_navigator.route_valid:
        return _safe_stop_result("invalid_route", {}, safe_braking_mps2)

    var position_xz := Vector2(vehicle_dynamics.position_x_m, vehicle_dynamics.position_z_m)
    if not position_xz.is_finite() || not is_finite(vehicle_dynamics.heading_rad) \
        || not is_finite(vehicle_dynamics.speed_mps) || vehicle_dynamics.speed_mps < 0.0:
        return _safe_stop_result("invalid_vehicle_state", {}, safe_braking_mps2)

    var navigation: Dictionary = route_navigator.find_nearest_point(position_xz)
    if not navigation.get("valid", false):
        return _safe_stop_result("route_localization_failed", {}, safe_braking_mps2)
    navigation["total_length_m"] = route_navigator.total_length_m
    var localization_error_m: float = navigation.distance_m
    if not is_finite(localization_error_m) || localization_error_m > max_localization_error_m:
        return _safe_stop_result("route_localization_lost", navigation, safe_braking_mps2)

    var progress_m: float = navigation.progress_m
    var remaining_distance_m: float = maxf(route_navigator.total_length_m - progress_m, 0.0)
    var obstacle_info: Dictionary = _get_obstacle_hold(
        obstacle,
        progress_m,
        route_navigator.total_length_m
    )
    var obstacle_hold: bool = bool(obstacle_info.get("hold", false))
    var obstacle_stop_distance_m: float = float(
        obstacle_info.get("stop_distance_m", INF)
    )
    var obstacle_remaining_distance_m: float = float(
        obstacle_info.get("remaining_distance_m", INF)
    )
    var effective_stop_distance_m: float = minf(
        route_navigator.total_length_m,
        obstacle_stop_distance_m
    )
    var effective_remaining_distance_m: float = maxf(effective_stop_distance_m - progress_m, 0.0)
    var endpoint_xz: Vector2 = route_navigator.get_point_at_distance(route_navigator.total_length_m)
    var endpoint_error_m: float = position_xz.distance_to(endpoint_xz)
    if not is_finite(endpoint_error_m):
        return _safe_stop_result("invalid_endpoint_geometry", navigation, safe_braking_mps2)

    var dynamics_braking: float = vehicle_dynamics.max_braking
    var dynamics_acceleration: float = vehicle_dynamics.max_acceleration
    var dynamics_speed_limit: float = vehicle_dynamics.max_speed
    if not is_finite(dynamics_braking) || dynamics_braking <= 0.0 \
        || not is_finite(dynamics_acceleration) || dynamics_acceleration < 0.0 \
        || not is_finite(dynamics_speed_limit) || dynamics_speed_limit <= 0.0:
        return _safe_stop_result("invalid_vehicle_dynamics_limits", navigation, safe_braking_mps2)

    if remaining_distance_m <= 0.0001 && endpoint_error_m > endpoint_tolerance_m:
        return _route_result(
            -dynamics_braking, 0.0, "MISSED DESTINATION", "endpoint_overshot",
            navigation, null, remaining_distance_m, endpoint_error_m, 0.0, false,
            "ok", 0.0, obstacle_info
        )

    if endpoint_error_m <= endpoint_tolerance_m \
        && vehicle_dynamics.speed_mps <= endpoint_max_speed_mps \
        && remaining_distance_m <= endpoint_tolerance_m:
        var completion_status: String = "ARRIVED" if not manual_intervention_seen else "EN ROUTE"
        var completion_reason: String = "route_complete" if not manual_intervention_seen else "manual_intervention_seen"
        var completion_acceleration_mps2: float = -dynamics_braking \
            if vehicle_dynamics.speed_mps > 0.0 else 0.0
        return _route_result(
            completion_acceleration_mps2, 0.0, completion_status, completion_reason,
            navigation, null, remaining_distance_m, endpoint_error_m, 0.0,
            not manual_intervention_seen, "ok", 0.0, obstacle_info
        )

    var lookahead: Dictionary = route_navigator.get_lookahead_point(lookahead_distance_m)
    if not lookahead.get("valid", false):
        return _safe_stop_result("invalid_lookahead", navigation, safe_braking_mps2)
    var steering_result: Dictionary = _pure_pursuit.calculate_steering_angle(
        position_xz,
        vehicle_dynamics.heading_rad,
        lookahead.position_xz,
        vehicle_dynamics.wheelbase_m,
        vehicle_dynamics.steering_limit_rad
    )
    var requested_steering_rad: float = 0.0
    var steering_status: String = steering_result.status
    if steering_result.valid:
        requested_steering_rad = steering_result.steering_angle_rad
    else:
        var endpoint_is_near_and_ahead: bool = endpoint_error_m <= lookahead_distance_m \
            && _is_endpoint_ahead(position_xz, vehicle_dynamics.heading_rad, endpoint_xz)
        if not endpoint_is_near_and_ahead:
            return _safe_stop_result(
                "invalid_pure_pursuit:" + steering_status,
                navigation,
                safe_braking_mps2
            )
        steering_status = "endpoint_steering_released"

    var braking_distance_m: float = vehicle_dynamics.speed_mps * vehicle_dynamics.speed_mps \
        / (2.0 * dynamics_braking)
    var should_brake: bool = effective_remaining_distance_m <= braking_distance_m \
        + braking_margin_m
    var requested_acceleration_mps2: float
    var desired_speed_mps: float
    if should_brake:
        requested_acceleration_mps2 = -dynamics_braking
        desired_speed_mps = 0.0
    else:
        desired_speed_mps = 0.0 if obstacle_hold else minf(
            navigation_speed_mps,
            dynamics_speed_limit
        )
        if vehicle_dynamics.speed_mps < desired_speed_mps:
            requested_acceleration_mps2 = minf(
                dynamics_acceleration,
                (desired_speed_mps - vehicle_dynamics.speed_mps) / delta
            )
        elif vehicle_dynamics.speed_mps > desired_speed_mps:
            requested_acceleration_mps2 = -minf(
                dynamics_braking,
                (vehicle_dynamics.speed_mps - desired_speed_mps) / delta
            )
        else:
            requested_acceleration_mps2 = 0.0

    return _route_result(
        requested_acceleration_mps2,
        requested_steering_rad,
        "EN ROUTE",
        "braking_for_endpoint" if should_brake else "tracking_route",
        navigation,
        lookahead,
        remaining_distance_m,
        endpoint_error_m,
        desired_speed_mps,
        false,
        steering_status,
        braking_distance_m,
        obstacle_info
    )

func _get_obstacle_hold(
    obstacle: RefCounted,
    progress_m: float,
    route_length_m: float
) -> Dictionary:
    var result := {
        "hold": false,
        "obstacle_distance_m": INF,
        "obstacle_remaining_distance_m": INF,
        "stop_distance_m": INF,
        "obstacle_length_m": 0.0
    }
    if obstacle == null || not obstacle.active:
        return result
    if not is_finite(obstacle.distance_along_route_m) \
        || not is_finite(obstacle.length_m) \
        || obstacle.length_m < 0.0:
        return result
    var obstacle_remaining_m: float = obstacle.distance_along_route_m - progress_m
    if obstacle_remaining_m < 0.0 || obstacle_remaining_m > DETECTION_RANGE_M:
        return result
    var stop_distance_m: float = obstacle.distance_along_route_m - STOP_GAP_M
    if stop_distance_m < 0.0 || stop_distance_m > route_length_m:
        return result
    result.hold = true
    result.obstacle_distance_m = obstacle.distance_along_route_m
    result.obstacle_remaining_distance_m = obstacle_remaining_m
    result.stop_distance_m = stop_distance_m
    result.obstacle_length_m = obstacle.length_m
    return result

func _configuration_is_valid() -> bool:
    return is_finite(navigation_speed_mps) && navigation_speed_mps > 0.0 \
        && is_finite(lookahead_distance_m) && lookahead_distance_m > 0.0 \
        && is_finite(max_localization_error_m) && max_localization_error_m > 0.0 \
        && is_finite(endpoint_tolerance_m) && endpoint_tolerance_m > 0.0 \
        && is_finite(endpoint_max_speed_mps) && endpoint_max_speed_mps >= 0.0 \
        && is_finite(braking_margin_m) && braking_margin_m >= 0.0

func _is_endpoint_ahead(position_xz: Vector2, heading_rad: float, endpoint_xz: Vector2) -> bool:
    var displacement: Vector2 = endpoint_xz - position_xz
    var forward_offset_m: float = displacement.x * sin(heading_rad) + displacement.y * cos(heading_rad)
    return forward_offset_m > 0.0

func _safe_stop_result(
    reason: String,
    navigation: Dictionary = {},
    max_braking_mps2: float = 0.0
) -> Dictionary:
    var remaining_distance_m: float = INF
    var endpoint_error_m: float = INF
    var progress_m: float = 0.0
    var cross_track_error_m: float = 0.0
    var localization_error_m: float = INF
    if not navigation.is_empty():
        remaining_distance_m = maxf(
            navigation.get("total_length_m", INF) - navigation.get("progress_m", 0.0),
            0.0
        )
        progress_m = navigation.get("progress_m", 0.0)
        cross_track_error_m = navigation.get("cross_track_error_m", 0.0)
        localization_error_m = navigation.get("distance_m", INF)
    return {
        "requested_acceleration_mps2": -maxf(max_braking_mps2, 0.0),
        "requested_steering_rad": 0.0,
        "status": "SAFE STOP / INVALID NAVIGATION",
        "reason": reason,
        "route_valid": not navigation.is_empty(),
        "localized": false,
        "route_progress_m": progress_m,
        "cross_track_error_m": cross_track_error_m,
        "localization_error_m": localization_error_m,
        "localization_quality": "LOST",
        "remaining_distance_m": remaining_distance_m,
        "endpoint_error_m": endpoint_error_m,
        "lookahead_position_xz": Vector2.ZERO,
        "desired_speed_mps": 0.0,
        "braking_distance_m": 0.0,
        "braking": true,
        "arrival_certified": false,
        "steering_status": "neutral_safe_stop"
    }

func _route_result(
    acceleration_mps2: float,
    steering_rad: float,
    status_text: String,
    reason: String,
    navigation: Dictionary,
    lookahead: Variant,
    remaining_distance_m: float,
    endpoint_error_m: float,
    desired_speed_mps: float,
    arrival_certified: bool,
    steering_status: String = "ok",
    braking_distance_m: float = 0.0,
    obstacle_info: Dictionary = {}
) -> Dictionary:
    var lookahead_position: Vector2 = Vector2.ZERO
    if lookahead is Dictionary:
        lookahead_position = lookahead.get("position_xz", Vector2.ZERO)
    var result := {
        "requested_acceleration_mps2": acceleration_mps2,
        "requested_steering_rad": steering_rad,
        "status": status_text,
        "reason": reason,
        "route_valid": true,
        "localized": true,
        "route_progress_m": navigation.progress_m,
        "cross_track_error_m": navigation.cross_track_error_m,
        "localization_error_m": navigation.distance_m,
        "localization_quality": "GOOD",
        "remaining_distance_m": remaining_distance_m,
        "endpoint_error_m": endpoint_error_m,
        "lookahead_position_xz": lookahead_position,
        "desired_speed_mps": desired_speed_mps,
        "braking_distance_m": braking_distance_m,
        "braking": acceleration_mps2 < 0.0,
        "arrival_certified": arrival_certified,
        "steering_status": steering_status
    }
    result["obstacle_hold"] = bool(obstacle_info.get("hold", false))
    result["obstacle_distance_m"] = obstacle_info.get("obstacle_distance_m", INF)
    result["obstacle_remaining_distance_m"] = obstacle_info.get(
        "obstacle_remaining_distance_m",
        INF
    )
    result["obstacle_stop_distance_m"] = obstacle_info.get("stop_distance_m", INF)
    result["obstacle_length_m"] = obstacle_info.get("obstacle_length_m", 0.0)
    if result.obstacle_hold && result.status == "EN ROUTE":
        result.status = "OBSTACLE AHEAD - HOLDING"
        result.reason = "braking_for_obstacle" if result.braking else "holding_for_obstacle"
    return result
