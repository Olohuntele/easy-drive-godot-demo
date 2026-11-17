extends RefCounted

const ROAD_WIDTH_M: float = 4.0
const VEHICLE_WIDTH_M: float = 1.6
const LANE_OFFSET_M: float = 1.0
const HOLD_BEFORE_BYPASS_S: float = 2.0
const MIN_REJOIN_CLEARANCE_M: float = 4.0
const TRANSITION_LENGTH_M: float = 8.0
const JUNCTION_ZONE_M: float = 6.0
const STATION_STOP_ZONE_M: float = 6.0
const END_SAFETY_MARGIN_M: float = 5.0
const PATH_SAMPLE_SPACING_M: float = 2.0
const ARRIVAL_SPEED_TOLERANCE_MPS: float = 0.1

func build(
    route_navigator: RefCounted,
    network: RefCounted,
    vehicle_position_xz: Vector2,
    vehicle_heading_rad: float,
    vehicle_speed_mps: float,
    obstacle: RefCounted
) -> Dictionary:
    if route_navigator == null || not route_navigator.route_valid:
        return _failure("invalid_active_route")
    if network == null:
        return _failure("missing_network")
    if obstacle == null || not obstacle.active:
        return _failure("no_active_obstacle")
    if not vehicle_position_xz.is_finite() || not is_finite(vehicle_heading_rad) \
        || not is_finite(vehicle_speed_mps):
        return _failure("invalid_vehicle_state")
    if vehicle_speed_mps > ARRIVAL_SPEED_TOLERANCE_MPS:
        return _failure("vehicle_not_stationary")

    var navigation: Dictionary = route_navigator.find_nearest_point(
        vehicle_position_xz,
        true
    )
    if not navigation.get("valid", false):
        return _failure("route_localization_failed")
    var progress_m: float = float(navigation.progress_m)
    var obstacle_distance_m: float = float(obstacle.distance_along_route_m)
    var rejoin_distance_m: float = obstacle_distance_m + obstacle.length_m \
        + MIN_REJOIN_CLEARANCE_M
    if obstacle_distance_m <= progress_m || rejoin_distance_m <= obstacle_distance_m:
        return _failure("obstacle_not_ahead")
    if rejoin_distance_m >= route_navigator.total_length_m - END_SAFETY_MARGIN_M:
        return _failure("rejoin_too_close_to_leg_end")

    var rejoin_point: Vector2 = route_navigator.get_point_at_distance(rejoin_distance_m)
    if _near_restricted_zone(rejoin_point, network):
        return _failure("rejoin_inside_restricted_zone")

    var half_clearance_m: float = ROAD_WIDTH_M * 0.5 - VEHICLE_WIDTH_M * 0.5
    if LANE_OFFSET_M > half_clearance_m:
        return _failure("offset_exceeds_road_clearance")

    var points: Array[Vector2] = [vehicle_position_xz]
    var heading_forward := Vector2(sin(vehicle_heading_rad), cos(vehicle_heading_rad))
    points.append(vehicle_position_xz + heading_forward * 1.0)
    var sample_distance_m: float = progress_m + PATH_SAMPLE_SPACING_M
    while sample_distance_m < route_navigator.total_length_m:
        var center: Vector2 = route_navigator.get_point_at_distance(sample_distance_m)
        var tangent := _route_tangent(route_navigator, sample_distance_m)
        if tangent.length_squared() <= 0.000001:
            return _failure("route_tangent_invalid")
        var right := Vector2(tangent.y, -tangent.x)
        var offset_m: float = _offset_at_distance(
            sample_distance_m,
            progress_m,
            obstacle_distance_m,
            rejoin_distance_m
        )
        if absf(offset_m) + VEHICLE_WIDTH_M * 0.5 > ROAD_WIDTH_M * 0.5:
            return _failure("offset_exceeds_road_clearance")
        points.append(center + right * offset_m)
        if sample_distance_m >= rejoin_distance_m:
            break
        sample_distance_m += PATH_SAMPLE_SPACING_M

    if points[points.size() - 1].distance_to(rejoin_point) > 0.01:
        points.append(rejoin_point)
    sample_distance_m = rejoin_distance_m + PATH_SAMPLE_SPACING_M
    while sample_distance_m < route_navigator.total_length_m:
        points.append(route_navigator.get_point_at_distance(sample_distance_m))
        sample_distance_m += PATH_SAMPLE_SPACING_M
    var endpoint: Vector2 = route_navigator.get_point_at_distance(
        route_navigator.total_length_m
    )
    if points[points.size() - 1].distance_to(endpoint) > 0.01:
        points.append(endpoint)
    return {
        "valid": true,
        "failure_reason": "",
        "centerline_points": points,
        "rejoin_distance_m": rejoin_distance_m,
        "lane_offset_m": LANE_OFFSET_M
    }

func _offset_at_distance(
    distance_m: float,
    start_distance_m: float,
    obstacle_distance_m: float,
    rejoin_distance_m: float
) -> float:
    var ramp_in_end := start_distance_m + TRANSITION_LENGTH_M
    var ramp_out_start := rejoin_distance_m - TRANSITION_LENGTH_M
    if distance_m <= ramp_in_end:
        return LANE_OFFSET_M * _smoothstep(
            clampf((distance_m - start_distance_m) / TRANSITION_LENGTH_M, 0.0, 1.0)
        )
    if distance_m >= ramp_out_start:
        return LANE_OFFSET_M * (1.0 - _smoothstep(
            clampf((distance_m - ramp_out_start) / TRANSITION_LENGTH_M, 0.0, 1.0)
        ))
    return LANE_OFFSET_M

func _route_tangent(route_navigator: RefCounted, distance_m: float) -> Vector2:
    var before: Vector2 = route_navigator.get_point_at_distance(
        maxf(distance_m - 0.25, 0.0)
    )
    var after: Vector2 = route_navigator.get_point_at_distance(
        minf(distance_m + 0.25, route_navigator.total_length_m)
    )
    return (after - before).normalized()

func _near_restricted_zone(point: Vector2, network: RefCounted) -> bool:
    for node in network.get_nodes():
        if point.distance_to(node.position_xz) < JUNCTION_ZONE_M:
            return true
    for station in network.get_stations():
        if point.distance_to(station.stop_position_xz) < STATION_STOP_ZONE_M:
            return true
    return false

func _smoothstep(value: float) -> float:
    return value * value * (3.0 - 2.0 * value)

func _failure(reason: String) -> Dictionary:
    return {"valid": false, "failure_reason": reason}
