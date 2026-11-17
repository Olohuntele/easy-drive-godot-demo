extends RefCounted

const RoadDimensions = preload("res://scripts/road_dimensions.gd")

const VEHICLE_WIDTH_M: float = 1.8
const VEHICLE_LENGTH_M: float = 3.7
const LATERAL_MARGIN_M: float = 0.3
const ROAD_EDGE_MARGIN_M: float = 0.0
const BYPASS_CLEARANCE_RESERVE_M: float = 0.5
const HOLD_BEFORE_BYPASS_S: float = 2.0
const MIN_REJOIN_CLEARANCE_M: float = 6.0
const JUNCTION_ZONE_M: float = 6.0
const STATION_STOP_ZONE_M: float = 6.0
const END_SAFETY_MARGIN_M: float = 5.0
const PATH_SAMPLE_SPACING_M: float = 0.1
const ARRIVAL_SPEED_TOLERANCE_MPS: float = 0.1
const DEFAULT_WHEELBASE_M: float = 2.5
const DEFAULT_STEERING_LIMIT_RAD: float = deg_to_rad(30.0)

func build(
    route_navigator: RefCounted,
    network: RefCounted,
    vehicle_position_xz: Vector2,
    vehicle_heading_rad: float,
    vehicle_speed_mps: float,
    obstacle: RefCounted,
    road_width_m: float = RoadDimensions.ROAD_WIDTH_M,
    wheelbase_m: float = DEFAULT_WHEELBASE_M,
    steering_limit_rad: float = DEFAULT_STEERING_LIMIT_RAD
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
    if not is_finite(road_width_m) || road_width_m <= VEHICLE_WIDTH_M:
        return _failure("road_width_invalid")
    if not is_finite(wheelbase_m) || wheelbase_m <= 0.0 \
        || not is_finite(steering_limit_rad) || steering_limit_rad <= 0.0:
        return _failure("vehicle_steering_limits_invalid")
    if not is_finite(obstacle.width_m) || obstacle.width_m <= 0.0 \
        || not is_finite(obstacle.length_m) || obstacle.length_m <= 0.0:
        return _failure("obstacle_dimensions_invalid")

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

    var required_offset_m: float = obstacle.width_m * 0.5 \
        + VEHICLE_WIDTH_M * 0.5 + LATERAL_MARGIN_M
    var bypass_offset_m: float = required_offset_m + BYPASS_CLEARANCE_RESERVE_M
    var maximum_center_offset_m: float = road_width_m * 0.5 \
        - ROAD_EDGE_MARGIN_M - VEHICLE_WIDTH_M * 0.5
    if bypass_offset_m > maximum_center_offset_m:
        return _failure("offset_exceeds_road_clearance")

    var rejoin_point: Vector2 = route_navigator.get_point_at_distance(rejoin_distance_m)
    if _near_restricted_zone(rejoin_point, network):
        return _failure("rejoin_inside_restricted_zone")

    var curvature_limit_per_m: float = tan(steering_limit_rad) / wheelbase_m
    var transition_length_m: float = sqrt(
        6.0 * bypass_offset_m / curvature_limit_per_m
    )
    var distance_to_obstacle_near_face_m: float = obstacle_distance_m \
        - obstacle.length_m * 0.5 - progress_m
    if transition_length_m > distance_to_obstacle_near_face_m:
        return _failure("stop_gap_too_short_for_transition")

    var initial_tangent := Vector2(
        sin(vehicle_heading_rad),
        cos(vehicle_heading_rad)
    ).normalized()
    var route_start_distance_m: float = progress_m + transition_length_m
    var initial_route_tangent: Vector2 = _route_tangent(
        route_navigator,
        progress_m
    )
    var heading_delta_rad := absf(
        wrapf(initial_tangent.angle_to(initial_route_tangent), -PI, PI)
    )
    var heading_blend_length_m: float = maxf(
        transition_length_m * 1.7,
        heading_delta_rad / curvature_limit_per_m * 1.7
    )
    route_start_distance_m = progress_m + heading_blend_length_m
    if route_start_distance_m >= obstacle_distance_m \
        - obstacle.length_m * 0.5:
        return _failure("stop_gap_too_short_for_transition")

    var points: Array[Vector2] = [vehicle_position_xz]
    var blend_target := _offset_point(
        route_navigator,
        route_start_distance_m,
        progress_m,
        rejoin_distance_m,
        bypass_offset_m,
        transition_length_m
    )
    if absf(_offset_at_distance(
        route_start_distance_m,
        progress_m,
        rejoin_distance_m,
        bypass_offset_m,
        transition_length_m
    )) + VEHICLE_WIDTH_M * 0.5 \
        > road_width_m * 0.5 - ROAD_EDGE_MARGIN_M:
        return _failure("offset_exceeds_road_clearance")
    var blend_tangent := _offset_point_tangent(
        route_navigator,
        route_start_distance_m,
        progress_m,
        rejoin_distance_m,
        bypass_offset_m,
        transition_length_m
    )
    var blend_length_m: float = vehicle_position_xz.distance_to(blend_target)
    var control_length_m: float = maxf(blend_length_m, heading_blend_length_m) / 3.0
    var blend_control_start := vehicle_position_xz + initial_tangent * control_length_m
    var blend_control_end := blend_target - blend_tangent * control_length_m
    var blend_sample_count: int = maxi(
        2,
        ceili(maxf(blend_length_m, heading_blend_length_m) / 0.05)
    )
    for blend_index in range(1, blend_sample_count + 1):
        var blend_t: float = float(blend_index) / float(blend_sample_count)
        var inverse_t: float = 1.0 - blend_t
        points.append(
            inverse_t * inverse_t * inverse_t * vehicle_position_xz
                + 3.0 * inverse_t * inverse_t * blend_t * blend_control_start
                + 3.0 * inverse_t * blend_t * blend_t * blend_control_end
                + blend_t * blend_t * blend_t * blend_target
        )

    var sample_distance_m: float = route_start_distance_m + PATH_SAMPLE_SPACING_M
    while sample_distance_m <= rejoin_distance_m:
        var sample_offset_m: float = _offset_at_distance(
            sample_distance_m,
            progress_m,
            rejoin_distance_m,
            bypass_offset_m,
            transition_length_m
        )
        if absf(sample_offset_m) + VEHICLE_WIDTH_M * 0.5 \
            > road_width_m * 0.5 - ROAD_EDGE_MARGIN_M:
            return _failure("offset_exceeds_road_clearance")
        points.append(_offset_point(
            route_navigator,
            sample_distance_m,
            progress_m,
            rejoin_distance_m,
            bypass_offset_m,
            transition_length_m
        ))
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

    points = _resample_uniform(points, PATH_SAMPLE_SPACING_M)
    var curvature_result := _measure_path_curvature(points)
    if curvature_result.max_curvature_per_m > curvature_limit_per_m + 0.0001:
        return {
            "valid": false,
            "failure_reason": "path_curvature_exceeds_steering_limit",
            "max_curvature_per_m": curvature_result.max_curvature_per_m,
            "max_curvature_index": curvature_result.max_curvature_index,
            "curvature_limit_per_m": curvature_limit_per_m,
            "transition_length_m": transition_length_m,
            "required_offset_m": required_offset_m,
            "bypass_offset_m": bypass_offset_m
        }
    var obstacle_tangent: Vector2 = Vector2(
        sin(obstacle.world_heading_rad),
        cos(obstacle.world_heading_rad)
    ).normalized()
    var obstacle_right := Vector2(obstacle_tangent.y, -obstacle_tangent.x)
    var obstacle_center: Vector2 = obstacle.world_position_xz
    var obstacle_rectangle := _rectangle(
        obstacle_center,
        obstacle_tangent,
        obstacle_right,
        obstacle.length_m,
        obstacle.width_m
    )
    var swept := _swept_clearance(points, obstacle_rectangle)
    if swept.minimum_clearance_m < LATERAL_MARGIN_M:
        return {
            "valid": false,
            "failure_reason": "swept_footprint_clearance_insufficient",
            "minimum_clearance_m": swept.minimum_clearance_m,
            "path_distance_m": swept.path_distance_m,
            "required_offset_m": required_offset_m,
            "transition_length_m": transition_length_m,
            "curvature_limit_per_m": curvature_limit_per_m,
            "max_curvature_per_m": curvature_result.max_curvature_per_m
        }
    return {
        "valid": true,
        "failure_reason": "",
        "centerline_points": points,
        "rejoin_distance_m": rejoin_distance_m,
        "lane_offset_m": bypass_offset_m,
        "required_offset_m": required_offset_m,
        "bypass_offset_m": bypass_offset_m,
        "transition_length_m": transition_length_m,
        "curvature_limit_per_m": curvature_limit_per_m,
        "max_curvature_per_m": curvature_result.max_curvature_per_m,
        "minimum_clearance_m": swept.minimum_clearance_m
    }

func _offset_at_distance(
    distance_m: float,
    start_distance_m: float,
    rejoin_distance_m: float,
    offset_m: float,
    transition_length_m: float
) -> float:
    var ramp_in_end := start_distance_m + transition_length_m
    var ramp_out_start := rejoin_distance_m - transition_length_m
    if distance_m <= ramp_in_end:
        return offset_m * _smoothstep(
            clampf((distance_m - start_distance_m) / transition_length_m, 0.0, 1.0)
        )
    if distance_m >= ramp_out_start:
        return offset_m * (1.0 - _smoothstep(
            clampf((distance_m - ramp_out_start) / transition_length_m, 0.0, 1.0)
        ))
    return offset_m

func _offset_point(
    route_navigator: RefCounted,
    distance_m: float,
    start_distance_m: float,
    rejoin_distance_m: float,
    offset_m: float,
    transition_length_m: float
) -> Vector2:
    var center: Vector2 = route_navigator.get_point_at_distance(distance_m)
    var tangent := _route_tangent(route_navigator, distance_m)
    var right := Vector2(tangent.y, -tangent.x)
    var lateral_offset := _offset_at_distance(
        distance_m,
        start_distance_m,
        rejoin_distance_m,
        offset_m,
        transition_length_m
    )
    return center + right * lateral_offset

func _offset_point_tangent(
    route_navigator: RefCounted,
    distance_m: float,
    start_distance_m: float,
    rejoin_distance_m: float,
    offset_m: float,
    transition_length_m: float
) -> Vector2:
    var sample_delta_m: float = 0.05
    var before := _offset_point(
        route_navigator,
        maxf(distance_m - sample_delta_m, start_distance_m),
        start_distance_m,
        rejoin_distance_m,
        offset_m,
        transition_length_m
    )
    var after := _offset_point(
        route_navigator,
        minf(distance_m + sample_delta_m, rejoin_distance_m),
        start_distance_m,
        rejoin_distance_m,
        offset_m,
        transition_length_m
    )
    return (after - before).normalized()

func _resample_uniform(
    source_points: Array[Vector2],
    spacing_m: float
) -> Array[Vector2]:
    var result: Array[Vector2] = [source_points[0]]
    var source_index: int = 1
    var previous := source_points[0]
    var remaining_to_sample: float = spacing_m
    while source_index < source_points.size():
        var current := source_points[source_index]
        var segment := current - previous
        var segment_length: float = segment.length()
        if segment_length <= 0.000001:
            previous = current
            source_index += 1
            continue
        if segment_length >= remaining_to_sample:
            var sample := previous + segment.normalized() * remaining_to_sample
            result.append(sample)
            previous = sample
            remaining_to_sample = spacing_m
        else:
            remaining_to_sample -= segment_length
            previous = current
            source_index += 1
    if result[-1].distance_to(source_points[-1]) > 0.001:
        result.append(source_points[-1])
    return result

func _swept_clearance(points: Array[Vector2], obstacle_rectangle: Array[Vector2]) -> Dictionary:
    var minimum_clearance_m: float = INF
    var path_distance_m: float = 0.0
    var minimum_path_distance_m: float = 0.0
    for index in range(points.size()):
        if index > 0:
            path_distance_m += points[index - 1].distance_to(points[index])
        var tangent: Vector2
        if index == 0:
            tangent = (points[1] - points[0]).normalized()
        elif index == points.size() - 1:
            tangent = (points[index] - points[index - 1]).normalized()
        else:
            tangent = (points[index + 1] - points[index - 1]).normalized()
        var right := Vector2(tangent.y, -tangent.x)
        var shuttle_rectangle := _rectangle(
            points[index],
            tangent,
            right,
            VEHICLE_LENGTH_M,
            VEHICLE_WIDTH_M
        )
        var clearance := _oriented_rectangle_clearance(
            shuttle_rectangle,
            obstacle_rectangle
        )
        if clearance < minimum_clearance_m:
            minimum_clearance_m = clearance
            minimum_path_distance_m = path_distance_m
    return {
        "minimum_clearance_m": minimum_clearance_m,
        "path_distance_m": minimum_path_distance_m
    }

func _oriented_rectangle_clearance(a: Array[Vector2], b: Array[Vector2]) -> float:
    var axes: Array[Vector2] = [
        (a[1] - a[0]).normalized(),
        (a[0] - a[3]).normalized(),
        (b[1] - b[0]).normalized(),
        (b[0] - b[3]).normalized()
    ]
    var minimum_overlap: float = INF
    var maximum_gap: float = 0.0
    for axis in axes:
        var a_projection := _project(a, axis)
        var b_projection := _project(b, axis)
        var gap: float = maxf(
            b_projection.x - a_projection.y,
            a_projection.x - b_projection.y
        )
        if gap > 0.0:
            maximum_gap = maxf(maximum_gap, gap)
        else:
            minimum_overlap = minf(
                minimum_overlap,
                minf(a_projection.y, b_projection.y)
                    - maxf(a_projection.x, b_projection.x)
            )
    return maximum_gap if maximum_gap > 0.0 else -minimum_overlap

func _project(points: Array[Vector2], axis: Vector2) -> Vector2:
    var minimum: float = INF
    var maximum: float = -INF
    for point in points:
        var value: float = point.dot(axis)
        minimum = minf(minimum, value)
        maximum = maxf(maximum, value)
    return Vector2(minimum, maximum)

func _rectangle(
    center: Vector2,
    forward: Vector2,
    right: Vector2,
    length_m: float,
    width_m: float
) -> Array[Vector2]:
    return [
        center + forward * length_m * 0.5 + right * width_m * 0.5,
        center + forward * length_m * 0.5 - right * width_m * 0.5,
        center - forward * length_m * 0.5 - right * width_m * 0.5,
        center - forward * length_m * 0.5 + right * width_m * 0.5
    ]

func _measure_path_curvature(points: Array[Vector2]) -> Dictionary:
    var maximum: float = 0.0
    var maximum_index: int = -1
    var window: int = 3
    for index in range(window, points.size() - window):
        var first: Vector2 = points[index] - points[index - window]
        var second: Vector2 = points[index + window] - points[index]
        if first.length() <= 0.001 || second.length() <= 0.001:
            continue
        var heading_change: float = absf(
            wrapf(first.angle_to(second), -PI, PI)
        )
        var curvature: float = heading_change \
            / maxf((first.length() + second.length()) * 0.5, 0.001)
        if curvature > maximum:
            maximum = curvature
            maximum_index = index
    return {
        "max_curvature_per_m": maximum,
        "max_curvature_index": maximum_index
    }

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
