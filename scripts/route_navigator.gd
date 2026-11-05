extends RefCounted

const DEGENERATE_SEGMENT_EPSILON_M: float = 0.0001
# Local tracking is deliberately bounded; callers should inspect distance_m and relocalize if it grows too large.
const LOCAL_BACKTRACK_WINDOW_M: float = 1.0
const LOCAL_FORWARD_WINDOW_M: float = 6.0

# Route Vector2 values are world (X, Z); positive cross-track error means right of the route's forward direction.
var _centerline: PackedVector2Array = PackedVector2Array()
var _cumulative_lengths: PackedFloat64Array = PackedFloat64Array()
var _progress_m: float = 0.0
var _last_segment_index: int = 0
var _is_localized: bool = false

var route_valid: bool:
    get:
        return _centerline.size() >= 2

var total_length_m: float:
    get:
        if _cumulative_lengths.is_empty():
            return 0.0
        return _cumulative_lengths[_cumulative_lengths.size() - 1]

var progress_m: float:
    get:
        return _progress_m

var is_localized: bool:
    get:
        return _is_localized

static func create_broad_bend_waypoints() -> Array[Vector2]:
    # World coordinates are (X, Z): this route starts along +Z and bends gradually toward +X.
    return [
        Vector2(0.0, 0.0),
        Vector2(0.0, 20.0),
        Vector2(0.8, 40.0),
        Vector2(4.5, 60.0),
        Vector2(13.0, 78.0),
        Vector2(27.0, 94.0)
    ]

func set_route(waypoints: Array[Vector2], samples_per_segment: int = 20) -> bool:
    clear_route()
    if waypoints.size() < 2 || samples_per_segment < 1:
        return false

    for i in range(waypoints.size()):
        if not waypoints[i].is_finite():
            return false
        if i > 0 && waypoints[i].distance_to(waypoints[i - 1]) <= DEGENERATE_SEGMENT_EPSILON_M:
            return false

    var samples := PackedVector2Array()
    samples.append(waypoints[0])
    for segment_index in range(waypoints.size() - 1):
        var p0: Vector2 = waypoints[maxi(segment_index - 1, 0)]
        var p1: Vector2 = waypoints[segment_index]
        var p2: Vector2 = waypoints[segment_index + 1]
        var p3: Vector2 = waypoints[mini(segment_index + 2, waypoints.size() - 1)]
        for sample_index in range(1, samples_per_segment + 1):
            var t: float = float(sample_index) / float(samples_per_segment)
            samples.append(_catmull_rom(p0, p1, p2, p3, t))

    var cumulative := PackedFloat64Array()
    cumulative.resize(samples.size())
    cumulative[0] = 0.0
    for sample_index in range(1, samples.size()):
        var segment_length: float = samples[sample_index].distance_to(samples[sample_index - 1])
        if segment_length <= DEGENERATE_SEGMENT_EPSILON_M:
            return false
        cumulative[sample_index] = cumulative[sample_index - 1] + segment_length

    _centerline = samples
    _cumulative_lengths = cumulative
    return true

func clear_route() -> void:
    _centerline = PackedVector2Array()
    _cumulative_lengths = PackedFloat64Array()
    reset_progress()

func reset_progress() -> void:
    _progress_m = 0.0
    _last_segment_index = 0
    _is_localized = false

func relocalize(vehicle_position_xz: Vector2) -> Dictionary:
    reset_progress()
    return find_nearest_point(vehicle_position_xz, true)

func find_nearest_point(vehicle_position_xz: Vector2, force_global_search: bool = false) -> Dictionary:
    if not route_valid || not vehicle_position_xz.is_finite():
        return _empty_nearest_result()

    var local_search: bool = _is_localized && not force_global_search
    var minimum_progress: float = 0.0
    var maximum_progress: float = total_length_m
    if local_search:
        minimum_progress = maxf(0.0, _progress_m - LOCAL_BACKTRACK_WINDOW_M)
        maximum_progress = minf(total_length_m, _progress_m + LOCAL_FORWARD_WINDOW_M)

    var best_distance_squared: float = INF
    var best_progress: float = _progress_m if local_search else 0.0
    var best_segment_index: int = _last_segment_index if local_search else 0
    var best_position: Vector2 = Vector2.ZERO
    var best_tangent: Vector2 = Vector2.ZERO

    for segment_index in range(_centerline.size() - 1):
        var segment_start_progress: float = _cumulative_lengths[segment_index]
        var segment_end_progress: float = _cumulative_lengths[segment_index + 1]
        if segment_end_progress < minimum_progress || segment_start_progress > maximum_progress:
            continue

        var segment_start: Vector2 = _centerline[segment_index]
        var segment: Vector2 = _centerline[segment_index + 1] - segment_start
        var segment_length_squared: float = segment.length_squared()
        var fraction: float = clampf(
            (vehicle_position_xz - segment_start).dot(segment) / segment_length_squared,
            0.0,
            1.0
        )
        var projected_progress: float = segment_start_progress + fraction * (
            segment_end_progress - segment_start_progress
        )
        projected_progress = clampf(projected_progress, minimum_progress, maximum_progress)
        var projected_position: Vector2 = _point_at_distance(projected_progress)
        var distance_squared: float = vehicle_position_xz.distance_squared_to(projected_position)
        if distance_squared < best_distance_squared:
            best_distance_squared = distance_squared
            best_progress = projected_progress
            best_segment_index = segment_index
            best_position = projected_position
            best_tangent = segment.normalized()

    if best_distance_squared == INF:
        return _empty_nearest_result()

    if local_search:
        best_progress = maxf(_progress_m, best_progress)
        best_position = _point_at_distance(best_progress)

    var right_of_route: Vector2 = Vector2(best_tangent.y, -best_tangent.x)
    var signed_cross_track_error: float = (vehicle_position_xz - best_position).dot(right_of_route)
    _progress_m = best_progress
    _last_segment_index = best_segment_index
    _is_localized = true
    return {
        "valid": true,
        "position_xz": best_position,
        "segment_index": best_segment_index,
        "progress_m": best_progress,
        "cross_track_error_m": signed_cross_track_error,
        "distance_m": sqrt(best_distance_squared),
        "localized": true
    }

func get_point_at_distance(distance_m: float) -> Vector2:
    if not route_valid || not is_finite(distance_m):
        return Vector2.ZERO
    return _point_at_distance(clampf(distance_m, 0.0, total_length_m))

func get_lookahead_point(lookahead_distance_m: float) -> Dictionary:
    if not route_valid || not is_finite(lookahead_distance_m) || lookahead_distance_m < 0.0:
        return {
            "valid": false,
            "position_xz": Vector2.ZERO,
            "progress_m": 0.0,
            "clamped_at_endpoint": false
        }
    var requested_progress: float = _progress_m + lookahead_distance_m
    var lookahead_progress: float = minf(requested_progress, total_length_m)
    return {
        "valid": true,
        "position_xz": _point_at_distance(lookahead_progress),
        "progress_m": lookahead_progress,
        "clamped_at_endpoint": requested_progress >= total_length_m
    }

func get_centerline_samples() -> PackedVector2Array:
    return _centerline.duplicate()

func get_cumulative_lengths() -> PackedFloat64Array:
    return _cumulative_lengths.duplicate()

func _point_at_distance(distance_m: float) -> Vector2:
    if not route_valid:
        return Vector2.ZERO
    if distance_m <= 0.0:
        return _centerline[0]
    if distance_m >= total_length_m:
        return _centerline[_centerline.size() - 1]

    var low: int = 0
    var high: int = _cumulative_lengths.size() - 1
    while low + 1 < high:
        var middle: int = (low + high) / 2
        if _cumulative_lengths[middle] < distance_m:
            low = middle
        else:
            high = middle
    var segment_length: float = _cumulative_lengths[high] - _cumulative_lengths[low]
    var fraction: float = (distance_m - _cumulative_lengths[low]) / segment_length
    return _centerline[low].lerp(_centerline[high], fraction)

func _empty_nearest_result() -> Dictionary:
    return {
        "valid": false,
        "position_xz": Vector2.ZERO,
        "segment_index": -1,
        "progress_m": _progress_m,
        "cross_track_error_m": 0.0,
        "distance_m": INF,
        "localized": _is_localized
    }

func _catmull_rom(p0: Vector2, p1: Vector2, p2: Vector2, p3: Vector2, t: float) -> Vector2:
    var t_squared: float = t * t
    var t_cubed: float = t_squared * t
    return 0.5 * (
        2.0 * p1
        + (-p0 + p2) * t
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t_squared
        + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t_cubed
    )
