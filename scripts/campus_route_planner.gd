extends RefCounted

## Plans geometrically screened routes; it does not certify closed-loop vehicle tracking.
## RouteNavigator's spline is checked for curvature and deviation from the network centerline,
## but road-surface containment and vehicle tracking still require later integration tests.
const CampusNetwork = preload("res://scripts/campus_network.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")

const POSITION_TOLERANCE_M: float = 0.001
const LENGTH_TOLERANCE_M: float = 0.001
const COST_COMPARISON_EPSILON_M: float = 0.000001
const MAX_HEADING_CHANGE_RAD: float = deg_to_rad(12.0)
const TURN_RADIUS_SAFETY_FACTOR: float = 1.25
const NAVIGATOR_DEVIATION_TOLERANCE_M: float = 0.25
const CURVATURE_EPSILON: float = 0.000001

var _network: RefCounted
var _vehicle_dynamics: RefCounted

func _init(network: RefCounted = null, vehicle_dynamics: RefCounted = null) -> void:
    _network = CampusNetwork.new() if network == null else network
    _vehicle_dynamics = VehicleDynamics.new() if vehicle_dynamics == null else vehicle_dynamics

## Returns the shortest legal, locally feasible route between two distinct stations.
## A valid result is planner-level geometry only; tracking_validated is always false.
func plan_route(origin_station_id: String, destination_station_id: String) -> Dictionary:
    if _network == null || not _network.is_valid:
        var network_errors: PackedStringArray = PackedStringArray()
        if _network != null && _network.has_method("get_validation_errors"):
            network_errors = _network.get_validation_errors()
        return _failure("invalid_network", {"validation_errors": network_errors})
    if origin_station_id.is_empty() || _network.get_station(origin_station_id).is_empty():
        return _failure("unknown_origin_station", {"station_id": origin_station_id})
    if destination_station_id.is_empty() || _network.get_station(destination_station_id).is_empty():
        return _failure("unknown_destination_station", {"station_id": destination_station_id})
    if origin_station_id == destination_station_id:
        return _failure("same_station_not_allowed", {"station_id": origin_station_id})

    var dynamics_limits: Dictionary = _get_dynamics_limits()
    if not dynamics_limits.valid:
        return _failure("invalid_vehicle_dynamics_limits", {"detail": dynamics_limits.reason})

    var origin_stop: Dictionary = _network.get_station_stop(origin_station_id)
    var destination_stop: Dictionary = _network.get_station_stop(destination_station_id)
    if not origin_stop.get("valid", false) || not destination_stop.get("valid", false):
        return _failure("invalid_station_stop")

    var segment_cache: Dictionary = {}
    var segment_errors: Dictionary = {}
    for segment_id in _network.get_segment_ids():
        var result: Dictionary = _validate_segment(str(segment_id), dynamics_limits)
        if result.valid:
            segment_cache[str(segment_id)] = result.segment
        else:
            segment_errors[str(segment_id)] = result.reason

    var search: Dictionary = _find_shortest_path(
        origin_stop.node_id,
        destination_stop.node_id,
        segment_cache,
        segment_errors,
        dynamics_limits
    )
    if not search.found:
        return _failure(search.reason, {"details": search.details})

    return _build_route_result(
        search.segment_ids,
        origin_stop,
        destination_stop,
        segment_cache,
        dynamics_limits
    )

func _find_shortest_path(
    origin_node_id: String,
    destination_node_id: String,
    segment_cache: Dictionary,
    segment_errors: Dictionary,
    dynamics_limits: Dictionary
) -> Dictionary:
    var distances: Dictionary = {}
    var paths: Dictionary = {}
    var pending: Dictionary = {}
    var geometry_failures := PackedStringArray()

    for segment in _network.get_outgoing_segments(origin_node_id):
        var segment_id: String = str(segment.id)
        if not segment_cache.has(segment_id):
            if segment_errors.has(segment_id):
                geometry_failures.append(
                    "origin segment %s is geometrically unavailable: %s"
                    % [segment_id, segment_errors[segment_id]]
                )
            continue
        var canonical_segment: Dictionary = segment_cache[segment_id]
        if canonical_segment.start_node_id != origin_node_id:
            geometry_failures.append(
                "origin segment %s is not directed from station node %s"
                % [segment_id, origin_node_id]
            )
            continue
        var start_cost: float = canonical_segment.length_m
        if not distances.has(segment_id) || start_cost < distances[segment_id]:
            distances[segment_id] = start_cost
            paths[segment_id] = [segment_id]
            pending[segment_id] = true

    while not pending.is_empty():
        var current_id: String = _select_next_segment(pending, distances, paths)
        pending.erase(current_id)
        var current_segment: Dictionary = segment_cache[current_id]
        var current_path: Array = paths[current_id]
        if current_segment.end_node_id == destination_node_id:
            return {"found": true, "segment_ids": current_path}

        var junction: Dictionary = _network.get_node(current_segment.end_node_id)
        if junction.is_empty():
            geometry_failures.append(
                "segment %s ends at missing junction %s"
                % [current_id, current_segment.end_node_id]
            )
            continue
        var allowed_turns: Variant = junction.get("allowed_turns", null)
        if not allowed_turns is Array:
            geometry_failures.append("junction %s has invalid allowed-turn data" % junction.get("id", ""))
            continue

        for raw_turn in allowed_turns:
            if not raw_turn is Dictionary \
                || str(raw_turn.get("incoming_segment_id", "")) != current_id:
                continue
            var next_id: String = str(raw_turn.get("outgoing_segment_id", ""))
            if not segment_cache.has(next_id):
                if segment_errors.has(next_id):
                    geometry_failures.append(
                        "permitted transition %s -> %s is unavailable: %s"
                        % [current_id, next_id, segment_errors[next_id]]
                    )
                continue
            var next_segment: Dictionary = segment_cache[next_id]
            var transition: Dictionary = _validate_junction_transition(
                current_segment,
                next_segment,
                junction,
                dynamics_limits
            )
            if not transition.valid:
                geometry_failures.append(
                    "transition %s -> %s at %s: %s"
                    % [current_id, next_id, junction.get("id", ""), transition.reason]
                )
                continue

            var candidate_cost: float = float(distances[current_id]) + next_segment.length_m
            var candidate_path: Array = current_path.duplicate()
            candidate_path.append(next_id)
            var should_update: bool = not distances.has(next_id) \
                || candidate_cost < float(distances[next_id]) - COST_COMPARISON_EPSILON_M
            if not should_update && distances.has(next_id) \
                && absf(candidate_cost - float(distances[next_id])) <= COST_COMPARISON_EPSILON_M:
                should_update = _path_precedes(candidate_path, paths[next_id])
            if should_update:
                distances[next_id] = candidate_cost
                paths[next_id] = candidate_path
                pending[next_id] = true

    if not geometry_failures.is_empty():
        return {
            "found": false,
            "reason": "geometric_infeasibility",
            "details": geometry_failures
        }
    return {
        "found": false,
        "reason": "no_legal_route",
        "details": PackedStringArray([
            "no permitted directed-segment path connects the station stop nodes"
        ])
    }

func _select_next_segment(pending: Dictionary, distances: Dictionary, paths: Dictionary) -> String:
    var selected_id: String = ""
    for raw_id in pending:
        var segment_id: String = str(raw_id)
        if selected_id.is_empty():
            selected_id = segment_id
            continue
        var cost: float = float(distances[segment_id])
        var selected_cost: float = float(distances[selected_id])
        if cost < selected_cost - COST_COMPARISON_EPSILON_M \
            || (absf(cost - selected_cost) <= COST_COMPARISON_EPSILON_M \
                && _path_precedes(paths[segment_id], paths[selected_id])):
            selected_id = segment_id
    return selected_id

func _path_precedes(candidate: Array, current: Array) -> bool:
    var shared_length: int = mini(candidate.size(), current.size())
    for index in range(shared_length):
        var candidate_id: String = str(candidate[index])
        var current_id: String = str(current[index])
        if candidate_id != current_id:
            return candidate_id < current_id
    return candidate.size() < current.size()

func _validate_segment(segment_id: String, dynamics_limits: Dictionary) -> Dictionary:
    var segment: Dictionary = _network.get_segment(segment_id)
    if segment.is_empty():
        return {"valid": false, "reason": "segment %s is missing" % segment_id}
    var points_value: Variant = segment.get("centerline_points", null)
    if not points_value is Array || points_value.size() < 2:
        return {"valid": false, "reason": "segment %s has fewer than two centerline points" % segment_id}
    var points: Array = points_value
    var start_node: Dictionary = _network.get_node(str(segment.get("start_node_id", "")))
    var end_node: Dictionary = _network.get_node(str(segment.get("end_node_id", "")))
    if start_node.is_empty() || end_node.is_empty():
        return {"valid": false, "reason": "segment %s references a missing endpoint node" % segment_id}
    if not start_node.get("position_xz", Vector2.ZERO) is Vector2 \
        || not end_node.get("position_xz", Vector2.ZERO) is Vector2:
        return {"valid": false, "reason": "segment %s endpoint node has invalid position data" % segment_id}

    var canonical_points: Array[Vector2] = []
    var measured_length_m: float = 0.0
    for index in range(points.size()):
        if not points[index] is Vector2 || not points[index].is_finite():
            return {"valid": false, "reason": "segment %s contains invalid coordinates" % segment_id}
        var point: Vector2 = points[index]
        canonical_points.append(point)
        if index > 0:
            var edge_length_m: float = canonical_points[index - 1].distance_to(point)
            if edge_length_m <= POSITION_TOLERANCE_M:
                return {"valid": false, "reason": "segment %s contains a degenerate edge" % segment_id}
            measured_length_m += edge_length_m

    var start_position: Vector2 = start_node.position_xz
    var end_position: Vector2 = end_node.position_xz
    if canonical_points[0].distance_to(start_position) > POSITION_TOLERANCE_M \
        || canonical_points[canonical_points.size() - 1].distance_to(end_position) > POSITION_TOLERANCE_M:
        return {"valid": false, "reason": "segment %s does not align with its declared nodes" % segment_id}
    if not _is_finite_number(segment.get("length_m", null)) \
        || absf(float(segment.length_m) - measured_length_m) > LENGTH_TOLERANCE_M:
        return {"valid": false, "reason": "segment %s has an invalid measured length" % segment_id}

    var geometry: Dictionary = _validate_polyline(canonical_points, dynamics_limits)
    if not geometry.valid:
        return {
            "valid": false,
            "reason": "segment %s: %s" % [segment_id, geometry.reason]
        }
    var canonical_segment: Dictionary = segment.duplicate(true)
    canonical_segment.centerline_points = canonical_points
    canonical_segment.length_m = measured_length_m
    return {"valid": true, "segment": canonical_segment}

func _validate_junction_transition(
    incoming: Dictionary,
    outgoing: Dictionary,
    junction: Dictionary,
    dynamics_limits: Dictionary
) -> Dictionary:
    if incoming.end_node_id != junction.id || outgoing.start_node_id != junction.id:
        return {"valid": false, "reason": "directed segments do not meet at the declared junction"}
    var incoming_points: Array = incoming.centerline_points
    var outgoing_points: Array = outgoing.centerline_points
    var incoming_end: Vector2 = incoming_points[incoming_points.size() - 1]
    var outgoing_start: Vector2 = outgoing_points[0]
    if incoming_end.distance_to(outgoing_start) > POSITION_TOLERANCE_M:
        return {"valid": false, "reason": "segment endpoints are positionally discontinuous"}

    var incoming_tangent: Vector2 = (
        incoming_end - incoming_points[incoming_points.size() - 2]
    ).normalized()
    var outgoing_tangent: Vector2 = (outgoing_points[1] - outgoing_start).normalized()
    var heading_change_rad: float = acos(clampf(
        incoming_tangent.dot(outgoing_tangent),
        -1.0,
        1.0
    ))
    if heading_change_rad > MAX_HEADING_CHANGE_RAD + CURVATURE_EPSILON:
        return {
            "valid": false,
            "reason": "junction heading discontinuity is %.3f degrees (limit %.3f)"
                % [rad_to_deg(heading_change_rad), rad_to_deg(MAX_HEADING_CHANGE_RAD)]
        }

    var junction_radius: Variant = junction.get("junction_design_radius_m", null)
    if not _is_finite_number(junction_radius) \
        || float(junction_radius) < dynamics_limits.minimum_turn_radius_m:
        return {
            "valid": false,
            "reason": "declared junction radius is below the conservative dynamics minimum"
        }

    var join_points: Array[Vector2] = [
        incoming_points[incoming_points.size() - 2],
        incoming_end,
        outgoing_points[1]
    ]
    var join_geometry: Dictionary = _validate_polyline(join_points, dynamics_limits)
    if not join_geometry.valid:
        return {"valid": false, "reason": join_geometry.reason}
    return {"valid": true}

func _build_route_result(
    segment_ids: Array,
    origin_stop: Dictionary,
    destination_stop: Dictionary,
    segment_cache: Dictionary,
    dynamics_limits: Dictionary
) -> Dictionary:
    var centerline_points: Array[Vector2] = []
    var total_length_m: float = 0.0
    for path_index in range(segment_ids.size()):
        var segment: Dictionary = segment_cache[str(segment_ids[path_index])]
        var points: Array = segment.centerline_points
        if path_index == 0:
            for point in points:
                centerline_points.append(point)
        else:
            if centerline_points.is_empty() \
                || centerline_points[centerline_points.size() - 1].distance_to(points[0]) \
                    > POSITION_TOLERANCE_M:
                return _failure(
                    "geometric_infeasibility",
                    {"detail": "segment %s does not join the preceding centerline" % segment.id}
                )
            for point_index in range(1, points.size()):
                centerline_points.append(points[point_index])
        total_length_m += float(segment.length_m)

    if centerline_points.is_empty() \
        || centerline_points[0].distance_to(origin_stop.position_xz) > POSITION_TOLERANCE_M \
        || centerline_points[centerline_points.size() - 1].distance_to(destination_stop.position_xz) \
            > POSITION_TOLERANCE_M:
        return _failure(
            "geometric_infeasibility",
            {"detail": "concatenated route endpoints do not align with station stops"}
        )

    var raw_geometry: Dictionary = _validate_polyline(centerline_points, dynamics_limits)
    if not raw_geometry.valid:
        return _failure("geometric_infeasibility", {"detail": raw_geometry.reason})

    var navigator = RouteNavigator.new()
    if not navigator.set_route(centerline_points):
        return _failure(
            "route_navigator_incompatible",
            {"detail": "RouteNavigator rejected the concatenated network centerline"}
        )
    var navigator_samples: PackedVector2Array = navigator.get_centerline_samples()
    var navigator_points: Array[Vector2] = []
    for point in navigator_samples:
        navigator_points.append(point)
    var navigator_geometry: Dictionary = _validate_polyline(navigator_points, dynamics_limits)
    if not navigator_geometry.valid:
        return _failure(
            "route_navigator_geometry_infeasible",
            {"detail": navigator_geometry.reason}
        )
    var maximum_deviation_m: float = _maximum_deviation_from_polyline(
        navigator_points,
        centerline_points
    )
    if maximum_deviation_m > NAVIGATOR_DEVIATION_TOLERANCE_M:
        return _failure(
            "route_navigator_deviation_exceeded",
            {
                "maximum_deviation_m": maximum_deviation_m,
                "tolerance_m": NAVIGATOR_DEVIATION_TOLERANCE_M
            }
        )
    if navigator_samples[0].distance_to(centerline_points[0]) > POSITION_TOLERANCE_M \
        || navigator_samples[navigator_samples.size() - 1].distance_to(
            centerline_points[centerline_points.size() - 1]
        ) > POSITION_TOLERANCE_M:
        return _failure(
            "route_navigator_incompatible",
            {"detail": "RouteNavigator changed a network route endpoint"}
        )

    var maximum_curvature_per_m: float = maxf(
        raw_geometry.maximum_curvature_per_m,
        navigator_geometry.maximum_curvature_per_m
    )
    var minimum_turn_radius_m: float = INF
    if maximum_curvature_per_m > CURVATURE_EPSILON:
        minimum_turn_radius_m = 1.0 / maximum_curvature_per_m
    return {
        "valid": true,
        "failure_reason": "",
        "origin_station_id": origin_stop.station_id,
        "destination_station_id": destination_stop.station_id,
        "segment_ids": PackedStringArray(segment_ids),
        "total_length_m": total_length_m,
        "centerline_points": centerline_points.duplicate(),
        "maximum_curvature_per_m": maximum_curvature_per_m,
        "minimum_turn_radius_m": minimum_turn_radius_m,
        "minimum_required_turn_radius_m": dynamics_limits.minimum_turn_radius_m,
        "maximum_junction_heading_change_rad": _maximum_route_junction_heading_change(
            segment_ids,
            segment_cache
        ),
        "navigator_maximum_curvature_per_m": navigator_geometry.maximum_curvature_per_m,
        "navigator_maximum_deviation_m": maximum_deviation_m,
        "navigator_compatible": true,
        "road_surface_containment_validated": false,
        "tracking_validated": false
    }

func _validate_polyline(points: Array, dynamics_limits: Dictionary) -> Dictionary:
    if points.size() < 2:
        return {"valid": false, "reason": "centerline has fewer than two points"}
    var maximum_curvature_per_m: float = 0.0
    var maximum_heading_change_rad: float = 0.0
    var previous_tangent: Vector2 = Vector2.ZERO
    for index in range(points.size()):
        if not points[index] is Vector2 || not points[index].is_finite():
            return {"valid": false, "reason": "centerline contains invalid coordinates"}
        if index == 0:
            continue
        var edge: Vector2 = points[index] - points[index - 1]
        if edge.length() <= POSITION_TOLERANCE_M:
            return {"valid": false, "reason": "centerline contains a degenerate edge"}
        var tangent: Vector2 = edge.normalized()
        if index > 1:
            var heading_change: float = acos(clampf(previous_tangent.dot(tangent), -1.0, 1.0))
            maximum_heading_change_rad = maxf(maximum_heading_change_rad, heading_change)
            if heading_change > MAX_HEADING_CHANGE_RAD + CURVATURE_EPSILON:
                return {
                    "valid": false,
                    "reason": "centerline heading discontinuity is %.3f degrees (limit %.3f)"
                        % [rad_to_deg(heading_change), rad_to_deg(MAX_HEADING_CHANGE_RAD)]
                }
            var previous_edge: Vector2 = points[index - 1] - points[index - 2]
            var chord: Vector2 = points[index] - points[index - 2]
            var denominator: float = previous_edge.length() * edge.length() * chord.length()
            if denominator > 0.0:
                var curvature: float = 2.0 * absf(
                    previous_edge.x * edge.y - previous_edge.y * edge.x
                ) / denominator
                maximum_curvature_per_m = maxf(maximum_curvature_per_m, curvature)
            previous_tangent = tangent
        else:
            previous_tangent = tangent

    var maximum_allowed_curvature_per_m: float = 1.0 / dynamics_limits.minimum_turn_radius_m
    if maximum_curvature_per_m > maximum_allowed_curvature_per_m + CURVATURE_EPSILON:
        return {
            "valid": false,
            "reason": "curvature %.6f per m implies radius %.3f m below conservative minimum %.3f m"
                % [
                    maximum_curvature_per_m,
                    1.0 / maximum_curvature_per_m,
                    dynamics_limits.minimum_turn_radius_m
                ]
        }
    return {
        "valid": true,
        "maximum_curvature_per_m": maximum_curvature_per_m,
        "maximum_heading_change_rad": maximum_heading_change_rad
    }

func _maximum_deviation_from_polyline(samples: Array[Vector2], polyline: Array[Vector2]) -> float:
    var maximum_deviation_m: float = 0.0
    for sample in samples:
        var nearest_distance_squared: float = INF
        for index in range(1, polyline.size()):
            var start: Vector2 = polyline[index - 1]
            var edge: Vector2 = polyline[index] - start
            var fraction: float = clampf((sample - start).dot(edge) / edge.length_squared(), 0.0, 1.0)
            nearest_distance_squared = minf(
                nearest_distance_squared,
                sample.distance_squared_to(start + edge * fraction)
            )
        maximum_deviation_m = maxf(maximum_deviation_m, sqrt(nearest_distance_squared))
    return maximum_deviation_m

func _maximum_route_junction_heading_change(
    segment_ids: Array,
    segment_cache: Dictionary
) -> float:
    var maximum_change_rad: float = 0.0
    for index in range(1, segment_ids.size()):
        var incoming: Dictionary = segment_cache[str(segment_ids[index - 1])]
        var outgoing: Dictionary = segment_cache[str(segment_ids[index])]
        var incoming_points: Array = incoming.centerline_points
        var outgoing_points: Array = outgoing.centerline_points
        var incoming_tangent: Vector2 = (
            incoming_points[incoming_points.size() - 1]
            - incoming_points[incoming_points.size() - 2]
        ).normalized()
        var outgoing_tangent: Vector2 = (
            outgoing_points[1] - outgoing_points[0]
        ).normalized()
        maximum_change_rad = maxf(
            maximum_change_rad,
            acos(clampf(incoming_tangent.dot(outgoing_tangent), -1.0, 1.0))
        )
    return maximum_change_rad

func _get_dynamics_limits() -> Dictionary:
    if _vehicle_dynamics == null:
        return {"valid": false, "reason": "vehicle dynamics are unavailable"}
    var wheelbase_m: float = _vehicle_dynamics.wheelbase_m
    var steering_limit_rad: float = _vehicle_dynamics.steering_limit_rad
    if not is_finite(wheelbase_m) || wheelbase_m <= 0.0 \
        || not is_finite(steering_limit_rad) || steering_limit_rad <= 0.0 \
        || steering_limit_rad >= PI / 2.0:
        return {"valid": false, "reason": "wheelbase or steering limit is invalid"}
    var physical_minimum_radius_m: float = wheelbase_m / tan(steering_limit_rad)
    return {
        "valid": true,
        "minimum_turn_radius_m": physical_minimum_radius_m * TURN_RADIUS_SAFETY_FACTOR
    }

func _is_finite_number(value: Variant) -> bool:
    return (value is int || value is float) && is_finite(float(value))

func _failure(reason: String, details: Dictionary = {}) -> Dictionary:
    return {
        "valid": false,
        "failure_reason": reason,
        "details": details,
        "tracking_validated": false,
        "road_surface_containment_validated": false
    }
