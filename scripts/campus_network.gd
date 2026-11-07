extends RefCounted

## Prototype locations are fictional and do not represent ADUSTECH campus coordinates.
## Positions use RouteNavigator's world X/Z convention: Vector2.x is X, Vector2.y is Z.
const POSITION_TOLERANCE_M: float = 0.001
const MIN_SEGMENT_LENGTH_M: float = 0.001
const DESIGN_TURN_RADIUS_M: float = 40.0
const JUNCTION_DESIGN_RADIUS_M: float = 12.0
const ARC_STEPS_PER_QUADRANT: int = 8
const CIRCLE_CENTER_XZ: Vector2 = Vector2(0.0, 40.0)

var _nodes: Array[Dictionary] = []
var _segments: Array[Dictionary] = []
var _stations: Array[Dictionary] = []
var _validation_errors: PackedStringArray = PackedStringArray()

var is_valid: bool:
    get:
        return _validation_errors.is_empty()

func _init() -> void:
    _nodes = _build_nodes()
    _segments = _build_segments()
    _stations = _build_stations()
    var report: Dictionary = validate_network_data(_nodes, _segments, _stations)
    _validation_errors = report.errors
    if not report.valid:
        push_error("Default campus network data is invalid: %s" % "; ".join(_validation_errors))

func get_validation_errors() -> PackedStringArray:
    return _validation_errors.duplicate()

func get_station_ids() -> PackedStringArray:
    var ids := PackedStringArray()
    for station in _stations:
        ids.append(station.id)
    return ids

func get_stations() -> Array[Dictionary]:
    return _duplicate_dictionary_array(_stations)

func get_station(station_id: String) -> Dictionary:
    for station in _stations:
        if station.id == station_id:
            return station.duplicate(true)
    return {}

func get_station_stop(station_id: String) -> Dictionary:
    var station: Dictionary = get_station(station_id)
    if station.is_empty():
        return {
            "valid": false,
            "station_id": station_id,
            "node_id": "",
            "position_xz": Vector2.ZERO
        }
    return {
        "valid": true,
        "station_id": station.id,
        "node_id": station.stop_reference.node_id,
        "position_xz": station.stop_position_xz
    }

func get_node_ids() -> PackedStringArray:
    var ids := PackedStringArray()
    for node in _nodes:
        ids.append(node.id)
    return ids

func get_nodes() -> Array[Dictionary]:
    return _duplicate_dictionary_array(_nodes)

func get_node(node_id: String) -> Dictionary:
    for node in _nodes:
        if node.id == node_id:
            return node.duplicate(true)
    return {}

func get_segment_ids() -> PackedStringArray:
    var ids := PackedStringArray()
    for segment in _segments:
        ids.append(segment.id)
    return ids

func get_segments() -> Array[Dictionary]:
    return _duplicate_dictionary_array(_segments)

func get_segment(segment_id: String) -> Dictionary:
    for segment in _segments:
        if segment.id == segment_id:
            return _duplicate_segment(segment)
    return {}

func get_outgoing_segments(node_id: String) -> Array[Dictionary]:
    var outgoing: Array[Dictionary] = []
    for segment in _segments:
        if segment.start_node_id == node_id:
            outgoing.append(_duplicate_segment(segment))
    return outgoing

func are_stations_mutually_reachable() -> bool:
    var report: Dictionary = validate_network_data(_nodes, _segments, _stations)
    return report.directed_reachability_valid

static func validate_network_data(
    nodes: Array,
    segments: Array,
    stations: Array
) -> Dictionary:
    var errors := PackedStringArray()
    var node_by_id: Dictionary = {}
    var valid_node_positions: Dictionary = {}
    var segment_by_id: Dictionary = {}
    var station_by_id: Dictionary = {}
    var station_node_by_id: Dictionary = {}

    for raw_node in nodes:
        if not raw_node is Dictionary:
            errors.append("node entry must be a Dictionary")
            continue
        var node: Dictionary = raw_node
        var node_id: String = str(node.get("id", ""))
        if node_id.is_empty():
            errors.append("node ID must not be empty")
        elif node_by_id.has(node_id):
            errors.append("duplicate node ID: " + node_id)
        else:
            node_by_id[node_id] = node
        if not node.get("position_xz", null) is Vector2:
            errors.append("node %s position must be Vector2" % node_id)
        elif not node.position_xz.is_finite():
            errors.append("node %s position must be finite" % node_id)
        else:
            valid_node_positions[node_id] = node.position_xz
        if not _is_positive_finite_number(node.get("junction_design_radius_m", null)):
            errors.append("node %s junction design radius must be positive and finite" % node_id)
        if not node.get("allowed_turns", null) is Array:
            errors.append("node %s allowed_turns must be an Array" % node_id)

    for raw_segment in segments:
        if not raw_segment is Dictionary:
            errors.append("segment entry must be a Dictionary")
            continue
        var segment: Dictionary = raw_segment
        var segment_id: String = str(segment.get("id", ""))
        if segment_id.is_empty():
            errors.append("segment ID must not be empty")
        elif segment_by_id.has(segment_id):
            errors.append("duplicate segment ID: " + segment_id)
        else:
            segment_by_id[segment_id] = segment

        var start_id: String = str(segment.get("start_node_id", ""))
        var end_id: String = str(segment.get("end_node_id", ""))
        if not node_by_id.has(start_id):
            errors.append("segment %s references missing start node %s" % [segment_id, start_id])
        if not node_by_id.has(end_id):
            errors.append("segment %s references missing end node %s" % [segment_id, end_id])

        var points_value: Variant = segment.get("centerline_points", null)
        if not points_value is Array:
            errors.append("segment %s centerline_points must be an Array" % segment_id)
            continue
        var points: Array = points_value
        if points.size() < 2:
            errors.append("segment %s centerline must have at least two points" % segment_id)
            continue

        var measured_length_m: float = 0.0
        var points_valid: bool = true
        for point_index in range(points.size()):
            if not points[point_index] is Vector2:
                errors.append("segment %s centerline point must be Vector2" % segment_id)
                points_valid = false
                continue
            var point: Vector2 = points[point_index]
            if not point.is_finite():
                errors.append("segment %s centerline point must be finite" % segment_id)
                points_valid = false
            if point_index > 0 && points[point_index - 1] is Vector2:
                var previous_point: Vector2 = points[point_index - 1]
                var point_distance_m: float = previous_point.distance_to(point)
                if point_distance_m <= MIN_SEGMENT_LENGTH_M:
                    errors.append("segment %s contains a degenerate centerline edge" % segment_id)
                    points_valid = false
                measured_length_m += point_distance_m

        if points_valid:
            if not valid_node_positions.has(start_id) || not valid_node_positions.has(end_id):
                continue
            var start_position: Vector2 = valid_node_positions[start_id]
            var end_position: Vector2 = valid_node_positions[end_id]
            if points[0].distance_to(start_position) > POSITION_TOLERANCE_M:
                errors.append("segment %s start point does not align with node %s" % [segment_id, start_id])
            if points[points.size() - 1].distance_to(end_position) > POSITION_TOLERANCE_M:
                errors.append("segment %s end point does not align with node %s" % [segment_id, end_id])
            var declared_length: Variant = segment.get("length_m", null)
            if not _is_positive_finite_number(declared_length):
                errors.append("segment %s length must be positive and finite" % segment_id)
            elif absf(float(declared_length) - measured_length_m) > POSITION_TOLERANCE_M:
                errors.append("segment %s declared length does not match centerline" % segment_id)
            if not _is_positive_finite_number(segment.get("design_turn_radius_m", null)):
                errors.append("segment %s design turn radius must be positive and finite" % segment_id)

    for raw_station in stations:
        if not raw_station is Dictionary:
            errors.append("station entry must be a Dictionary")
            continue
        var station: Dictionary = raw_station
        var station_id: String = str(station.get("id", ""))
        if station_id.is_empty():
            errors.append("station ID must not be empty")
        elif station_by_id.has(station_id):
            errors.append("duplicate station ID: " + station_id)
        else:
            station_by_id[station_id] = station
        if str(station.get("display_name", "")).strip_edges().is_empty():
            errors.append("station %s display name must not be empty" % station_id)

        var stop_reference: Variant = station.get("stop_reference", null)
        if not stop_reference is Dictionary:
            errors.append("station %s stop_reference must be a Dictionary" % station_id)
            continue
        var stop_node_id: String = str(stop_reference.get("node_id", ""))
        if not valid_node_positions.has(stop_node_id):
            errors.append("station %s references missing stop node %s" % [station_id, stop_node_id])
            continue
        var stop_position_value: Variant = station.get("stop_position_xz", null)
        if not stop_position_value is Vector2:
            errors.append("station %s stop position must be Vector2" % station_id)
        elif not stop_position_value.is_finite():
            errors.append("station %s stop position must be finite" % station_id)
        elif stop_position_value.distance_to(valid_node_positions[stop_node_id]) > POSITION_TOLERANCE_M:
            errors.append("station %s stop position does not align with its node" % station_id)
        else:
            station_node_by_id[station_id] = stop_node_id

    for node_id in node_by_id:
        var node: Dictionary = node_by_id[node_id]
        for raw_turn in node.get("allowed_turns", []):
            if not raw_turn is Dictionary:
                errors.append("node %s turn entry must be a Dictionary" % node_id)
                continue
            var turn: Dictionary = raw_turn
            var incoming_id: String = str(turn.get("incoming_segment_id", ""))
            var outgoing_id: String = str(turn.get("outgoing_segment_id", ""))
            if not segment_by_id.has(incoming_id) || not segment_by_id.has(outgoing_id):
                errors.append("node %s turn references a missing segment" % node_id)
                continue
            if segment_by_id[incoming_id].end_node_id != node_id \
                || segment_by_id[outgoing_id].start_node_id != node_id:
                errors.append("node %s turn segments do not connect at the node" % node_id)

    var reachable: bool = station_node_by_id.size() == station_by_id.size() \
        && station_by_id.size() >= 2
    for start_station_id in station_node_by_id:
        var start_node_id: String = station_node_by_id[start_station_id]
        var reached_nodes: Dictionary = {start_node_id: true}
        var visited_segments: Dictionary = {}
        var segments_to_visit: Array[String] = []
        for segment in segment_by_id.values():
            if segment.start_node_id == start_node_id:
                segments_to_visit.append(segment.id)
        var visit_index: int = 0
        while visit_index < segments_to_visit.size():
            var incoming_segment_id: String = segments_to_visit[visit_index]
            visit_index += 1
            if visited_segments.has(incoming_segment_id):
                continue
            visited_segments[incoming_segment_id] = true
            var incoming_segment: Dictionary = segment_by_id[incoming_segment_id]
            var junction_id: String = incoming_segment.end_node_id
            reached_nodes[junction_id] = true
            var junction: Dictionary = node_by_id.get(junction_id, {})
            for turn in junction.get("allowed_turns", []):
                if not turn is Dictionary \
                    || turn.get("incoming_segment_id", "") != incoming_segment_id:
                    continue
                var outgoing_segment_id: String = str(turn.get("outgoing_segment_id", ""))
                if segment_by_id.has(outgoing_segment_id):
                    segments_to_visit.append(outgoing_segment_id)
        for destination_station_id in station_node_by_id:
            var destination_node_id: String = station_node_by_id[destination_station_id]
            if not reached_nodes.has(destination_node_id):
                reachable = false
                errors.append(
                    "station %s cannot reach station %s through directed segments"
                    % [start_station_id, destination_station_id]
                )

    return {
        "valid": errors.is_empty(),
        "errors": errors,
        "directed_reachability_valid": reachable
    }

static func _is_positive_finite_number(value: Variant) -> bool:
    return (value is int || value is float) && is_finite(float(value)) && float(value) > 0.0

func _build_nodes() -> Array[Dictionary]:
    var nodes: Array[Dictionary] = [
        {
            "id": "junction_north_gate",
            "position_xz": Vector2(0.0, 0.0),
            "junction_design_radius_m": JUNCTION_DESIGN_RADIUS_M,
            "allowed_turns": [
                _turn("west_to_north", "north_to_east"),
                _turn("east_to_north", "north_to_west")
            ]
        },
        {
            "id": "junction_library",
            "position_xz": Vector2(40.0, 40.0),
            "junction_design_radius_m": JUNCTION_DESIGN_RADIUS_M,
            "allowed_turns": [
                _turn("north_to_east", "east_to_south"),
                _turn("south_to_east", "east_to_north")
            ]
        },
        {
            "id": "junction_sports_hall",
            "position_xz": Vector2(0.0, 80.0),
            "junction_design_radius_m": JUNCTION_DESIGN_RADIUS_M,
            "allowed_turns": [
                _turn("east_to_south", "south_to_west"),
                _turn("west_to_south", "south_to_east")
            ]
        },
        {
            "id": "junction_innovation_centre",
            "position_xz": Vector2(-40.0, 40.0),
            "junction_design_radius_m": JUNCTION_DESIGN_RADIUS_M,
            "allowed_turns": [
                _turn("south_to_west", "west_to_north"),
                _turn("north_to_west", "west_to_south")
            ]
        }
    ]
    return nodes

func _build_segments() -> Array[Dictionary]:
    var segments: Array[Dictionary] = []
    var clockwise_arcs: Array[Dictionary] = [
        {"id": "north_to_east", "reverse_id": "east_to_north", "start": "junction_north_gate", "end": "junction_library", "start_angle": -PI / 2.0},
        {"id": "east_to_south", "reverse_id": "south_to_east", "start": "junction_library", "end": "junction_sports_hall", "start_angle": 0.0},
        {"id": "south_to_west", "reverse_id": "west_to_south", "start": "junction_sports_hall", "end": "junction_innovation_centre", "start_angle": PI / 2.0},
        {"id": "west_to_north", "reverse_id": "north_to_west", "start": "junction_innovation_centre", "end": "junction_north_gate", "start_angle": PI}
    ]
    for arc in clockwise_arcs:
        var points: Array[Vector2] = _quarter_circle_points(float(arc.start_angle))
        segments.append(_make_segment(
            arc.id,
            arc.start,
            arc.end,
            points
        ))
    for arc in clockwise_arcs:
        var reverse_points: Array[Vector2] = _reverse_points(_quarter_circle_points(float(arc.start_angle)))
        segments.append(_make_segment(
            arc.reverse_id,
            arc.end,
            arc.start,
            reverse_points
        ))
    return segments

func _build_stations() -> Array[Dictionary]:
    return [
        _station("north_gate", "North Gate Stop", "junction_north_gate"),
        _station("library", "Library Stop", "junction_library"),
        _station("sports_hall", "Sports Hall Stop", "junction_sports_hall"),
        _station("innovation_centre", "Innovation Centre Stop", "junction_innovation_centre")
    ]

func _station(station_id: String, display_name: String, node_id: String) -> Dictionary:
    var node: Dictionary = get_node(node_id)
    return {
        "id": station_id,
        "display_name": display_name,
        "stop_reference": {"node_id": node_id},
        "stop_position_xz": node.position_xz
    }

func _make_segment(
    segment_id: String,
    start_node_id: String,
    end_node_id: String,
    points: Array[Vector2]
) -> Dictionary:
    var length_m: float = 0.0
    for index in range(1, points.size()):
        length_m += points[index - 1].distance_to(points[index])
    return {
        "id": segment_id,
        "start_node_id": start_node_id,
        "end_node_id": end_node_id,
        "centerline_points": points,
        "length_m": length_m,
        "design_turn_radius_m": DESIGN_TURN_RADIUS_M
    }

func _quarter_circle_points(start_angle: float) -> Array[Vector2]:
    var points: Array[Vector2] = []
    for step in range(ARC_STEPS_PER_QUADRANT + 1):
        var angle: float = start_angle + (PI / 2.0) * float(step) / float(ARC_STEPS_PER_QUADRANT)
        points.append(CIRCLE_CENTER_XZ + Vector2(cos(angle), sin(angle)) * DESIGN_TURN_RADIUS_M)
    return points

func _reverse_points(points: Array[Vector2]) -> Array[Vector2]:
    var reversed: Array[Vector2] = []
    for index in range(points.size() - 1, -1, -1):
        reversed.append(points[index])
    return reversed

func _turn(incoming_segment_id: String, outgoing_segment_id: String) -> Dictionary:
    return {
        "incoming_segment_id": incoming_segment_id,
        "outgoing_segment_id": outgoing_segment_id
    }

func _duplicate_dictionary_array(source: Array[Dictionary]) -> Array[Dictionary]:
    var copies: Array[Dictionary] = []
    for entry in source:
        if entry.has("centerline_points"):
            copies.append(_duplicate_segment(entry))
        else:
            copies.append(entry.duplicate(true))
    return copies

func _duplicate_segment(segment: Dictionary) -> Dictionary:
    var copy: Dictionary = segment.duplicate(true)
    var points: Array[Vector2] = []
    for point in segment.centerline_points:
        points.append(point)
    copy.centerline_points = points
    return copy
