extends RefCounted

## Additive fictional navigation scenario; the original CampusNetwork stays unchanged.
const CampusNetwork = preload("res://scripts/campus_network.gd")

const JUNCTION_DESIGN_RADIUS_M: float = 12.0
const DESIGN_TURN_RADIUS_M: float = 40.0
const CURVE_SAMPLES: int = 24
const ALLOWED_TURN_DIRECTIONS: PackedStringArray = ["straight", "left", "right"]

var _nodes: Array[Dictionary] = []
var _segments: Array[Dictionary] = []
var _stations: Array[Dictionary] = []
var _validation_errors: PackedStringArray = PackedStringArray()

var is_valid: bool:
    get:
        return _validation_errors.is_empty()

func _init() -> void:
    var ring := CampusNetwork.new()
    _nodes = ring.get_nodes()
    _segments = ring.get_segments()
    _stations = ring.get_stations()

    var north_position: Vector2 = ring.get_station_stop("north_gate").position_xz
    var library_position: Vector2 = ring.get_station_stop("library").position_xz
    var bypass_junction := {
        "id": "junction_bypass_mid",
        "position_xz": Vector2(40.0, 20.0),
        "junction_design_radius_m": JUNCTION_DESIGN_RADIUS_M,
        "allowed_turns": []
    }
    _nodes.append(bypass_junction)

    var bypass_curve: Array[Vector2] = _north_to_bypass_points(
        north_position,
        bypass_junction.position_xz
    )
    var bypass_left: Array[Vector2] = _bypass_to_library_points(
        bypass_junction.position_xz,
        library_position,
        -1.0
    )
    var bypass_right: Array[Vector2] = _bypass_to_library_points(
        bypass_junction.position_xz,
        library_position,
        1.0
    )
    _segments.append(_make_segment(
        "north_to_bypass_mid",
        "junction_north_gate",
        "junction_bypass_mid",
        bypass_curve
    ))
    _segments.append(_make_segment(
        "bypass_mid_to_north",
        "junction_bypass_mid",
        "junction_north_gate",
        _reverse_points(bypass_curve)
    ))
    _segments.append(_make_segment(
        "bypass_left_to_library",
        "junction_bypass_mid",
        "junction_library",
        bypass_left
    ))
    _segments.append(_make_segment(
        "library_to_bypass_left",
        "junction_library",
        "junction_bypass_mid",
        _reverse_points(bypass_left)
    ))
    _segments.append(_make_segment(
        "bypass_right_to_library",
        "junction_bypass_mid",
        "junction_library",
        bypass_right
    ))
    _segments.append(_make_segment(
        "library_to_bypass_right",
        "junction_library",
        "junction_bypass_mid",
        _reverse_points(bypass_right)
    ))
    var bypass_straight: Array[Vector2] = []
    for index in range(CURVE_SAMPLES + 1):
        var t: float = float(index) / float(CURVE_SAMPLES)
        bypass_straight.append(Vector2(40.0, lerpf(
            bypass_junction.position_xz.y,
            library_position.y,
            t
        )))
    _segments.append(_make_segment(
        "bypass_straight_to_library",
        "junction_bypass_mid",
        "junction_library",
        bypass_straight
    ))
    _segments.append(_make_segment(
        "library_to_bypass_straight",
        "junction_library",
        "junction_bypass_mid",
        _reverse_points(bypass_straight)
    ))

    _add_turn("junction_north_gate", "west_to_north", "north_to_bypass_mid", "left")
    _add_turn("junction_north_gate", "bypass_mid_to_north", "north_to_west", "straight")
    _add_turn(
        "junction_bypass_mid",
        "north_to_bypass_mid",
        "bypass_straight_to_library",
        "straight"
    )
    _add_turn("junction_bypass_mid", "north_to_bypass_mid", "bypass_left_to_library", "left")
    _add_turn("junction_bypass_mid", "north_to_bypass_mid", "bypass_right_to_library", "right")
    _add_turn("junction_bypass_mid", "library_to_bypass_left", "bypass_mid_to_north", "right")
    _add_turn("junction_bypass_mid", "library_to_bypass_right", "bypass_mid_to_north", "left")
    _add_turn(
        "junction_bypass_mid",
        "library_to_bypass_straight",
        "bypass_mid_to_north",
        "straight"
    )
    _add_turn("junction_library", "bypass_left_to_library", "east_to_south", "straight")
    _add_turn("junction_library", "bypass_right_to_library", "east_to_south", "straight")
    _add_turn("junction_library", "bypass_straight_to_library", "east_to_south", "straight")
    _add_turn("junction_library", "south_to_east", "library_to_bypass_left", "right")
    _add_turn("junction_library", "south_to_east", "library_to_bypass_right", "left")
    _add_turn(
        "junction_library",
        "south_to_east",
        "library_to_bypass_straight",
        "straight"
    )

    for node in _nodes:
        for turn in node.allowed_turns:
            if not turn.has("turn_direction"):
                turn["turn_direction"] = "straight"

    var report: Dictionary = CampusNetwork.validate_network_data(
        _nodes,
        _segments,
        _stations
    )
    _validation_errors = report.errors
    _validate_turn_metadata()

func get_validation_errors() -> PackedStringArray:
    return _validation_errors.duplicate()

func get_station_ids() -> PackedStringArray:
    var ids := PackedStringArray()
    for station in _stations:
        ids.append(str(station.id))
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
        ids.append(str(node.id))
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
        ids.append(str(segment.id))
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
    var report: Dictionary = CampusNetwork.validate_network_data(
        _nodes,
        _segments,
        _stations
    )
    return report.directed_reachability_valid

func _validate_turn_metadata() -> void:
    var segment_by_id: Dictionary = {}
    for segment in _segments:
        segment_by_id[segment.id] = segment
    for node in _nodes:
        for turn in node.allowed_turns:
            var direction: String = str(turn.get("turn_direction", ""))
            if not ALLOWED_TURN_DIRECTIONS.has(direction):
                _validation_errors.append(
                    "junction %s has an unsupported turn direction" % node.id
                )
                continue
            if not segment_by_id.has(turn.incoming_segment_id) \
                || not segment_by_id.has(turn.outgoing_segment_id):
                continue
            var incoming: Dictionary = segment_by_id[turn.incoming_segment_id]
            var outgoing: Dictionary = segment_by_id[turn.outgoing_segment_id]
            var incoming_points: Array = incoming.centerline_points
            var outgoing_points: Array = outgoing.centerline_points
            var incoming_tangent: Vector2 = (
                incoming_points[incoming_points.size() - 1]
                - incoming_points[incoming_points.size() - 2]
            ).normalized()
            var outgoing_tangent: Vector2 = (
                outgoing_points[1] - outgoing_points[0]
            ).normalized()
            if incoming_tangent.dot(outgoing_tangent) < cos(deg_to_rad(12.0)):
                _validation_errors.append(
                    "junction %s has a discontinuous or reversing directed transition"
                        % node.id
                )

func _add_turn(
    node_id: String,
    incoming_segment_id: String,
    outgoing_segment_id: String,
    direction: String
) -> void:
    for node in _nodes:
        if node.id == node_id:
            node.allowed_turns.append({
                "incoming_segment_id": incoming_segment_id,
                "outgoing_segment_id": outgoing_segment_id,
                "turn_direction": direction
            })
            return

func _north_to_bypass_points(
    start: Vector2,
    end: Vector2
) -> Array[Vector2]:
    var control_one := Vector2(30.0, 0.0)
    var control_two := Vector2(40.0, 5.0)
    var points: Array[Vector2] = []
    for index in range(CURVE_SAMPLES + 1):
        var t: float = float(index) / float(CURVE_SAMPLES)
        var inverse_t: float = 1.0 - t
        points.append(
            start * pow(inverse_t, 3.0)
            + control_one * (3.0 * pow(inverse_t, 2.0) * t)
            + control_two * (3.0 * inverse_t * pow(t, 2.0))
            + end * pow(t, 3.0)
        )
    return points

func _bypass_to_library_points(
    start: Vector2,
    end: Vector2,
    side: float
) -> Array[Vector2]:
    var points: Array[Vector2] = []
    for index in range(CURVE_SAMPLES + 1):
        var t: float = float(index) / float(CURVE_SAMPLES)
        var lateral_offset: float = side * 1.5 * pow(sin(PI * t), 2.0)
        points.append(Vector2(start.x + lateral_offset, lerpf(start.y, end.y, t)))
    return points

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

func _reverse_points(points: Array[Vector2]) -> Array[Vector2]:
    var reversed: Array[Vector2] = []
    for index in range(points.size() - 1, -1, -1):
        reversed.append(points[index])
    return reversed

func _duplicate_dictionary_array(source: Array[Dictionary]) -> Array[Dictionary]:
    var copies: Array[Dictionary] = []
    for entry in source:
        copies.append(
            _duplicate_segment(entry)
            if entry.has("centerline_points")
            else entry.duplicate(true)
        )
    return copies

func _duplicate_segment(segment: Dictionary) -> Dictionary:
    var copy: Dictionary = segment.duplicate(true)
    var points: Array[Vector2] = []
    for point in segment.centerline_points:
        points.append(point)
    copy.centerline_points = points
    return copy
