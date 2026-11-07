extends SceneTree

const CampusNetwork = preload("res://scripts/campus_network.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")

const POSITION_TOLERANCE_M: float = 0.001

var failures: int = 0
var assertions: int = 0
var network: RefCounted

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    network = CampusNetwork.new()
    _test_default_network_validity()
    _test_station_data_and_stops()
    _test_unique_nodes_and_segments()
    _test_segment_geometry()
    _test_turn_geometry_metadata()
    _test_directed_connectivity_and_return_service()
    _test_invalid_data_rejection()
    _test_deterministic_queries()
    _test_query_mutation_protection()
    _test_route_navigator_coordinate_compatibility()

    if failures == 0:
        print("CampusNetwork tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d CampusNetwork assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _test_default_network_validity() -> void:
    _expect(network.is_valid, "fictional campus network validates")
    _expect(network.get_validation_errors().is_empty(), "valid default network has no validation errors")
    _expect(
        network.are_stations_mutually_reachable(),
        "all default stations are mutually reachable through directed roads"
    )

func _test_station_data_and_stops() -> void:
    var expected_ids := PackedStringArray([
        "north_gate",
        "library",
        "sports_hall",
        "innovation_centre"
    ])
    var ids: PackedStringArray = network.get_station_ids()
    _expect(ids == expected_ids, "network exposes the four stable fictional station IDs in deterministic order")

    var expected_names := {
        "north_gate": "North Gate Stop",
        "library": "Library Stop",
        "sports_hall": "Sports Hall Stop",
        "innovation_centre": "Innovation Centre Stop"
    }
    var station_names := PackedStringArray()
    for station_id in expected_ids:
        var station: Dictionary = network.get_station(station_id)
        _expect(not station.is_empty(), "%s station metadata exists" % station_id)
        station_names.append(station.display_name)
        _expect(
            station.display_name == expected_names[station_id],
            "%s has its expected display name" % station_id
        )
        _expect(station.stop_reference is Dictionary, "%s has a network stop reference" % station_id)
        var stop: Dictionary = network.get_station_stop(station_id)
        _expect(stop.valid, "%s resolves to a valid stopping location" % station_id)
        var node: Dictionary = network.get_node(stop.node_id)
        _expect(not node.is_empty(), "%s stop references an existing junction" % station_id)
        _expect(
            stop.position_xz.distance_to(node.position_xz) <= POSITION_TOLERANCE_M,
            "%s stop position aligns with its referenced junction" % station_id
        )
        _expect(
            stop.position_xz.is_finite(),
            "%s stop location is finite in world X/Z coordinates" % station_id
        )
        _expect(
            not network.get_outgoing_segments(stop.node_id).is_empty(),
            "%s stop is connected to at least one directed road segment" % station_id
        )

    _expect(_strings_are_unique(station_names), "all station display names are unique")
    _expect(
        not network.get_station_stop("unknown_station").valid,
        "unknown station produces an explicitly invalid stop result"
    )

func _test_unique_nodes_and_segments() -> void:
    var node_ids: PackedStringArray = network.get_node_ids()
    var segment_ids: PackedStringArray = network.get_segment_ids()
    _expect(node_ids.size() == 4, "network contains four junction nodes")
    _expect(segment_ids.size() == 8, "network contains explicit forward and reverse road edges")
    _expect(_strings_are_unique(node_ids), "junction node IDs are unique")
    _expect(_strings_are_unique(segment_ids), "directed road segment IDs are unique")
    _expect(
        _strings_are_unique(network.get_station_ids()),
        "station IDs are unique"
    )

func _test_segment_geometry() -> void:
    for node in network.get_nodes():
        _expect(
            node.position_xz is Vector2 && node.position_xz.is_finite(),
            "%s has a finite X/Z position" % node.id
        )

    for segment in network.get_segments():
        var start_node: Dictionary = network.get_node(segment.start_node_id)
        var end_node: Dictionary = network.get_node(segment.end_node_id)
        var points: Array[Vector2] = segment.centerline_points
        var measured_length_m: float = 0.0
        var valid_points: bool = points.size() >= 2
        for index in range(points.size()):
            valid_points = valid_points && points[index].is_finite()
            if index > 0:
                var edge_length_m: float = points[index - 1].distance_to(points[index])
                valid_points = valid_points && edge_length_m > 0.0
                measured_length_m += edge_length_m
        _expect(valid_points, "%s has finite non-degenerate ordered centerline points" % segment.id)
        _expect(
            points[0].distance_to(start_node.position_xz) <= POSITION_TOLERANCE_M,
            "%s centerline starts at its declared node" % segment.id
        )
        _expect(
            points[points.size() - 1].distance_to(end_node.position_xz) <= POSITION_TOLERANCE_M,
            "%s centerline ends at its declared node" % segment.id
        )
        _expect(
            segment.length_m > 0.0 && absf(segment.length_m - measured_length_m) <= POSITION_TOLERANCE_M,
            "%s has a positive measured polyline length" % segment.id
        )
        _expect(
            is_finite(segment.design_turn_radius_m) && segment.design_turn_radius_m >= 40.0,
            "%s carries feasible design-radius metadata" % segment.id
        )

func _test_turn_geometry_metadata() -> void:
    var maximum_tangent_change_rad: float = 0.0
    for node in network.get_nodes():
        _expect(
            node.junction_design_radius_m > VehicleDynamics.new().wheelbase_m
                / tan(VehicleDynamics.new().steering_limit_rad),
            "%s design radius exceeds the dynamics minimum turn radius" % node.id
        )
        for turn in node.allowed_turns:
            var incoming: Dictionary = network.get_segment(turn.incoming_segment_id)
            var outgoing: Dictionary = network.get_segment(turn.outgoing_segment_id)
            _expect(
                incoming.end_node_id == node.id && outgoing.start_node_id == node.id,
                "%s permitted movement connects directed segments at the junction" % node.id
            )
            var incoming_points: Array[Vector2] = incoming.centerline_points
            var outgoing_points: Array[Vector2] = outgoing.centerline_points
            var in_tangent: Vector2 = (
                incoming_points[incoming_points.size() - 1]
                - incoming_points[incoming_points.size() - 2]
            ).normalized()
            var out_tangent: Vector2 = (
                outgoing_points[1] - outgoing_points[0]
            ).normalized()
            var tangent_change_rad: float = acos(clampf(in_tangent.dot(out_tangent), -1.0, 1.0))
            maximum_tangent_change_rad = maxf(maximum_tangent_change_rad, tangent_change_rad)
    _expect(
        maximum_tangent_change_rad <= deg_to_rad(12.0),
        "permitted ring movements have near-continuous sampled tangents"
    )

func _test_directed_connectivity_and_return_service() -> void:
    var expected_outgoing := {
        "junction_north_gate": ["north_to_east", "north_to_west"],
        "junction_library": ["east_to_south", "east_to_north"],
        "junction_sports_hall": ["south_to_west", "south_to_east"],
        "junction_innovation_centre": ["west_to_north", "west_to_south"]
    }
    for node_id in expected_outgoing:
        var outgoing: Array[Dictionary] = network.get_outgoing_segments(node_id)
        var actual_ids := PackedStringArray()
        for segment in outgoing:
            actual_ids.append(segment.id)
            _expect(
                segment.start_node_id == node_id,
                "%s outgoing query returns only segments directed from that node" % node_id
            )
        _expect(
            actual_ids == PackedStringArray(expected_outgoing[node_id]),
            "%s outgoing directed edges have deterministic order" % node_id
        )

    for origin_id in network.get_station_ids():
        for destination_id in network.get_station_ids():
            _expect(
                _can_reach_station(origin_id, destination_id),
                "%s can reach %s using directed road segments" % [origin_id, destination_id]
            )
        _expect(
            _can_reach_station(origin_id, "north_gate"),
            "%s can return to the north-gate service station" % origin_id
        )

func _test_invalid_data_rejection() -> void:
    var nodes: Array[Dictionary] = network.get_nodes()
    var segments: Array[Dictionary] = network.get_segments()
    var stations: Array[Dictionary] = network.get_stations()

    var duplicate_nodes: Array[Dictionary] = nodes.duplicate(true)
    duplicate_nodes.append(nodes[0].duplicate(true))
    _expect(
        not _validate(duplicate_nodes, segments, stations).valid,
        "validator rejects duplicate node IDs"
    )

    var duplicate_segments: Array[Dictionary] = segments.duplicate(true)
    duplicate_segments.append(segments[0].duplicate(true))
    _expect(
        not _validate(nodes, duplicate_segments, stations).valid,
        "validator rejects duplicate directed segment IDs"
    )

    var duplicate_stations: Array[Dictionary] = stations.duplicate(true)
    duplicate_stations.append(stations[0].duplicate(true))
    _expect(
        not _validate(nodes, segments, duplicate_stations).valid,
        "validator rejects duplicate station IDs"
    )

    var missing_reference_segments: Array[Dictionary] = segments.duplicate(true)
    missing_reference_segments[0].start_node_id = "missing_node"
    _expect(
        not _validate(nodes, missing_reference_segments, stations).valid,
        "validator rejects missing segment node references"
    )

    var non_finite_nodes: Array[Dictionary] = nodes.duplicate(true)
    non_finite_nodes[0].position_xz = Vector2(NAN, 0.0)
    _expect(
        not _validate(non_finite_nodes, segments, stations).valid,
        "validator rejects non-finite junction positions"
    )

    var non_finite_points: Array[Dictionary] = segments.duplicate(true)
    var broken_points: Array[Vector2] = non_finite_points[0].centerline_points
    broken_points[1] = Vector2(0.0, INF)
    non_finite_points[0].centerline_points = broken_points
    _expect(
        not _validate(nodes, non_finite_points, stations).valid,
        "validator rejects non-finite centerline points"
    )

    var degenerate_segments: Array[Dictionary] = segments.duplicate(true)
    var degenerate_points: Array[Vector2] = degenerate_segments[0].centerline_points
    degenerate_points[1] = degenerate_points[0]
    degenerate_segments[0].centerline_points = degenerate_points
    _expect(
        not _validate(nodes, degenerate_segments, stations).valid,
        "validator rejects degenerate centerline edges"
    )

    var mismatched_segments: Array[Dictionary] = segments.duplicate(true)
    var mismatched_points: Array[Vector2] = mismatched_segments[0].centerline_points
    mismatched_points[0] += Vector2(1.0, 0.0)
    mismatched_segments[0].centerline_points = mismatched_points
    _expect(
        not _validate(nodes, mismatched_segments, stations).valid,
        "validator rejects segment endpoints that mismatch junction positions"
    )

    var invalid_station_reference: Array[Dictionary] = stations.duplicate(true)
    invalid_station_reference[0].stop_reference.node_id = "missing_node"
    _expect(
        not _validate(nodes, segments, invalid_station_reference).valid,
        "validator rejects missing station stop references"
    )

    var unreachable_segments: Array[Dictionary] = segments.duplicate(true)
    for index in range(unreachable_segments.size() - 1, -1, -1):
        if unreachable_segments[index].start_node_id == "junction_north_gate" \
            || unreachable_segments[index].end_node_id == "junction_north_gate":
            unreachable_segments.remove_at(index)
    var unreachable_report: Dictionary = _validate(nodes, unreachable_segments, stations)
    _expect(not unreachable_report.valid, "validator rejects a directed-disconnected station network")
    _expect(
        not unreachable_report.directed_reachability_valid,
        "disconnected station network reports directed reachability failure"
    )

    var prohibited_turn_nodes: Array[Dictionary] = nodes.duplicate(true)
    for node in prohibited_turn_nodes:
        node.allowed_turns.clear()
    var prohibited_turn_report: Dictionary = _validate(
        prohibited_turn_nodes,
        segments,
        stations
    )
    _expect(
        not prohibited_turn_report.directed_reachability_valid,
        "station reachability respects explicit allowed-turn connections"
    )

func _test_deterministic_queries() -> void:
    var first_ids: PackedStringArray = network.get_station_ids()
    var second_ids: PackedStringArray = network.get_station_ids()
    _expect(first_ids == second_ids, "station listing is deterministic")
    _expect(
        network.get_outgoing_segments("junction_north_gate")
            == network.get_outgoing_segments("junction_north_gate"),
        "outgoing-segment queries are deterministic"
    )
    for station_id in first_ids:
        _expect(
            network.get_station_stop(station_id) == network.get_station_stop(station_id),
            "%s stop lookup is deterministic" % station_id
        )

func _test_query_mutation_protection() -> void:
    var stations: Array[Dictionary] = network.get_stations()
    stations[0].display_name = "Mutated station"
    stations[0].stop_reference.node_id = "mutated_node"
    _expect(
        network.get_station("north_gate").display_name == "North Gate Stop",
        "station query copies protect stored display metadata"
    )
    _expect(
        network.get_station_stop("north_gate").node_id == "junction_north_gate",
        "nested stop-reference copies protect stored station references"
    )

    var segments: Array[Dictionary] = network.get_segments()
    var original_start: Vector2 = segments[0].centerline_points[0]
    var copied_points: Array = segments[0].centerline_points
    copied_points[0] = Vector2(999.0, 999.0)
    segments[0].centerline_points = copied_points
    var preserved_start: Vector2 = network.get_segment(segments[0].id).centerline_points[0]
    _expect(
        preserved_start.distance_to(original_start) <= POSITION_TOLERANCE_M
            && preserved_start.distance_to(network.get_node(segments[0].start_node_id).position_xz)
                <= POSITION_TOLERANCE_M,
        "segment query copies protect stored centerline geometry"
    )

    var outgoing: Array[Dictionary] = network.get_outgoing_segments("junction_north_gate")
    outgoing[0].id = "mutated_segment"
    _expect(
        network.get_segment_ids().has("north_to_east"),
        "outgoing-segment query copies protect stored edge IDs"
    )

func _test_route_navigator_coordinate_compatibility() -> void:
    var navigator = RouteNavigator.new()
    for segment in network.get_segments():
        var points: Array[Vector2] = segment.centerline_points
        _expect(
            navigator.set_route(points),
            "%s X/Z centerline is accepted by RouteNavigator" % segment.id
        )
        var samples: PackedVector2Array = navigator.get_centerline_samples()
        _expect(
            samples[0].distance_to(points[0]) <= POSITION_TOLERANCE_M
                && samples[samples.size() - 1].distance_to(points[points.size() - 1]) <= POSITION_TOLERANCE_M,
            "%s preserves endpoint coordinates in RouteNavigator" % segment.id
        )
        _expect(
            absf(navigator.total_length_m - segment.length_m) < 0.5,
            "%s length is compatible with navigator X/Z arc-length measurement" % segment.id
        )

func _validate(
    nodes: Array,
    segments: Array,
    stations: Array
) -> Dictionary:
    return CampusNetwork.validate_network_data(nodes, segments, stations)

func _can_reach_station(origin_station_id: String, destination_station_id: String) -> bool:
    var origin: Dictionary = network.get_station_stop(origin_station_id)
    var destination: Dictionary = network.get_station_stop(destination_station_id)
    if not origin.valid || not destination.valid:
        return false
    var visited: Dictionary = {origin.node_id: true}
    var pending: Array[String] = [origin.node_id]
    var current_index: int = 0
    while current_index < pending.size():
        var node_id: String = pending[current_index]
        current_index += 1
        if node_id == destination.node_id:
            return true
        for segment in network.get_outgoing_segments(node_id):
            if not visited.has(segment.end_node_id):
                visited[segment.end_node_id] = true
                pending.append(segment.end_node_id)
    return false

func _strings_are_unique(values: PackedStringArray) -> bool:
    var seen: Dictionary = {}
    for value in values:
        if seen.has(value):
            return false
        seen[value] = true
    return true
