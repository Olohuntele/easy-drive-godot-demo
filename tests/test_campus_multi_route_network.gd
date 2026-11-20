extends SceneTree

const CampusNetwork = preload("res://scripts/campus_network.gd")
const CampusMultiRouteNetwork = preload("res://scripts/campus_multi_route_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")

const MAX_TRANSITION_HEADING_CHANGE_RAD: float = deg_to_rad(12.0)
const GEOMETRY_EPSILON: float = 0.000001

class SegmentFilteredNetwork extends RefCounted:
    var source: RefCounted
    var unavailable_segments: Dictionary = {}

    func _init(network: RefCounted, unavailable: PackedStringArray) -> void:
        source = network
        for segment_id in unavailable:
            unavailable_segments[segment_id] = true

    var is_valid: bool:
        get:
            return source.is_valid

    func get_validation_errors() -> PackedStringArray:
        return source.get_validation_errors()

    func get_station_ids() -> PackedStringArray:
        return source.get_station_ids()

    func get_station(station_id: String) -> Dictionary:
        return source.get_station(station_id)

    func get_station_stop(station_id: String) -> Dictionary:
        return source.get_station_stop(station_id)

    func get_segment_ids() -> PackedStringArray:
        var ids := PackedStringArray()
        for segment_id in source.get_segment_ids():
            if not unavailable_segments.has(segment_id):
                ids.append(segment_id)
        return ids

    func get_segments() -> Array[Dictionary]:
        var segments: Array[Dictionary] = []
        for segment in source.get_segments():
            if not unavailable_segments.has(segment.id):
                segments.append(segment)
        return segments

    func get_segment(segment_id: String) -> Dictionary:
        if unavailable_segments.has(segment_id):
            return {}
        return source.get_segment(segment_id)

    func get_node(node_id: String) -> Dictionary:
        return source.get_node(node_id)

    func get_outgoing_segments(node_id: String) -> Array[Dictionary]:
        var outgoing: Array[Dictionary] = []
        for segment in source.get_outgoing_segments(node_id):
            if not unavailable_segments.has(segment.id):
                outgoing.append(segment)
        return outgoing

var failures: int = 0
var assertions: int = 0
var network: RefCounted
var planner: RefCounted

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    network = CampusMultiRouteNetwork.new()
    planner = CampusRoutePlanner.new(network)
    _test_network_and_stable_stations()
    _test_directed_segments_and_junction_rules()
    _test_all_station_pairs_and_determinism()
    _test_alternative_paths()
    _test_pose_aware_planning_and_navigator()
    _test_original_ring_remains_unchanged()

    if failures == 0:
        print("CampusMultiRouteNetwork tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d CampusMultiRouteNetwork assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _test_network_and_stable_stations() -> void:
    _expect(network.is_valid, "multi-route scenario passes CampusNetwork structural validation")
    _expect(
        network.get_validation_errors().is_empty()
            && network.are_stations_mutually_reachable(),
        "scenario has no validation errors and all stations are mutually reachable"
    )
    _expect(
        network.get_node_ids().size() == 5
            && network.get_segment_ids().size() == 16,
        "scenario adds a connected intersection and eight directed bypass roads to the four-node ring"
    )
    _expect(
        network.get_station_ids() == PackedStringArray([
            "north_gate",
            "library",
            "sports_hall",
            "innovation_centre"
        ]),
        "scenario preserves all four station identities and deterministic ordering"
    )
    _expect(
        network.get_node("junction_bypass_mid").position_xz == Vector2(40.0, 20.0),
        "bypass intersection is placed between North Gate and Library"
    )

func _test_directed_segments_and_junction_rules() -> void:
    var unique_segment_ids: Dictionary = {}
    var all_directions_explicit: bool = true
    for segment in network.get_segments():
        all_directions_explicit = all_directions_explicit \
            && segment.start_node_id != segment.end_node_id \
            && segment.centerline_points.size() >= 2 \
            && segment.centerline_points[0].distance_to(
                network.get_node(segment.start_node_id).position_xz
            ) <= 0.001 \
            && segment.centerline_points[segment.centerline_points.size() - 1].distance_to(
                network.get_node(segment.end_node_id).position_xz
            ) <= 0.001
        unique_segment_ids[segment.id] = true
    _expect(
        all_directions_explicit && unique_segment_ids.size() == network.get_segment_ids().size(),
        "every directed road has unique identity and joins its declared endpoint nodes"
    )

    var left_turn_count: int = 0
    var right_turn_count: int = 0
    var transitions_valid: bool = true
    for node in network.get_nodes():
        for turn in node.allowed_turns:
            var incoming: Dictionary = network.get_segment(turn.incoming_segment_id)
            var outgoing: Dictionary = network.get_segment(turn.outgoing_segment_id)
            if incoming.is_empty() || outgoing.is_empty() \
                || incoming.end_node_id != node.id || outgoing.start_node_id != node.id:
                transitions_valid = false
                continue
            if turn.turn_direction == "left":
                left_turn_count += 1
            elif turn.turn_direction == "right":
                right_turn_count += 1
            elif turn.turn_direction != "straight":
                transitions_valid = false
            var in_points: Array = incoming.centerline_points
            var out_points: Array = outgoing.centerline_points
            var in_tangent: Vector2 = (
                in_points[in_points.size() - 1] - in_points[in_points.size() - 2]
            ).normalized()
            var out_tangent: Vector2 = (out_points[1] - out_points[0]).normalized()
            transitions_valid = transitions_valid \
                && acos(clampf(in_tangent.dot(out_tangent), -1.0, 1.0)) \
                    <= MAX_TRANSITION_HEADING_CHANGE_RAD + GEOMETRY_EPSILON
    _expect(
        transitions_valid,
        "all explicit junction transitions connect directed roads with continuous, non-reversing tangents"
    )
    _expect(
        left_turn_count >= 2 && right_turn_count >= 2,
        "junction rules include explicit legal left and right route choices"
    )

    var required_curves_are_present: bool = true
    for segment_id in [
        "north_to_bypass_mid",
        "bypass_left_to_library",
        "bypass_right_to_library"
    ]:
        var points: Array = network.get_segment(segment_id).centerline_points
        var is_curved: bool = false
        var initial_tangent: Vector2 = (points[1] - points[0]).normalized()
        for index in range(2, points.size()):
            var tangent: Vector2 = (points[index] - points[index - 1]).normalized()
            if absf(initial_tangent.cross(tangent)) > 0.001:
                is_curved = true
                break
        required_curves_are_present = required_curves_are_present && is_curved
    var straight_points: Array = network.get_segment(
        "bypass_straight_to_library"
    ).centerline_points
    var straight_segment_is_straight: bool = true
    var straight_tangent: Vector2 = (straight_points[1] - straight_points[0]).normalized()
    for index in range(2, straight_points.size()):
        straight_segment_is_straight = straight_segment_is_straight \
            && (straight_points[index] - straight_points[index - 1]).normalized() \
                .distance_to(straight_tangent) <= GEOMETRY_EPSILON
    _expect(
        required_curves_are_present && straight_segment_is_straight,
        "added bypass centerlines contain smooth curves as well as a straight road segment"
    )

func _test_all_station_pairs_and_determinism() -> void:
    var station_ids: PackedStringArray = network.get_station_ids()
    var successful_pairs: int = 0
    for origin_id in station_ids:
        for destination_id in station_ids:
            if origin_id == destination_id:
                continue
            var first: Dictionary = planner.plan_route(origin_id, destination_id)
            var second: Dictionary = planner.plan_route(origin_id, destination_id)
            _expect(
                first.get("valid", false),
                "%s to %s has a feasible legal route (%s)"
                    % [origin_id, destination_id, first.get("failure_reason", "")]
            )
            if not first.get("valid", false):
                continue
            successful_pairs += 1
            _expect(
                first.segment_ids == second.segment_ids
                    && is_equal_approx(first.total_length_m, second.total_length_m),
                "%s to %s route and cost are deterministic" % [origin_id, destination_id]
            )
            _expect(
                first.origin_station_id == origin_id
                    && first.destination_station_id == destination_id
                    && first.navigator_compatible
                    && first.tracking_validated == false,
                "%s to %s returns station metadata and planner-only geometry status"
                    % [origin_id, destination_id]
            )
            var navigator = RouteNavigator.new()
            _expect(
                navigator.set_route(first.centerline_points),
                "%s to %s centerline installs in RouteNavigator" % [origin_id, destination_id]
            )
            _expect(
                first.maximum_curvature_per_m
                    <= 1.0 / first.minimum_required_turn_radius_m + GEOMETRY_EPSILON,
                "%s to %s stays within the planner's existing curvature limit"
                    % [origin_id, destination_id]
            )
    _expect(successful_pairs == 12, "planner reaches all 12 ordered station pairs")

func _test_alternative_paths() -> void:
    var default_route: Dictionary = planner.plan_route("north_gate", "library")
    _expect(
        default_route.valid
            && default_route.segment_ids == PackedStringArray(["north_to_east"]),
        "shortest route retains the original direct North Gate to Library ring road"
    )
    var closed_direct_route_network := SegmentFilteredNetwork.new(
        network,
        PackedStringArray(["north_to_east", "bypass_straight_to_library"])
    )
    var alternate_planner := CampusRoutePlanner.new(closed_direct_route_network)
    var alternate_route: Dictionary = alternate_planner.plan_route("north_gate", "library")
    _expect(
        alternate_route.valid
            && alternate_route.segment_ids[0] == "north_to_bypass_mid"
            && alternate_route.segment_ids.size() == 2,
        "a legal bypass route remains plannable when the direct road is unavailable"
    )
    _expect(
        alternate_route.get("valid", false)
            && alternate_route.get("segment_ids", PackedStringArray()).size() == 2
            && alternate_route.get("segment_ids", PackedStringArray())[1] in [
                "bypass_left_to_library",
                "bypass_right_to_library"
            ],
        "alternative path selects one of the explicitly permitted left/right branches"
    )
    var right_branch_network := SegmentFilteredNetwork.new(
        network,
        PackedStringArray([
            "north_to_east",
            "bypass_straight_to_library",
            "bypass_left_to_library"
        ])
    )
    var right_branch_route: Dictionary = CampusRoutePlanner.new(right_branch_network).plan_route(
        "north_gate",
        "library"
    )
    _expect(
        right_branch_route.valid
            && right_branch_route.segment_ids == PackedStringArray([
                "north_to_bypass_mid",
                "bypass_right_to_library"
            ]),
        "planner reuses the explicit legal right-turn branch when other paths are unavailable"
    )
    var alternate_station_pair_network := SegmentFilteredNetwork.new(
        network,
        PackedStringArray(["east_to_north", "east_to_south"])
    )
    var alternate_station_pair := CampusRoutePlanner.new(
        alternate_station_pair_network
    ).plan_route(
        "library",
        "innovation_centre"
    )
    _expect(
        alternate_station_pair.get("valid", false)
            && alternate_station_pair.get("segment_ids", PackedStringArray()) == PackedStringArray([
                "library_to_bypass_straight",
                "bypass_mid_to_north",
                "north_to_west"
            ]),
        "a second station pair has a legal bypass route when its ring road is unavailable"
    )

func _test_pose_aware_planning_and_navigator() -> void:
    var start_stop: Dictionary = network.get_station_stop("north_gate")
    var start_segment: Dictionary = network.get_segment("north_to_bypass_mid")
    var start_points: Array = start_segment.centerline_points
    var first_tangent: Vector2 = (start_points[1] - start_points[0]).normalized()
    var heading: float = atan2(first_tangent.x, first_tangent.y)
    var route: Dictionary = planner.plan_route_from_pose(
        "north_gate",
        "library",
        start_stop.position_xz,
        heading
    )
    _expect(
        route.valid && route.get("heading_aware", false)
            && route.origin_position_error_m <= 1.0
            && route.origin_heading_error_rad <= MAX_TRANSITION_HEADING_CHANGE_RAD,
        "existing pose-aware planner accepts a legal bypass-aligned station pose"
    )
    var navigator = RouteNavigator.new()
    _expect(
        route.valid && navigator.set_route(route.centerline_points),
        "pose-aware alternative route remains compatible with RouteNavigator"
    )
    var incompatible_heading: Dictionary = planner.plan_route_from_pose(
        "north_gate",
        "library",
        start_stop.position_xz,
        heading + PI / 2.0
    )
    _expect(
        not incompatible_heading.valid
            && incompatible_heading.failure_reason == "no_heading_compatible_route",
        "pose-aware planning retains the existing heading constraint"
    )

func _test_original_ring_remains_unchanged() -> void:
    var ring := CampusNetwork.new()
    _expect(
        ring.get_node_ids().size() == 4 && ring.get_segment_ids().size() == 8,
        "original CampusNetwork remains the unchanged four-node, eight-directional-edge ring"
    )
    var ring_segments_unchanged: bool = true
    for segment_id in ring.get_segment_ids():
        var original_segment: Dictionary = ring.get_segment(segment_id)
        var scenario_segment: Dictionary = network.get_segment(segment_id)
        ring_segments_unchanged = ring_segments_unchanged \
            && original_segment.start_node_id == scenario_segment.start_node_id \
            && original_segment.end_node_id == scenario_segment.end_node_id \
            && original_segment.centerline_points == scenario_segment.centerline_points
    _expect(
        ring_segments_unchanged,
        "multi-route scenario preserves every original ring segment geometry and direction"
    )
