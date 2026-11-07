extends SceneTree

const CampusNetwork = preload("res://scripts/campus_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")

const POSITION_TOLERANCE_M: float = 0.001

class PlannerNetworkFixture extends RefCounted:
    var is_valid: bool = true
    var nodes: Dictionary = {}
    var segments: Dictionary = {}
    var stops: Dictionary = {}

    func get_station(station_id: String) -> Dictionary:
        return {"id": station_id} if stops.has(station_id) else {}

    func get_station_stop(station_id: String) -> Dictionary:
        if not stops.has(station_id):
            return {"valid": false}
        return stops[station_id].duplicate(true)

    func get_segment_ids() -> PackedStringArray:
        var ids := PackedStringArray()
        for segment_id in segments:
            ids.append(str(segment_id))
        ids.sort()
        return ids

    func get_segment(segment_id: String) -> Dictionary:
        return segments.get(segment_id, {}).duplicate(true)

    func get_node(node_id: String) -> Dictionary:
        return nodes.get(node_id, {}).duplicate(true)

    func get_outgoing_segments(node_id: String) -> Array[Dictionary]:
        var outgoing: Array[Dictionary] = []
        for segment in segments.values():
            if segment.start_node_id == node_id:
                outgoing.append(segment.duplicate(true))
        return outgoing

var failures: int = 0
var assertions: int = 0
var network: RefCounted
var planner: RefCounted

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    network = CampusNetwork.new()
    planner = CampusRoutePlanner.new(network)
    _test_network_and_station_pairs()
    _test_invalid_station_requests()
    _test_shortest_path_and_tie_breaking()
    _test_route_geometry_and_navigator_compatibility()
    _test_determinism_and_mutation_protection()
    _test_unavailable_and_geometrically_invalid_routes()

    if failures == 0:
        print("CampusRoutePlanner tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d CampusRoutePlanner assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _test_network_and_station_pairs() -> void:
    _expect(network.is_valid, "planner uses a validated campus network")
    var station_ids: PackedStringArray = network.get_station_ids()
    var successful_pairs: int = 0
    for origin_id in station_ids:
        for destination_id in station_ids:
            if origin_id == destination_id:
                continue
            var result: Dictionary = planner.plan_route(origin_id, destination_id)
            _expect(
                result.valid,
                "%s to %s has a legal geometrically feasible route (%s)"
                    % [origin_id, destination_id, result.get("failure_reason", "")]
            )
            if not result.valid:
                continue
            successful_pairs += 1
            var measured_path_length_m: float = 0.0
            for segment_id in result.segment_ids:
                measured_path_length_m += network.get_segment(segment_id).length_m
            print(
                "ROUTE: %s -> %s | %s | %.3f m | max curvature %.6f /m | min radius %.3f m"
                    % [
                        origin_id,
                        destination_id,
                        ", ".join(result.segment_ids),
                        result.total_length_m,
                        result.maximum_curvature_per_m,
                        result.minimum_turn_radius_m
                    ]
            )
            _expect(
                result.origin_station_id == origin_id
                    && result.destination_station_id == destination_id,
                "%s to %s result identifies its requested stations" % [origin_id, destination_id]
            )
            _expect(
                absf(result.total_length_m - measured_path_length_m) <= POSITION_TOLERANCE_M,
                "%s to %s route length sums authoritative measured segments"
                    % [origin_id, destination_id]
            )
            var transitions_are_legal: bool = true
            for segment_index in range(1, result.segment_ids.size()):
                var previous_segment: Dictionary = network.get_segment(
                    result.segment_ids[segment_index - 1]
                )
                var next_segment: Dictionary = network.get_segment(result.segment_ids[segment_index])
                var junction: Dictionary = network.get_node(previous_segment.end_node_id)
                var found_permitted_turn: bool = false
                for turn in junction.allowed_turns:
                    found_permitted_turn = found_permitted_turn || (
                        turn.incoming_segment_id == previous_segment.id
                            && turn.outgoing_segment_id == next_segment.id
                    )
                transitions_are_legal = transitions_are_legal && found_permitted_turn
            _expect(
                transitions_are_legal,
                "%s to %s uses only explicitly permitted directed transitions"
                    % [origin_id, destination_id]
            )
            _expect(
                not result.segment_ids.is_empty() && result.total_length_m > 0.0,
                "%s to %s returns directed segments and a positive route length"
                    % [origin_id, destination_id]
            )
            _expect(
                result.tracking_validated == false
                    && result.road_surface_containment_validated == false,
                "%s to %s reports planner geometry without claiming tracking or road containment"
                    % [origin_id, destination_id]
            )
    _expect(successful_pairs == 12, "planner serves all 12 ordered station pairs")

func _test_invalid_station_requests() -> void:
    var unknown_origin: Dictionary = planner.plan_route("not_a_station", "library")
    _expect(
        not unknown_origin.valid && unknown_origin.failure_reason == "unknown_origin_station",
        "unknown origin station is rejected explicitly"
    )
    var unknown_destination: Dictionary = planner.plan_route("library", "not_a_station")
    _expect(
        not unknown_destination.valid
            && unknown_destination.failure_reason == "unknown_destination_station",
        "unknown destination station is rejected explicitly"
    )
    var same_station: Dictionary = planner.plan_route("library", "library")
    _expect(
        not same_station.valid && same_station.failure_reason == "same_station_not_allowed",
        "same-station passenger journey is rejected"
    )

func _test_shortest_path_and_tie_breaking() -> void:
    var one_segment: Dictionary = planner.plan_route("north_gate", "library")
    _expect(
        one_segment.segment_ids == PackedStringArray(["north_to_east"]),
        "shortest adjacent-station route uses the direct permitted segment"
    )
    var tied_route: Dictionary = planner.plan_route("north_gate", "sports_hall")
    _expect(
        tied_route.segment_ids == PackedStringArray(["north_to_east", "east_to_south"]),
        "equal-length opposite-station routes use lexicographic segment-path tie-breaking"
    )
    var reverse_tie: Dictionary = planner.plan_route("library", "innovation_centre")
    _expect(
        reverse_tie.segment_ids == PackedStringArray(["east_to_north", "north_to_west"]),
        "tie-breaking is deterministic in the reverse direction"
    )
    var quarter_length: float = network.get_segment("north_to_east").length_m
    _expect(
        absf(one_segment.total_length_m - quarter_length) <= POSITION_TOLERANCE_M,
        "route cost is the measured segment length"
    )
    _expect(
        absf(tied_route.total_length_m - 2.0 * quarter_length) <= POSITION_TOLERANCE_M,
        "multi-segment route cost sums measured segment lengths"
    )

func _test_route_geometry_and_navigator_compatibility() -> void:
    var dynamics: VehicleDynamics = VehicleDynamics.new()
    var expected_minimum_radius_m: float = dynamics.wheelbase_m \
        / tan(dynamics.steering_limit_rad) * 1.25
    for origin_id in network.get_station_ids():
        for destination_id in network.get_station_ids():
            if origin_id == destination_id:
                continue
            var result: Dictionary = planner.plan_route(origin_id, destination_id)
            if not result.valid:
                continue
            var origin_stop: Dictionary = network.get_station_stop(origin_id)
            var destination_stop: Dictionary = network.get_station_stop(destination_id)
            var points: Array[Vector2] = result.centerline_points
            _expect(
                points[0].distance_to(origin_stop.position_xz) <= POSITION_TOLERANCE_M
                    && points[points.size() - 1].distance_to(destination_stop.position_xz)
                        <= POSITION_TOLERANCE_M,
                "%s to %s centerline endpoints match station stops" % [origin_id, destination_id]
            )
            var no_duplicate_points: bool = true
            for index in range(1, points.size()):
                if points[index - 1].distance_to(points[index]) <= POSITION_TOLERANCE_M:
                    no_duplicate_points = false
            _expect(no_duplicate_points, "%s to %s removes duplicate joining points" % [origin_id, destination_id])
            _expect(
                result.minimum_turn_radius_m + 0.0001 >= result.minimum_required_turn_radius_m,
                "%s to %s respects the safety-margined dynamics turn radius"
                    % [origin_id, destination_id]
            )
            _expect(
                absf(result.minimum_required_turn_radius_m - expected_minimum_radius_m) < 0.000001,
                "%s to %s radius threshold uses VehicleDynamics wheelbase and steering limits"
                    % [origin_id, destination_id]
            )
            _expect(
                result.maximum_curvature_per_m
                    <= 1.0 / result.minimum_required_turn_radius_m + 0.000001,
                "%s to %s maximum curvature is within the conservative dynamics limit"
                    % [origin_id, destination_id]
            )
            _expect(
                result.maximum_junction_heading_change_rad <= deg_to_rad(12.0) + 0.000001,
                "%s to %s has heading-continuous permitted junction transitions"
                    % [origin_id, destination_id]
            )
            _expect(
                result.navigator_maximum_deviation_m <= 0.25,
                "%s to %s RouteNavigator spline remains near the network centerline"
                    % [origin_id, destination_id]
            )

            var navigator = RouteNavigator.new()
            _expect(
                navigator.set_route(points),
                "%s to %s centerline is accepted by RouteNavigator" % [origin_id, destination_id]
            )
            var samples: PackedVector2Array = navigator.get_centerline_samples()
            _expect(
                samples[0].distance_to(points[0]) <= POSITION_TOLERANCE_M
                    && samples[samples.size() - 1].distance_to(points[points.size() - 1])
                        <= POSITION_TOLERANCE_M,
                "%s to %s RouteNavigator preserves route endpoints" % [origin_id, destination_id]
            )

func _test_determinism_and_mutation_protection() -> void:
    var first: Dictionary = planner.plan_route("north_gate", "sports_hall")
    for repetition in range(5):
        var repeated: Dictionary = planner.plan_route("north_gate", "sports_hall")
        _expect(
            repeated.segment_ids == first.segment_ids
                && is_equal_approx(repeated.total_length_m, first.total_length_m)
                && repeated.centerline_points == first.centerline_points,
            "repeated planner query %d returns identical route data" % repetition
        )
    first.segment_ids[0] = "mutated_segment"
    first.centerline_points[0] = Vector2(999.0, 999.0)
    var after_mutation: Dictionary = planner.plan_route("north_gate", "sports_hall")
    _expect(
        after_mutation.segment_ids[0] == "north_to_east",
        "mutating a route's segment ID result does not affect a later query"
    )
    _expect(
        after_mutation.centerline_points[0].distance_to(Vector2(0.0, 0.0))
            <= POSITION_TOLERANCE_M,
        "mutating a route's centerline result does not affect a later query (got %s)"
            % str(after_mutation.centerline_points[0])
    )

func _test_unavailable_and_geometrically_invalid_routes() -> void:
    var disconnected_fixture: PlannerNetworkFixture = _make_fixture(false)
    var disconnected_planner = CampusRoutePlanner.new(disconnected_fixture)
    var unavailable: Dictionary = disconnected_planner.plan_route("origin", "destination")
    _expect(
        not unavailable.valid && unavailable.failure_reason == "no_legal_route",
        "planner reports unavailable routes when no permitted transition exists"
    )

    var sharp_fixture: PlannerNetworkFixture = _make_fixture(true)
    var sharp_planner = CampusRoutePlanner.new(sharp_fixture)
    var sharp_result: Dictionary = sharp_planner.plan_route("origin", "destination")
    _expect(
        not sharp_result.valid && sharp_result.failure_reason == "geometric_infeasibility",
        "planner rejects a legal but sharply discontinuous junction route"
    )
    var details: PackedStringArray = sharp_result.details.details
    _expect(
        not details.is_empty() && details[0].contains("heading discontinuity"),
        "geometric failure identifies the unsupported junction heading change"
    )

    var bad_dynamics: VehicleDynamics = VehicleDynamics.new()
    bad_dynamics.steering_limit_rad = 0.0
    var invalid_dynamics_planner = CampusRoutePlanner.new(network, bad_dynamics)
    var invalid_dynamics: Dictionary = invalid_dynamics_planner.plan_route("north_gate", "library")
    _expect(
        not invalid_dynamics.valid
            && invalid_dynamics.failure_reason == "invalid_vehicle_dynamics_limits",
        "invalid steering constraints fail safely instead of assuming feasibility"
    )

func _make_fixture(allow_turn: bool) -> PlannerNetworkFixture:
    var fixture := PlannerNetworkFixture.new()
    fixture.nodes = {
        "origin_node": {
            "id": "origin_node",
            "position_xz": Vector2(0.0, 0.0),
            "junction_design_radius_m": 12.0,
            "allowed_turns": []
        },
        "junction": {
            "id": "junction",
            "position_xz": Vector2(0.0, 10.0),
            "junction_design_radius_m": 12.0,
            "allowed_turns": []
        },
        "destination_node": {
            "id": "destination_node",
            "position_xz": Vector2(10.0, 10.0),
            "junction_design_radius_m": 12.0,
            "allowed_turns": []
        }
    }
    fixture.segments = {
        "incoming": {
            "id": "incoming",
            "start_node_id": "origin_node",
            "end_node_id": "junction",
            "centerline_points": [Vector2(0.0, 0.0), Vector2(0.0, 10.0)],
            "length_m": 10.0
        },
        "outgoing": {
            "id": "outgoing",
            "start_node_id": "junction",
            "end_node_id": "destination_node",
            "centerline_points": [Vector2(0.0, 10.0), Vector2(10.0, 10.0)],
            "length_m": 10.0
        }
    }
    if allow_turn:
        fixture.nodes.junction.allowed_turns = [
            {"incoming_segment_id": "incoming", "outgoing_segment_id": "outgoing"}
        ]
    fixture.stops = {
        "origin": {
            "valid": true,
            "station_id": "origin",
            "node_id": "origin_node",
            "position_xz": Vector2(0.0, 0.0)
        },
        "destination": {
            "valid": true,
            "station_id": "destination",
            "node_id": "destination_node",
            "position_xz": Vector2(10.0, 10.0)
        }
    }
    return fixture
