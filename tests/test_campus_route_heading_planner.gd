extends SceneTree

const CampusNetwork = preload("res://scripts/campus_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const PassengerJourney = preload("res://scripts/passenger_journey.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const RouteFollowingController = preload("res://scripts/route_following_controller.gd")
const ShuttleDispatcher = preload("res://scripts/shuttle_dispatcher.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")

const HEADING_TOLERANCE_RAD: float = deg_to_rad(12.0)
const POSITION_TOLERANCE_M: float = 1.0
const GEOMETRY_TOLERANCE: float = 0.000001
const MAX_SIMULATION_TICKS: int = 12000

var failures: int = 0
var assertions: int = 0
var network: RefCounted
var planner: RefCounted

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    network = CampusNetwork.new()
    planner = CampusRoutePlanner.new(network)
    _test_existing_shortest_path_api()
    _test_pose_validation_and_incompatible_heading()
    _test_all_station_arrival_headings()
    _test_heading_compatible_longer_return_routes()
    _test_determinism_and_mutation_protection()
    _test_journey_route_override_handoff()
    _test_closed_loop_heading_aware_returns()
    if failures == 0:
        print("CampusRouteHeadingPlanner tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d CampusRouteHeadingPlanner assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _heading_for_segment_start(segment_id: String) -> float:
    var segment: Dictionary = network.get_segment(segment_id)
    var points: Array = segment.centerline_points
    var tangent: Vector2 = (points[1] - points[0]).normalized()
    return atan2(tangent.x, tangent.y)

func _heading_for_segment_end(segment_id: String) -> float:
    var segment: Dictionary = network.get_segment(segment_id)
    var points: Array = segment.centerline_points
    var tangent: Vector2 = (
        points[points.size() - 1] - points[points.size() - 2]
    ).normalized()
    return atan2(tangent.x, tangent.y)

func _assert_route_geometry(route: Dictionary, context: String) -> void:
    _expect(
        route.get("navigator_compatible", false)
            && route.get("tracking_validated", true) == false,
        "%s preserves RouteNavigator compatibility without claiming closed-loop tracking"
            % context
    )
    var navigator = RouteNavigator.new()
    var points: Array[Vector2] = route.centerline_points
    _expect(
        navigator.set_route(points),
        "%s centerline installs in RouteNavigator" % context
    )
    var samples: PackedVector2Array = navigator.get_centerline_samples()
    _expect(
        samples.size() >= 2
            && samples[0].distance_to(points[0]) <= 0.001
            && samples[samples.size() - 1].distance_to(points[points.size() - 1]) <= 0.001,
        "%s RouteNavigator spline preserves both endpoints" % context
    )
    _expect(
        float(route.maximum_curvature_per_m)
            <= 1.0 / float(route.minimum_required_turn_radius_m) + GEOMETRY_TOLERANCE,
        "%s route passes the existing curvature limit" % context
    )

func _test_existing_shortest_path_api() -> void:
    var direct: Dictionary = planner.plan_route("north_gate", "library")
    _expect(
        direct.valid && direct.segment_ids == PackedStringArray(["north_to_east"]),
        "existing shortest-path API retains its direct route behavior"
    )
    var opposite_station: Dictionary = planner.plan_route("north_gate", "sports_hall")
    _expect(
        opposite_station.valid
            && opposite_station.segment_ids == PackedStringArray([
                "north_to_east",
                "east_to_south"
            ]),
        "existing shortest-path API retains deterministic shortest-path tie behavior"
    )
    _expect(
        not direct.has("heading_aware"),
        "legacy shortest-path results remain pose-unconstrained"
    )

func _test_pose_validation_and_incompatible_heading() -> void:
    var north_stop: Vector2 = network.get_station_stop("north_gate").position_xz
    var compatible_heading: float = _heading_for_segment_start("north_to_east")
    var invalid_pose: Dictionary = planner.plan_route_from_pose(
        "north_gate",
        "library",
        Vector2(INF, 0.0),
        compatible_heading
    )
    _expect(
        not invalid_pose.valid && invalid_pose.failure_reason == "invalid_origin_pose",
        "non-finite origin position is rejected"
    )
    var incompatible_position: Dictionary = planner.plan_route_from_pose(
        "north_gate",
        "library",
        north_stop + Vector2(1.01, 0.0),
        compatible_heading
    )
    _expect(
        not incompatible_position.valid
            && incompatible_position.failure_reason == "origin_pose_incompatible",
        "origin position outside handoff tolerance is rejected"
    )
    var incompatible_heading: Dictionary = planner.plan_route_from_pose(
        "north_gate",
        "library",
        north_stop,
        compatible_heading + PI / 2.0
    )
    _expect(
        not incompatible_heading.valid
            && incompatible_heading.failure_reason == "no_heading_compatible_route",
        "no legal route is reported when no outgoing tangent matches the actual heading"
    )

    var boundary_heading: Dictionary = planner.plan_route_from_pose(
        "north_gate",
        "library",
        north_stop,
        compatible_heading + deg_to_rad(11.9)
    )
    _expect(
        boundary_heading.valid
            && boundary_heading.origin_heading_error_rad
                <= PassengerJourney.HEADING_TOLERANCE_RAD,
        "heading within the existing 12-degree handoff tolerance is accepted"
    )
    _expect(
        is_equal_approx(
            PassengerJourney.HEADING_TOLERANCE_RAD,
            HEADING_TOLERANCE_RAD
        ),
        "planner compatibility uses the journey's unchanged 12-degree tolerance"
    )

func _test_all_station_arrival_headings() -> void:
    var requests_tested: int = 0
    for station_id in network.get_station_ids():
        var stop: Dictionary = network.get_station_stop(station_id)
        var incoming_segments: Array[Dictionary] = []
        for segment in network.get_segments():
            if segment.end_node_id == stop.node_id:
                incoming_segments.append(segment)
        _expect(
            incoming_segments.size() == 2,
            "%s has both directed ring approach headings" % station_id
        )
        for incoming in incoming_segments:
            var actual_heading: float = _heading_for_segment_end(incoming.id)
            for destination_id in network.get_station_ids():
                if station_id == destination_id:
                    continue
                var pose: Dictionary = planner.plan_route_from_pose(
                    station_id,
                    destination_id,
                    stop.position_xz,
                    actual_heading
                )
                requests_tested += 1
                _expect(
                    pose.valid,
                    "%s approach %s reaches %s (%s)"
                        % [
                            station_id,
                            incoming.id,
                            destination_id,
                            pose.get("failure_reason", "")
                        ]
                )
                if not pose.valid:
                    continue
                _expect(
                    pose.origin_heading_error_rad
                        <= HEADING_TOLERANCE_RAD + GEOMETRY_TOLERANCE,
                    "%s -> %s first tangent matches incoming %s heading"
                        % [station_id, destination_id, incoming.id]
                )
                var outgoing_id: String = str(pose.segment_ids[0])
                _expect(
                    _turn_is_permitted(incoming.id, outgoing_id),
                    "%s -> %s uses a turn permitted from incoming %s"
                        % [station_id, destination_id, incoming.id]
                )
                _assert_route_geometry(
                    pose,
                    "%s approach %s -> %s" % [station_id, incoming.id, destination_id]
                )
    _expect(
        requests_tested == 24,
        "all 24 station/directed-approach/destination combinations were exercised"
    )

func _test_heading_compatible_longer_return_routes() -> void:
    var library_position: Vector2 = network.get_station_stop("library").position_xz
    var library_arrival_heading: float = _heading_for_segment_end("north_to_east")
    var library_return: Dictionary = planner.plan_route_from_pose(
        "library",
        "north_gate",
        library_position,
        library_arrival_heading
    )
    _expect(
        library_return.valid
            && library_return.segment_ids == PackedStringArray([
                "east_to_south",
                "south_to_west",
                "west_to_north"
            ]),
        "Library return continues around the ring instead of taking the opposite-heading shortcut"
    )
    var legacy_library_return: Dictionary = planner.plan_route("library", "north_gate")
    _expect(
        legacy_library_return.segment_ids == PackedStringArray(["east_to_north"])
            && library_return.total_length_m > legacy_library_return.total_length_m,
        "heading-aware Library return is a longer legal route than the legacy shortest route"
    )
    _assert_route_geometry(library_return, "Library return")

    var sports_position: Vector2 = network.get_station_stop("sports_hall").position_xz
    var sports_arrival_heading: float = _heading_for_segment_end("east_to_south")
    var sports_return: Dictionary = planner.plan_route_from_pose(
        "sports_hall",
        "north_gate",
        sports_position,
        sports_arrival_heading
    )
    _expect(
        sports_return.valid
            && sports_return.segment_ids == PackedStringArray([
                "south_to_west",
                "west_to_north"
            ]),
        "Sports Hall return continues around the ring from the actual arrival heading"
    )
    var legacy_sports_return: Dictionary = planner.plan_route("sports_hall", "north_gate")
    _expect(
        legacy_sports_return.segment_ids == PackedStringArray([
            "south_to_east",
            "east_to_north"
        ]) && sports_return.segment_ids != legacy_sports_return.segment_ids
            && is_equal_approx(
                sports_return.total_length_m,
                legacy_sports_return.total_length_m
            ),
        "heading-aware Sports Hall return selects the equal-cost direction-compatible ring path"
    )
    _assert_route_geometry(sports_return, "Sports Hall return")

func _test_determinism_and_mutation_protection() -> void:
    var position: Vector2 = network.get_station_stop("library").position_xz
    var heading: float = _heading_for_segment_end("north_to_east")
    var first: Dictionary = planner.plan_route_from_pose(
        "library",
        "north_gate",
        position,
        heading
    )
    var repeated: Dictionary = planner.plan_route_from_pose(
        "library",
        "north_gate",
        position,
        heading
    )
    _expect(
        first.valid && repeated.valid && first.segment_ids == repeated.segment_ids,
        "identical heading-aware requests deterministically select the same path"
    )
    _expect(
        first.centerline_points == repeated.centerline_points,
        "identical heading-aware requests return identical route geometry"
    )
    _expect(
        position == network.get_station_stop("library").position_xz
            && is_equal_approx(heading, _heading_for_segment_end("north_to_east")),
        "planning leaves the caller's vehicle position and heading unchanged"
    )
    var stored_route: Dictionary = network.get_segment("north_to_east")
    stored_route.centerline_points.clear()
    _expect(
        not network.get_segment("north_to_east").centerline_points.is_empty(),
        "route planning uses network read copies and does not mutate authoritative geometry"
    )

func _test_journey_route_override_handoff() -> void:
    var dispatcher = ShuttleDispatcher.new(network, planner)
    var booking: Dictionary = dispatcher.request_booking("library", "sports_hall")
    var journey = PassengerJourney.new(network, dispatcher)
    var assigned: Dictionary = journey.assign_booking(booking)
    var origin_position: Vector2 = network.get_station_stop("north_gate").position_xz
    var origin_heading: float = _heading_for_segment_start("north_to_east")
    var pickup_route: Dictionary = planner.plan_route_from_pose(
        "north_gate",
        "library",
        origin_position,
        origin_heading
    )
    var vehicle_state := {
        "position_xz": origin_position,
        "heading_rad": origin_heading,
        "speed_mps": 0.0,
        "localized": true,
        "manual_intervention": false
    }
    var install_request: Dictionary = journey.request_next_route_installation(
        vehicle_state,
        pickup_route
    )
    _expect(
        booking.valid && assigned.valid && pickup_route.valid
            && install_request.valid
            && install_request.centerline_points == pickup_route.centerline_points,
        "journey accepts a pose-planned route override for the active leg"
    )
    var mismatch: Dictionary = planner.plan_route_from_pose(
        "north_gate",
        "sports_hall",
        origin_position,
        origin_heading
    )
    var mismatch_request: Dictionary = journey.request_next_route_installation(
        vehicle_state,
        mismatch
    )
    _expect(
        not mismatch_request.valid
            && journey.state == PassengerJourney.BLOCKED
            && journey.blocked_reason == "route_destination_mismatch",
        "journey rejects a heading-aware route override for a different station leg"
    )

func _test_closed_loop_heading_aware_returns() -> void:
    for delta in [1.0 / 60.0, 1.0 / 120.0]:
        for destination_id in ["library", "sports_hall"]:
            var dynamics = VehicleDynamics.new()
            var navigator = RouteNavigator.new()
            var controller = RouteFollowingController.new()
            var start_position: Vector2 = network.get_station_stop("north_gate").position_xz
            var start_heading: float = _heading_for_segment_start("north_to_east")
            dynamics.reset(start_position.y, 0.0, start_position.x, start_heading)

            var outbound: Dictionary = planner.plan_route_from_pose(
                "north_gate",
                destination_id,
                Vector2(dynamics.position_x_m, dynamics.position_z_m),
                dynamics.heading_rad
            )
            _expect(
                outbound.valid && navigator.set_route(outbound.centerline_points),
                "north_gate outbound to %s initializes at %.6f s" % [destination_id, delta]
            )
            var outbound_drive: Dictionary = _drive_leg(
                dynamics,
                navigator,
                controller,
                delta
            )
            _expect(
                outbound_drive.arrived,
                "north_gate -> %s physically arrives at %.6f s (%s)"
                    % [destination_id, delta, outbound_drive.reason]
            )
            if not outbound_drive.arrived:
                continue

            var outbound_stop: Dictionary = network.get_station_stop(destination_id)
            var actual_position := Vector2(dynamics.position_x_m, dynamics.position_z_m)
            var position_error_m: float = actual_position.distance_to(outbound_stop.position_xz)
            var actual_heading: float = dynamics.heading_rad
            _expect(
                position_error_m <= POSITION_TOLERANCE_M
                    && dynamics.speed_mps <= 0.1,
                "%s outbound ends at its station stopped and within handoff position tolerance"
                    % destination_id
            )
            var return_route: Dictionary = planner.plan_route_from_pose(
                destination_id,
                "north_gate",
                actual_position,
                actual_heading
            )
            _expect(
                return_route.valid
                    && return_route.origin_heading_error_rad
                        <= PassengerJourney.HEADING_TOLERANCE_RAD,
                "%s return route is selected from actual vehicle arrival heading"
                    % destination_id
            )
            if not return_route.valid:
                continue
            var pose_before_install := Vector2(dynamics.position_x_m, dynamics.position_z_m)
            var heading_before_install: float = dynamics.heading_rad
            _expect(
                navigator.set_route(return_route.centerline_points),
                "%s heading-aware return installs in RouteNavigator" % destination_id
            )
            _expect(
                Vector2(dynamics.position_x_m, dynamics.position_z_m)
                    .distance_to(pose_before_install) <= 0.000001
                    && is_equal_approx(dynamics.heading_rad, heading_before_install),
                "%s return route installation preserves actual vehicle pose and heading"
                    % destination_id
            )
            var return_drive: Dictionary = _drive_leg(
                dynamics,
                navigator,
                controller,
                delta
            )
            _expect(
                return_drive.arrived,
                "%s -> north_gate physically returns at %.6f s (%s)"
                    % [destination_id, delta, return_drive.reason]
            )
            _expect(
                Vector2(dynamics.position_x_m, dynamics.position_z_m).distance_to(
                    network.get_station_stop("north_gate").position_xz
                ) <= POSITION_TOLERANCE_M
                    && dynamics.speed_mps <= 0.1,
                "%s return ends at the service stop without vehicle reset" % destination_id
            )

func _drive_leg(
    dynamics: RefCounted,
    navigator: RefCounted,
    controller: RefCounted,
    delta: float
) -> Dictionary:
    var result := {"arrived": false, "reason": "timeout", "ticks": 0}
    for tick in range(MAX_SIMULATION_TICKS):
        var control: Dictionary = controller.update(dynamics, navigator, delta)
        result.ticks = tick + 1
        if control.status == "ARRIVED" && control.get("arrival_certified", false):
            result.arrived = true
            result.reason = "arrival_certified"
            break
        if control.status == "SAFE STOP / INVALID NAVIGATION" \
            || control.status == "MISSED DESTINATION":
            result.reason = str(control.get("reason", control.status))
            break
        dynamics.step(
            float(control.get("requested_acceleration_mps2", 0.0)),
            delta,
            float(control.get("requested_steering_rad", 0.0))
        )
    return result

func _turn_is_permitted(incoming_segment_id: String, outgoing_segment_id: String) -> bool:
    var incoming: Dictionary = network.get_segment(incoming_segment_id)
    var junction: Dictionary = network.get_node(str(incoming.get("end_node_id", "")))
    for turn in junction.get("allowed_turns", []):
        if turn.get("incoming_segment_id", "") == incoming_segment_id \
            && turn.get("outgoing_segment_id", "") == outgoing_segment_id:
            return true
    return false
