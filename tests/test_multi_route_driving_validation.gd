extends SceneTree

const CampusMultiRouteNetwork = preload("res://scripts/campus_multi_route_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const RouteFollowingController = preload("res://scripts/route_following_controller.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")

const FIXED_DELTA: float = 1.0 / 60.0
const MAX_SIMULATION_TICKS: int = 12000
const CROSS_TRACK_LIMIT_M: float = 0.75
const ENDPOINT_TOLERANCE_M: float = 1.0
const ARRIVAL_SPEED_TOLERANCE_MPS: float = 0.1
const JUNCTION_SPEED_WINDOW_M: float = 2.0
const JUNCTION_HEADING_SAMPLE_OFFSET_M: float = 1.0
const OSCILLATION_STEERING_DEADBAND_RAD: float = deg_to_rad(1.0)

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

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    network = CampusMultiRouteNetwork.new()
    _expect(network.is_valid, "multi-route network passes structural validation")
    if not network.is_valid:
        push_error("Multi-route validation aborted: %s" % "; ".join(network.get_validation_errors()))
        quit(1)
        return

    var cases: Array[Dictionary] = [
        {
            "name": "North Gate -> Library (original curved ring)",
            "origin": "north_gate",
            "destination": "library",
            "unavailable": PackedStringArray(),
            "expected_segments": PackedStringArray(["north_to_east"]),
            "turn_focus": ""
        },
        {
            "name": "North Gate -> Library (bypass left branch)",
            "origin": "north_gate",
            "destination": "library",
            "unavailable": PackedStringArray([
                "north_to_east",
                "bypass_straight_to_library",
                "bypass_right_to_library"
            ]),
            "expected_segments": PackedStringArray([
                "north_to_bypass_mid",
                "bypass_left_to_library"
            ]),
            "turn_focus": "left"
        },
        {
            "name": "North Gate -> Library (bypass right branch)",
            "origin": "north_gate",
            "destination": "library",
            "unavailable": PackedStringArray([
                "north_to_east",
                "bypass_straight_to_library",
                "bypass_left_to_library"
            ]),
            "expected_segments": PackedStringArray([
                "north_to_bypass_mid",
                "bypass_right_to_library"
            ]),
            "turn_focus": "right"
        },
        {
            "name": "Library -> Innovation Centre (multi-segment straight bypass)",
            "origin": "library",
            "destination": "innovation_centre",
            "unavailable": PackedStringArray(["east_to_south", "east_to_north"]),
            "expected_segments": PackedStringArray([
                "library_to_bypass_straight",
                "bypass_mid_to_north",
                "north_to_west"
            ]),
            "turn_focus": "straight"
        }
    ]

    var completed_routes: int = 0
    for route_case in cases:
        var planner_network: RefCounted = network
        if not route_case.unavailable.is_empty():
            planner_network = SegmentFilteredNetwork.new(
                network,
                route_case.unavailable
            )
        var plan: Dictionary = CampusRoutePlanner.new(planner_network).plan_route(
            route_case.origin,
            route_case.destination
        )
        var plan_valid: bool = plan.get("valid", false)
        _expect(
            plan_valid,
            "%s plans with existing planner (%s)"
                % [route_case.name, plan.get("failure_reason", "")]
        )
        if not plan_valid:
            print("DRIVING RESULT %s: plan failed: %s" % [
                route_case.name,
                plan.get("details", plan.get("failure_reason", "unknown"))
            ])
            continue

        _expect(
            plan.segment_ids == route_case.expected_segments,
            "%s selects the intended directed roads: %s"
                % [route_case.name, ", ".join(plan.segment_ids)]
        )
        var drive: Dictionary = _drive_route(plan)
        _report_route(route_case, plan, drive)
        if drive.arrived:
            completed_routes += 1
        _expect(
            drive.arrived,
            "%s completes under closed-loop fixed-step control (%s)"
                % [route_case.name, drive.terminal_status]
        )
        _expect(
            drive.failure_reason.is_empty(),
            "%s has no controller or localization failure (%s)"
                % [route_case.name, drive.failure_reason]
        )
        _expect(
            drive.maximum_cross_track_error_m <= CROSS_TRACK_LIMIT_M,
            "%s maximum cross-track error %.3f m is within %.2f m"
                % [
                    route_case.name,
                    drive.maximum_cross_track_error_m,
                    CROSS_TRACK_LIMIT_M
                ]
        )
        _expect(
            drive.endpoint_error_m <= ENDPOINT_TOLERANCE_M,
            "%s ends within %.2f m of destination (%.3f m)"
                % [route_case.name, ENDPOINT_TOLERANCE_M, drive.endpoint_error_m]
        )
        _expect(
            drive.final_speed_mps <= ARRIVAL_SPEED_TOLERANCE_MPS,
            "%s stops within %.2f m/s (%.3f m/s)"
                % [
                    route_case.name,
                    ARRIVAL_SPEED_TOLERANCE_MPS,
                    drive.final_speed_mps
                ]
        )
        _expect(
            drive.invalid_localization_count == 0,
            "%s retains localization on every control step" % route_case.name
        )
        _expect(
            drive.wrong_way_ticks == 0,
            "%s has no wrong-way vehicle-heading samples (%d)"
                % [route_case.name, drive.wrong_way_ticks]
        )
        _expect(
            drive.maximum_speed_mps <= drive.speed_limit_mps + 0.0001,
            "%s stays within vehicle speed limit %.2f m/s"
                % [route_case.name, drive.speed_limit_mps]
        )
        _expect(
            drive.maximum_steering_rad <= drive.steering_limit_rad + 0.0001,
            "%s stays within steering limit %.2f degrees"
                % [route_case.name, rad_to_deg(drive.steering_limit_rad)]
        )
        _expect(
            drive.junctions.size() == maxi(plan.segment_ids.size() - 1, 0),
            "%s records entry and exit heading observations at every junction"
                % route_case.name
        )

    _expect(completed_routes == cases.size(), "all representative multi-route drives complete")
    print(
        "Multi-route driving validation: %d/%d routes completed; %d assertions."
            % [completed_routes, cases.size(), assertions]
    )
    if failures == 0:
        quit(0)
    else:
        push_error("%d of %d multi-route driving assertions failed." % [failures, assertions])
        quit(1)

func _drive_route(plan: Dictionary) -> Dictionary:
    var dynamics = VehicleDynamics.new()
    var navigator = RouteNavigator.new()
    var controller = RouteFollowingController.new()
    if not navigator.set_route(plan.centerline_points):
        return _empty_drive_result("route_navigator_rejected_centerline", plan)
    var start_segment: Dictionary = network.get_segment(plan.segment_ids[0])
    var start_points: Array = start_segment.centerline_points
    var start_tangent: Vector2 = (start_points[1] - start_points[0]).normalized()
    var initial_heading_rad: float = atan2(start_tangent.x, start_tangent.y)
    var start_position: Vector2 = plan.centerline_points[0]
    dynamics.reset(
        start_position.y,
        0.0,
        start_position.x,
        initial_heading_rad
    )

    var result := _empty_drive_result("", plan)
    var traces: Array[Dictionary] = []
    var previous_steering_sign: int = 0
    var last_route_progress_m: float = 0.0
    var dynamics_step_count: int = 0
    var terminal_status: String = "TIMEOUT"

    for tick in range(MAX_SIMULATION_TICKS):
        var control: Dictionary = controller.update(dynamics, navigator, FIXED_DELTA)
        var progress_m: float = float(control.get("route_progress_m", 0.0))
        var steering_rad: float = float(control.get("requested_steering_rad", 0.0))
        var speed_before_step: float = dynamics.speed_mps
        result.tick_count = tick + 1
        result.maximum_cross_track_error_m = maxf(
            result.maximum_cross_track_error_m,
            absf(float(control.get("cross_track_error_m", 0.0)))
        )
        result.maximum_steering_rad = maxf(
            result.maximum_steering_rad,
            absf(steering_rad)
        )
        result.maximum_speed_mps = maxf(result.maximum_speed_mps, speed_before_step)
        result.maximum_route_progress_m = maxf(result.maximum_route_progress_m, progress_m)
        if not control.get("localized", false):
            result.invalid_localization_count += 1
        if progress_m + 0.0001 < last_route_progress_m:
            result.progress_regression_count += 1
        last_route_progress_m = maxf(last_route_progress_m, progress_m)

        var steering_sign: int = 0
        if absf(steering_rad) >= OSCILLATION_STEERING_DEADBAND_RAD:
            steering_sign = 1 if steering_rad > 0.0 else -1
        if steering_sign != 0:
            if previous_steering_sign != 0 && steering_sign != previous_steering_sign:
                result.steering_direction_reversals += 1
            previous_steering_sign = steering_sign

        var tangent_heading: float = _route_heading_at_progress(navigator, progress_m)
        var heading_error_rad: float = absf(wrapf(
            dynamics.heading_rad - tangent_heading,
            -PI,
            PI
        ))
        if heading_error_rad > PI / 2.0:
            result.wrong_way_ticks += 1
        traces.append({
            "progress_m": progress_m,
            "heading_rad": dynamics.heading_rad,
            "speed_mps": speed_before_step,
            "steering_rad": steering_rad
        })

        if control.status == "SAFE STOP / INVALID NAVIGATION" \
            || control.status == "MISSED DESTINATION":
            result.failure_reason = str(control.get("reason", control.status))
            terminal_status = control.status
            break
        if control.status == "ARRIVED" && control.get("arrival_certified", false):
            result.arrived = true
            terminal_status = control.status
            break
        dynamics.step(
            float(control.get("requested_acceleration_mps2", 0.0)),
            FIXED_DELTA,
            steering_rad
        )
        dynamics_step_count += 1

    var final_position := Vector2(dynamics.position_x_m, dynamics.position_z_m)
    var destination_stop: Dictionary = network.get_station_stop(plan.destination_station_id)
    result.final_position_xz = final_position
    result.final_heading_rad = dynamics.heading_rad
    result.final_speed_mps = dynamics.speed_mps
    result.endpoint_error_m = final_position.distance_to(destination_stop.position_xz)
    result.speed_limit_mps = dynamics.max_speed
    result.steering_limit_rad = dynamics.steering_limit_rad
    result.elapsed_simulation_s = float(dynamics_step_count) * FIXED_DELTA
    result.terminal_status = terminal_status
    result.junctions = _measure_junctions(plan.segment_ids, navigator, traces)
    return result

func _measure_junctions(
    segment_ids: PackedStringArray,
    navigator: RefCounted,
    traces: Array[Dictionary]
) -> Array[Dictionary]:
    var samples: PackedVector2Array = navigator.get_centerline_samples()
    var cumulative: PackedFloat64Array = navigator.get_cumulative_lengths()
    var measurements: Array[Dictionary] = []
    for segment_index in range(segment_ids.size() - 1):
        var incoming: Dictionary = network.get_segment(segment_ids[segment_index])
        var outgoing: Dictionary = network.get_segment(segment_ids[segment_index + 1])
        var junction_id: String = str(incoming.end_node_id)
        var junction: Dictionary = network.get_node(junction_id)
        var junction_position: Vector2 = junction.position_xz
        var closest_index: int = 0
        var closest_distance_squared: float = INF
        for sample_index in range(samples.size()):
            var distance_squared: float = samples[sample_index].distance_squared_to(
                junction_position
            )
            if distance_squared < closest_distance_squared:
                closest_distance_squared = distance_squared
                closest_index = sample_index
        var boundary_progress_m: float = cumulative[closest_index]
        var incoming_points: Array = incoming.centerline_points
        var outgoing_points: Array = outgoing.centerline_points
        var incoming_tangent: Vector2 = (
            incoming_points[incoming_points.size() - 1]
            - incoming_points[incoming_points.size() - 2]
        ).normalized()
        var outgoing_tangent: Vector2 = (
            outgoing_points[1] - outgoing_points[0]
        ).normalized()
        var entry_trace: Dictionary = _nearest_trace(
            traces,
            boundary_progress_m - JUNCTION_HEADING_SAMPLE_OFFSET_M
        )
        var exit_trace: Dictionary = _nearest_trace(
            traces,
            boundary_progress_m + JUNCTION_HEADING_SAMPLE_OFFSET_M
        )
        var maximum_turn_speed_mps: float = 0.0
        for trace in traces:
            if absf(float(trace.progress_m) - boundary_progress_m) <= JUNCTION_SPEED_WINDOW_M:
                maximum_turn_speed_mps = maxf(
                    maximum_turn_speed_mps,
                    float(trace.speed_mps)
                )
        var turn_direction: String = _turn_direction(
            junction,
            str(incoming.id),
            str(outgoing.id)
        )
        measurements.append({
            "junction_id": junction_id,
            "incoming_segment_id": incoming.id,
            "outgoing_segment_id": outgoing.id,
            "turn_direction": turn_direction,
            "boundary_progress_m": boundary_progress_m,
            "entry_heading_error_deg": _heading_error_degrees(
                float(entry_trace.get("heading_rad", 0.0)),
                incoming_tangent
            ),
            "exit_heading_error_deg": _heading_error_degrees(
                float(exit_trace.get("heading_rad", 0.0)),
                outgoing_tangent
            ),
            "maximum_turn_speed_mps": maximum_turn_speed_mps
        })
    return measurements

func _nearest_trace(traces: Array[Dictionary], target_progress_m: float) -> Dictionary:
    var nearest: Dictionary = {}
    var nearest_error_m: float = INF
    for trace in traces:
        var error_m: float = absf(float(trace.progress_m) - target_progress_m)
        if error_m < nearest_error_m:
            nearest_error_m = error_m
            nearest = trace
    return nearest

func _turn_direction(node: Dictionary, incoming_id: String, outgoing_id: String) -> String:
    for turn in node.get("allowed_turns", []):
        if turn.incoming_segment_id == incoming_id \
            && turn.outgoing_segment_id == outgoing_id:
            return str(turn.get("turn_direction", "unspecified"))
    return "unlisted"

func _heading_error_degrees(actual_heading_rad: float, tangent: Vector2) -> float:
    var expected_heading_rad: float = atan2(tangent.x, tangent.y)
    return rad_to_deg(absf(wrapf(actual_heading_rad - expected_heading_rad, -PI, PI)))

func _route_heading_at_progress(navigator: RefCounted, progress_m: float) -> float:
    var samples: PackedVector2Array = navigator.get_centerline_samples()
    var cumulative: PackedFloat64Array = navigator.get_cumulative_lengths()
    var closest_index: int = 0
    var progress_error_m: float = INF
    for index in range(cumulative.size()):
        var error_m: float = absf(cumulative[index] - progress_m)
        if error_m < progress_error_m:
            progress_error_m = error_m
            closest_index = index
    var tangent_index: int = mini(closest_index + 1, samples.size() - 1)
    if tangent_index == closest_index:
        tangent_index = maxi(closest_index - 1, 0)
    var tangent: Vector2 = (samples[tangent_index] - samples[closest_index]).normalized()
    return atan2(tangent.x, tangent.y)

func _segment_shape(segment: Dictionary) -> String:
    var points: Array = segment.centerline_points
    var first_tangent: Vector2 = (points[1] - points[0]).normalized()
    for index in range(2, points.size()):
        var tangent: Vector2 = (points[index] - points[index - 1]).normalized()
        if absf(first_tangent.cross(tangent)) > 0.001:
            return "curved"
    return "straight"

func _report_route(
    route_case: Dictionary,
    plan: Dictionary,
    drive: Dictionary
) -> void:
    var shapes: PackedStringArray = PackedStringArray()
    for segment_id in plan.segment_ids:
        var segment: Dictionary = network.get_segment(segment_id)
        shapes.append("%s:%s" % [segment_id, _segment_shape(segment)])
    var route_format: String = (
        "DRIVE %s | status=%s failure=%s simulated=%.2fs steps=%d roads=[%s] "
        + "max_cte=%.3fm endpoint=%.3fm final_speed=%.3fm/s max_speed=%.3fm/s "
        + "max_steering=%.2fdeg steering_reversals=%d wrong_way_ticks=%d "
        + "progress_regressions=%d containment=UNVERIFIED"
    )
    print(route_format % [
        route_case.name,
        drive.terminal_status,
        drive.failure_reason,
        drive.elapsed_simulation_s,
        drive.tick_count,
        ", ".join(shapes),
        drive.maximum_cross_track_error_m,
        drive.endpoint_error_m,
        drive.final_speed_mps,
        drive.maximum_speed_mps,
        rad_to_deg(drive.maximum_steering_rad),
        drive.steering_direction_reversals,
        drive.wrong_way_ticks,
        drive.progress_regression_count
    ])
    for junction in drive.junctions:
        var junction_format: String = (
            "  JUNCTION %s %s->%s turn=%s entry_heading_error=%.2fdeg "
            + "exit_heading_error=%.2fdeg max_speed_within_%.1fm=%.3fm/s"
        )
        print(junction_format % [
            junction.junction_id,
            junction.incoming_segment_id,
            junction.outgoing_segment_id,
            junction.turn_direction,
            junction.entry_heading_error_deg,
            junction.exit_heading_error_deg,
            JUNCTION_SPEED_WINDOW_M,
            junction.maximum_turn_speed_mps
        ])
    if drive.junctions.is_empty():
        print("  JUNCTION none (single segment)")
    print("  ROAD CONTAINMENT: UNVERIFIED (network has no road widths; vehicle footprint unspecified)")

func _empty_drive_result(failure_reason: String, plan: Dictionary) -> Dictionary:
    return {
        "origin_station_id": plan.get("origin_station_id", ""),
        "destination_station_id": plan.get("destination_station_id", ""),
        "arrived": false,
        "terminal_status": "SETUP FAILED",
        "failure_reason": failure_reason,
        "tick_count": 0,
        "elapsed_simulation_s": 0.0,
        "final_position_xz": Vector2.ZERO,
        "final_heading_rad": 0.0,
        "final_speed_mps": 0.0,
        "endpoint_error_m": INF,
        "maximum_cross_track_error_m": 0.0,
        "maximum_steering_rad": 0.0,
        "maximum_speed_mps": 0.0,
        "speed_limit_mps": 0.0,
        "steering_limit_rad": 0.0,
        "maximum_route_progress_m": 0.0,
        "invalid_localization_count": 0,
        "wrong_way_ticks": 0,
        "progress_regression_count": 0,
        "steering_direction_reversals": 0,
        "junctions": []
    }

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1
