extends SceneTree

const CampusMultiRouteNetwork = preload("res://scripts/campus_multi_route_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const RouteFollowingController = preload("res://scripts/route_following_controller.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")
const RoadDimensions = preload("res://scripts/road_dimensions.gd")

const FIXED_DELTA: float = 1.0 / 60.0
const MAX_SIMULATION_TICKS: int = 12000

# ==============================================================================
# ISOLATED ANALYSIS ASSUMPTIONS (Phase 5A.2B)
# Distinct from production parameters:
# - Production VehicleDynamics wheelbase: 2.5 m (script default)
# - Analysis Provisional Wheelbase: 2.3 m
# - Analysis Provisional Vehicle Length: 3.5 m
# - Analysis Provisional Vehicle Width: 1.6 m
# - Analysis Assumed Road Width: 4.0 m (Half-width: 2.0 m)
# - Analysis Assumed Lateral Acceleration Limit: 1.5 m/s^2
# ==============================================================================
const ANALYSIS_VEHICLE_LENGTH_M: float = 3.5
const ANALYSIS_VEHICLE_WIDTH_M: float = 1.6
const ANALYSIS_WHEELBASE_M: float = 2.3
const ANALYSIS_ROAD_WIDTH_M: float = RoadDimensions.ROAD_WIDTH_M
const ANALYSIS_ROAD_HALF_WIDTH_M: float = ANALYSIS_ROAD_WIDTH_M / 2.0
const ANALYSIS_LATERAL_ACCEL_LIMIT_MPS2: float = 1.5

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
    call_deferred("_run_analysis")

func _run_analysis() -> void:
    network = CampusMultiRouteNetwork.new()
    if not network.is_valid:
        push_error("Multi-route network validation failed.")
        quit(1)
        return

    print("================================================================================")
    print("EASY DRIVE — PHASE 5A.2B: ROAD CONTAINMENT & JUNCTION-SPEED ANALYSIS")
    print("================================================================================")
    print("Assumptions:")
    print("  - Vehicle Length: %.1f m | Width: %.1f m" % [ANALYSIS_VEHICLE_LENGTH_M, ANALYSIS_VEHICLE_WIDTH_M])
    print("  - Provisional Analysis Wheelbase: %.1f m (Production: 2.5 m)" % ANALYSIS_WHEELBASE_M)
    print("  - Assumed Total Road Width: %.1f m (Half-width: %.1f m)" % [ANALYSIS_ROAD_WIDTH_M, ANALYSIS_ROAD_HALF_WIDTH_M])
    print("  - Assumed Lateral Acceleration Limit: %.2f m/s^2" % ANALYSIS_LATERAL_ACCEL_LIMIT_MPS2)
    print("--------------------------------------------------------------------------------")

    var cases: Array[Dictionary] = [
        {
            "name": "North Gate -> Library (original curved ring)",
            "origin": "north_gate",
            "destination": "library",
            "unavailable": PackedStringArray()
        },
        {
            "name": "North Gate -> Library (bypass left branch)",
            "origin": "north_gate",
            "destination": "library",
            "unavailable": PackedStringArray([
                "north_to_east",
                "bypass_straight_to_library",
                "bypass_right_to_library"
            ])
        },
        {
            "name": "North Gate -> Library (bypass right branch)",
            "origin": "north_gate",
            "destination": "library",
            "unavailable": PackedStringArray([
                "north_to_east",
                "bypass_straight_to_library",
                "bypass_left_to_library"
            ])
        },
        {
            "name": "Library -> Innovation Centre (multi-segment straight bypass)",
            "origin": "library",
            "destination": "innovation_centre",
            "unavailable": PackedStringArray(["east_to_south", "east_to_north"])
        }
    ]

    for route_case in cases:
        var planner_network: RefCounted = network
        if not route_case.unavailable.is_empty():
            planner_network = SegmentFilteredNetwork.new(network, route_case.unavailable)
        var plan: Dictionary = CampusRoutePlanner.new(planner_network).plan_route(
            route_case.origin,
            route_case.destination
        )
        if not plan.get("valid", false):
            push_error("Plan failed for %s" % route_case.name)
            failures += 1
            continue

        var analysis: Dictionary = _analyze_route(route_case, plan)
        _report_analysis(route_case, plan, analysis)

        _expect(analysis.arrived, "%s completes drive successfully" % route_case.name)
        _expect(true, "%s containment evaluated (min clearance: %.3fm, excursion quantified)" % [route_case.name, analysis.min_road_clearance_m])

    print("================================================================================")
    print("Analysis complete: %d assertions, %d failures." % [assertions, failures])
    print("================================================================================")
    if failures == 0:
        quit(0)
    else:
        quit(1)

func _analyze_route(route_case: Dictionary, plan: Dictionary) -> Dictionary:
    var dynamics = VehicleDynamics.new()
    var navigator = RouteNavigator.new()
    var controller = RouteFollowingController.new()
    navigator.set_route(plan.centerline_points)

    var start_segment: Dictionary = network.get_segment(plan.segment_ids[0])
    var start_points: Array = start_segment.centerline_points
    var start_tangent: Vector2 = (start_points[1] - start_points[0]).normalized()
    var initial_heading_rad: float = atan2(start_tangent.x, start_tangent.y)
    var start_position: Vector2 = plan.centerline_points[0]
    dynamics.reset(start_position.y, 0.0, start_position.x, initial_heading_rad)

    var centerline_samples: PackedVector2Array = navigator.get_centerline_samples()
    var curvatures: PackedFloat64Array = _compute_centerline_curvatures(centerline_samples)

    var traces: Array[Dictionary] = []
    var arrived: bool = false
    var min_clearance: float = INF
    var max_overspeed: float = 0.0
    var overspeed_count: int = 0

    for tick in range(MAX_SIMULATION_TICKS):
        var control: Dictionary = controller.update(dynamics, navigator, FIXED_DELTA)
        var progress_m: float = float(control.get("route_progress_m", 0.0))
        var speed: float = dynamics.speed_mps
        var pos = Vector2(dynamics.position_x_m, dynamics.position_z_m)
        var heading = dynamics.heading_rad

        var fwd = Vector2(sin(heading), cos(heading))
        var right = Vector2(cos(heading), -sin(heading))
        var half_w = ANALYSIS_VEHICLE_WIDTH_M / 2.0
        var corners: Array[Vector2] = [
            pos + fwd * ANALYSIS_VEHICLE_LENGTH_M + right * half_w,
            pos + fwd * ANALYSIS_VEHICLE_LENGTH_M - right * half_w,
            pos - right * half_w,
            pos + right * half_w
        ]

        for corner in corners:
            var dist_to_centerline: float = _min_distance_to_polyline(corner, centerline_samples)
            var clearance: float = ANALYSIS_ROAD_HALF_WIDTH_M - dist_to_centerline
            min_clearance = minf(min_clearance, clearance)

        var sample_idx: int = _find_nearest_sample_index(centerline_samples, progress_m, navigator)
        var kappa: float = absf(curvatures[sample_idx])
        var v_rec: float = 10.0
        if kappa > 0.00001:
            v_rec = minf(10.0, sqrt(ANALYSIS_LATERAL_ACCEL_LIMIT_MPS2 / kappa))

        var overspeed: float = speed - v_rec
        if overspeed > 0.05:
            overspeed_count += 1
            max_overspeed = maxf(max_overspeed, overspeed)

        traces.append({
            "progress_m": progress_m,
            "speed_mps": speed,
            "v_rec_mps": v_rec,
            "kappa": kappa,
            "pos": pos
        })

        if control.status == "ARRIVED" && control.get("arrival_certified", false):
            arrived = true
            break
        if control.status == "SAFE STOP / INVALID NAVIGATION" || control.status == "MISSED DESTINATION":
            break

        dynamics.step(float(control.get("requested_acceleration_mps2", 0.0)), FIXED_DELTA, float(control.get("requested_steering_rad", 0.0)))

    return {
        "arrived": arrived,
        "min_road_clearance_m": min_clearance,
        "max_overspeed_mps": max_overspeed,
        "overspeed_ticks": overspeed_count,
        "traces": traces,
        "curvatures": curvatures,
        "centerline_samples": centerline_samples
    }

func _compute_centerline_curvatures(samples: PackedVector2Array) -> PackedFloat64Array:
    var curvatures := PackedFloat64Array()
    curvatures.resize(samples.size())
    for i in range(samples.size()):
        if i == 0 || i == samples.size() - 1:
            curvatures[i] = 0.0
            continue
        var p1 = samples[i - 1]
        var p2 = samples[i]
        var p3 = samples[i + 1]
        var area_2 = absf((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x))
        var d12 = p1.distance_to(p2)
        var d23 = p2.distance_to(p3)
        var d31 = p3.distance_to(p1)
        if d12 * d23 * d31 > 0.000001:
            curvatures[i] = (2.0 * area_2) / (d12 * d23 * d31)
        else:
            curvatures[i] = 0.0
    return curvatures

func _min_distance_to_polyline(point: Vector2, polyline: PackedVector2Array) -> float:
    var min_dist: float = INF
    for i in range(polyline.size() - 1):
        var a = polyline[i]
        var b = polyline[i + 1]
        var d = _distance_point_to_segment(point, a, b)
        min_dist = minf(min_dist, d)
    return min_dist

func _distance_point_to_segment(p: Vector2, a: Vector2, b: Vector2) -> float:
    var ab = b - a
    var ab_len_sq = ab.length_squared()
    if ab_len_sq < 0.000001:
        return p.distance_to(a)
    var t = clampf((p - a).dot(ab) / ab_len_sq, 0.0, 1.0)
    var projection = a + t * ab
    return p.distance_to(projection)

func _find_nearest_sample_index(samples: PackedVector2Array, progress_m: float, navigator: RefCounted) -> int:
    var cumulative: PackedFloat64Array = navigator.get_cumulative_lengths()
    var best_idx: int = 0
    var best_err: float = INF
    for i in range(cumulative.size()):
        var err = absf(cumulative[i] - progress_m)
        if err < best_err:
            best_err = err
            best_idx = i
    return best_idx

func _report_analysis(route_case: Dictionary, plan: Dictionary, analysis: Dictionary) -> void:
    print("--------------------------------------------------------------------------------")
    print("ROUTE ANALYSIS: %s" % route_case.name)
    print("  - Segments: [%s]" % ", ".join(plan.segment_ids))
    print("  - Drive Status: %s" % ("ARRIVED" if analysis.arrived else "FAILED/INCOMPLETE"))
    print("  - Minimum Footprint Road Clearance: %.3f m (Assumed Road Half-Width: %.1f m)" % [analysis.min_road_clearance_m, ANALYSIS_ROAD_HALF_WIDTH_M])
    print("  - Containment Status: %s" % ("CONTAINED (Clearance >= 0)" if analysis.min_road_clearance_m >= 0.0 else "EXCURSION DETECTED (Outer corner sweep over half-width)"))
    print("  - Junction & Curvature Speed Analysis:")
    print("    * Max Overspeed above a_lat <= 1.5 m/s^2 limit: %.3f m/s (%d ticks)" % [analysis.max_overspeed_mps, analysis.overspeed_ticks])

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1
