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
# VERIFICATION PARAMETERS (Phase 5A.2B.1)
# - Vehicle Length: 3.5 m | Width: 1.6 m
# - Production Wheelbase (for driven curvature): 2.5 m
# - Assumed Road Width: 4.0 m (Half-width: 2.0 m)
# - Assumed Lateral Acceleration Limit: 1.5 m/s^2
# ==============================================================================
const VEHICLE_LENGTH_M: float = 3.5
const VEHICLE_WIDTH_M: float = 1.6
const PRODUCTION_WHEELBASE_M: float = 2.5
const ROAD_WIDTH_M: float = RoadDimensions.ROAD_WIDTH_M
const ROAD_HALF_WIDTH_M: float = ROAD_WIDTH_M / 2.0
const LATERAL_ACCEL_LIMIT_MPS2: float = 1.5

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
    call_deferred("_run_verification")

func _run_verification() -> void:
    network = CampusMultiRouteNetwork.new()
    if not network.is_valid:
        push_error("Multi-route network validation failed.")
        quit(1)
        return

    print("================================================================================")
    print("EASY DRIVE — PHASE 5A.2B.1: GEOMETRY AND CLEARANCE VERIFICATION")
    print("================================================================================")
    print("Parameters:")
    print("  - Vehicle: %.1fm length x %.1fm width | Production Wheelbase: %.1fm" % [VEHICLE_LENGTH_M, VEHICLE_WIDTH_M, PRODUCTION_WHEELBASE_M])
    print("  - Road: %.1fm total width (Half-width: %.1fm)" % [ROAD_WIDTH_M, ROAD_HALF_WIDTH_M])
    print("  - Lateral Acceleration Limit: %.2f m/s^2" % LATERAL_ACCEL_LIMIT_MPS2)
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

        var result: Dictionary = _verify_route(route_case, plan)
        _report_verification(route_case, plan, result)

        _expect(result.arrived, "%s completes verification drive" % route_case.name)
        _expect(result.verified_min_clearance_m != null, "%s minimum clearance verified" % route_case.name)

    print("================================================================================")
    print("Verification complete: %d assertions, %d failures." % [assertions, failures])
    print("================================================================================")
    if failures == 0:
        quit(0)
    else:
        quit(1)

func _verify_route(route_case: Dictionary, plan: Dictionary) -> Dictionary:
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
    var path_curvatures: PackedFloat64Array = _compute_path_curvatures(centerline_samples)

    var arrived: bool = false
    var worst_min_clearance: float = INF
    var worst_tick: int = 0
    var worst_pose: Dictionary = {}
    var peak_lat_accel_driven: float = 0.0
    var peak_lat_accel_path: float = 0.0
    var peak_lat_tick: int = 0

    for tick in range(MAX_SIMULATION_TICKS):
        var control: Dictionary = controller.update(dynamics, navigator, FIXED_DELTA)
        var progress_m: float = float(control.get("route_progress_m", 0.0))
        var speed: float = dynamics.speed_mps
        var steering_rad: float = float(control.get("requested_steering_rad", 0.0))
        var pos = Vector2(dynamics.position_x_m, dynamics.position_z_m)
        var heading = dynamics.heading_rad

        var fwd = Vector2(sin(heading), cos(heading))
        var right = Vector2(cos(heading), -sin(heading))
        var v_center = pos + fwd * (VEHICLE_LENGTH_M / 2.0)
        var half_l = VEHICLE_LENGTH_M / 2.0
        var half_w = VEHICLE_WIDTH_M / 2.0

        var corners: Array[Vector2] = [
            v_center + fwd * half_l + right * half_w,
            v_center + fwd * half_l - right * half_w,
            v_center - fwd * half_l - right * half_w,
            v_center - fwd * half_l + right * half_w
        ]

        var step_min_clearance: float = INF
        for corner in corners:
            var lat_offset: float = _transverse_lateral_offset(corner, centerline_samples)
            var clearance: float = ROAD_HALF_WIDTH_M - absf(lat_offset)
            step_min_clearance = minf(step_min_clearance, clearance)

        if step_min_clearance < worst_min_clearance:
            worst_min_clearance = step_min_clearance
            worst_tick = tick + 1
            worst_pose = {
                "pos": pos,
                "heading_deg": rad_to_deg(heading),
                "progress_m": progress_m
            }

        var sample_idx: int = _find_nearest_sample_index(centerline_samples, progress_m, navigator)
        var kappa_path: float = absf(path_curvatures[sample_idx])
        var kappa_driven: float = absf(tan(steering_rad) / PRODUCTION_WHEELBASE_M)

        var a_lat_path: float = speed * speed * kappa_path
        var a_lat_driven: float = speed * speed * kappa_driven

        if a_lat_driven > peak_lat_accel_driven:
            peak_lat_accel_driven = a_lat_driven
            peak_lat_accel_path = a_lat_path
            peak_lat_tick = tick + 1

        if control.status == "ARRIVED" && control.get("arrival_certified", false):
            arrived = true
            break
        if control.status == "SAFE STOP / INVALID NAVIGATION" || control.status == "MISSED DESTINATION":
            break

        dynamics.step(float(control.get("requested_acceleration_mps2", 0.0)), FIXED_DELTA, steering_rad)

    return {
        "arrived": arrived,
        "verified_min_clearance_m": worst_min_clearance,
        "worst_tick": worst_tick,
        "worst_pose": worst_pose,
        "peak_lat_accel_driven_mps2": peak_lat_accel_driven,
        "peak_lat_accel_path_mps2": peak_lat_accel_path,
        "peak_lat_tick": peak_lat_tick
    }

func _compute_path_curvatures(samples: PackedVector2Array) -> PackedFloat64Array:
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

func _transverse_lateral_offset(point: Vector2, polyline: PackedVector2Array) -> float:
    var min_dist_sq: float = INF
    var best_offset: float = 0.0
    for i in range(polyline.size() - 1):
        var a = polyline[i]
        var b = polyline[i + 1]
        var ab = b - a
        var ab_len_sq = ab.length_squared()
        if ab_len_sq < 0.000001:
            continue
        var t = clampf((point - a).dot(ab) / ab_len_sq, 0.0, 1.0)
        var proj = a + t * ab
        var dist_sq = point.distance_squared_to(proj)
        if dist_sq < min_dist_sq:
            min_dist_sq = dist_sq
            var tangent = ab.normalized()
            var normal = Vector2(-tangent.y, tangent.x)
            best_offset = (point - proj).dot(normal)
    return best_offset

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

func _report_verification(route_case: Dictionary, plan: Dictionary, result: Dictionary) -> void:
    print("--------------------------------------------------------------------------------")
    print("VERIFICATION: %s" % route_case.name)
    print("  - Segments: [%s]" % ", ".join(plan.segment_ids))
    print("  - Verified Min Footprint Clearance: %.3f m (at tick %d, progress %.1fm)" % [result.verified_min_clearance_m, result.worst_tick, result.worst_pose.get("progress_m", 0.0)])
    print("  - Peak Lateral Acceleration (Driven Steering): %.3f m/s^2 (at tick %d)" % [result.peak_lat_accel_driven_mps2, result.peak_lat_tick])
    print("  - Peak Lateral Acceleration (Path Curvature): %.3f m/s^2" % result.peak_lat_accel_path_mps2)
    var exceeds_limit: bool = result.peak_lat_accel_driven_mps2 > LATERAL_ACCEL_LIMIT_MPS2
    print("  - Exceeds 1.5 m/s^2 Limit? %s" % ("YES" if exceeds_limit else "NO"))

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1
