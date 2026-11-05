extends SceneTree

const RouteNavigator = preload("res://scripts/route_navigator.gd")

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    _test_straight_route_geometry()
    _test_smooth_bend_geometry()
    _test_signed_cross_track_error()
    _test_nearest_point_projection()
    _test_lookahead_interpolation()
    _test_monotonic_progress_and_local_tracking()
    _test_endpoint_clamping()
    _test_reset_and_relocalization()
    _test_invalid_route_handling()
    _test_deterministic_repeated_runs()

    if failures == 0:
        print("RouteNavigator tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d RouteNavigator assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _straight_route() -> RouteNavigator:
    var navigator = RouteNavigator.new()
    var waypoints: Array[Vector2] = [Vector2(0.0, 0.0), Vector2(0.0, 10.0)]
    navigator.set_route(waypoints, 10)
    return navigator

func _test_straight_route_geometry() -> void:
    var navigator = _straight_route()
    var samples: PackedVector2Array = navigator.get_centerline_samples()
    _expect(navigator.route_valid, "two valid waypoints create a route")
    _expect(samples.size() == 11, "straight route is sampled at the requested resolution")
    _expect(samples[0].is_equal_approx(Vector2(0.0, 0.0)), "route starts at its first waypoint")
    _expect(samples[samples.size() - 1].is_equal_approx(Vector2(0.0, 10.0)), "route ends at its last waypoint")
    _expect(absf(navigator.total_length_m - 10.0) < 0.0001, "straight route length is measured in meters")
    for sample in samples:
        _expect(absf(sample.x) < 0.0001, "straight route remains at X=0")

func _test_smooth_bend_geometry() -> void:
    var navigator = RouteNavigator.new()
    var waypoints: Array[Vector2] = RouteNavigator.create_broad_bend_waypoints()
    var succeeded: bool = navigator.set_route(waypoints, 24)
    var samples: PackedVector2Array = navigator.get_centerline_samples()
    var cumulative: PackedFloat64Array = navigator.get_cumulative_lengths()
    var has_rightward_bend: bool = false
    var has_positive_z_progress: bool = true
    for i in range(samples.size()):
        has_rightward_bend = has_rightward_bend || samples[i].x > 1.0
        if i > 0 && samples[i].y <= samples[i - 1].y:
            has_positive_z_progress = false
        if i > 0:
            _expect(cumulative[i] > cumulative[i - 1], "sample arc lengths increase strictly")
    _expect(succeeded, "documented broad bend creates successfully")
    _expect(samples.size() == (waypoints.size() - 1) * 24 + 1, "broad bend has a dense smooth centerline")
    _expect(has_rightward_bend, "broad bend gradually turns toward world +X")
    _expect(has_positive_z_progress, "broad bend progresses forward along world +Z")
    _expect(samples[0].is_equal_approx(waypoints[0]), "broad bend preserves its start")
    _expect(samples[samples.size() - 1].is_equal_approx(waypoints[waypoints.size() - 1]), "broad bend preserves its endpoint")

func _test_signed_cross_track_error() -> void:
    var navigator = _straight_route()
    var right_result: Dictionary = navigator.find_nearest_point(Vector2(2.0, 4.0), true)
    var left_result: Dictionary = navigator.find_nearest_point(Vector2(-2.0, 4.0), true)
    _expect(is_equal_approx(right_result.cross_track_error_m, 2.0), "positive cross-track means right of a +Z route")
    _expect(is_equal_approx(left_result.cross_track_error_m, -2.0), "negative cross-track means left of a +Z route")

func _test_nearest_point_projection() -> void:
    var navigator = _straight_route()
    var result: Dictionary = navigator.find_nearest_point(Vector2(3.0, 4.25), true)
    _expect(result.valid, "nearest point query succeeds on a valid route")
    _expect(result.position_xz.is_equal_approx(Vector2(0.0, 4.25)), "nearest point projects onto the centerline")
    _expect(is_equal_approx(result.progress_m, 4.25), "nearest point reports cumulative progress")
    _expect(is_equal_approx(result.distance_m, 3.0), "nearest point reports lateral distance")

func _test_lookahead_interpolation() -> void:
    var navigator = _straight_route()
    navigator.find_nearest_point(Vector2(0.0, 2.25), true)
    var lookahead: Dictionary = navigator.get_lookahead_point(3.5)
    _expect(lookahead.valid, "lookahead is available on a valid route")
    _expect(is_equal_approx(lookahead.progress_m, 5.75), "lookahead adds distance to current progress")
    _expect(lookahead.position_xz.is_equal_approx(Vector2(0.0, 5.75)), "lookahead interpolates between route samples")
    _expect(not lookahead.clamped_at_endpoint, "in-range lookahead is not endpoint-clamped")

func _test_monotonic_progress_and_local_tracking() -> void:
    var navigator = RouteNavigator.new()
    var waypoints: Array[Vector2] = [
        Vector2(0.0, 0.0),
        Vector2(0.0, 10.0),
        Vector2(1.0, 20.0),
        Vector2(0.0, 30.0),
        Vector2(0.0, 40.0)
    ]
    navigator.set_route(waypoints, 20)
    var first: Dictionary = navigator.find_nearest_point(Vector2(0.0, 8.0), true)
    var progress_before: float = first.progress_m
    var near_nonadjacent: Dictionary = navigator.find_nearest_point(Vector2(0.0, 30.0))
    _expect(near_nonadjacent.progress_m >= progress_before, "normal route progress never decreases")
    _expect(
        near_nonadjacent.progress_m <= progress_before + 6.0,
        "local tracking does not jump to a nearby nonadjacent segment"
    )
    var forward: Dictionary = navigator.find_nearest_point(Vector2(0.0, 12.0))
    _expect(forward.progress_m >= near_nonadjacent.progress_m, "progress advances monotonically while following forward")

func _test_endpoint_clamping() -> void:
    var navigator = _straight_route()
    navigator.relocalize(Vector2(0.0, 9.0))
    var lookahead: Dictionary = navigator.get_lookahead_point(5.0)
    _expect(lookahead.valid, "endpoint lookahead remains valid")
    _expect(lookahead.clamped_at_endpoint, "lookahead beyond route end is marked clamped")
    _expect(is_equal_approx(lookahead.progress_m, 10.0), "lookahead distance clamps to route length")
    _expect(lookahead.position_xz.is_equal_approx(Vector2(0.0, 10.0)), "clamped lookahead returns route endpoint")
    _expect(navigator.get_point_at_distance(100.0).is_equal_approx(Vector2(0.0, 10.0)), "point-at-distance clamps to endpoint")

func _test_reset_and_relocalization() -> void:
    var navigator = RouteNavigator.new()
    var waypoints: Array[Vector2] = [Vector2(0.0, 0.0), Vector2(0.0, 10.0), Vector2(10.0, 10.0)]
    navigator.set_route(waypoints, 20)
    navigator.find_nearest_point(Vector2(0.0, 8.0), true)
    var relocated: Dictionary = navigator.relocalize(Vector2(10.0, 10.0))
    _expect(relocated.valid, "explicit relocalization performs a global nearest search")
    _expect(relocated.progress_m > 15.0, "relocalization can recover at a distant route position")
    navigator.reset_progress()
    _expect(is_equal_approx(navigator.progress_m, 0.0), "reset clears route progress")
    _expect(not navigator.is_localized, "reset requires a new localization")
    var after_reset: Dictionary = navigator.find_nearest_point(Vector2(10.0, 10.0))
    _expect(after_reset.valid, "first query after reset initializes localization")
    _expect(after_reset.progress_m > 15.0, "first query after reset searches the full route")

func _test_invalid_route_handling() -> void:
    var navigator = RouteNavigator.new()
    var empty: Array[Vector2] = []
    _expect(not navigator.set_route(empty), "empty route is rejected")
    _expect(not navigator.route_valid, "rejected empty route is safely cleared")
    _expect(not navigator.find_nearest_point(Vector2.ZERO).valid, "nearest query safely reports invalid route")
    _expect(not navigator.get_lookahead_point(2.0).valid, "lookahead safely reports invalid route")

    var one_point: Array[Vector2] = [Vector2.ZERO]
    _expect(not navigator.set_route(one_point), "single-point route is rejected")
    var duplicate: Array[Vector2] = [Vector2.ZERO, Vector2.ZERO]
    _expect(not navigator.set_route(duplicate), "degenerate waypoint segment is rejected")
    var non_finite: Array[Vector2] = [Vector2.ZERO, Vector2(INF, 2.0)]
    _expect(not navigator.set_route(non_finite), "non-finite waypoint is rejected")
    var valid: Array[Vector2] = [Vector2.ZERO, Vector2(0.0, 2.0)]
    _expect(navigator.set_route(valid, 0) == false, "invalid sample count is rejected")
    _expect(not navigator.route_valid, "invalid route setup never leaves a partial route")

func _test_deterministic_repeated_runs() -> void:
    var waypoints: Array[Vector2] = RouteNavigator.create_broad_bend_waypoints()
    var first := _run_deterministic_route(waypoints)
    var second := _run_deterministic_route(waypoints)
    _expect(first.size() == second.size(), "repeated route runs return the same sample count")
    for i in range(first.size()):
        _expect(first[i].is_equal_approx(second[i]), "repeated route runs are deterministic at sample %d" % i)

func _run_deterministic_route(waypoints: Array[Vector2]) -> PackedVector2Array:
    var navigator = RouteNavigator.new()
    navigator.set_route(waypoints, 18)
    var results := PackedVector2Array()
    for i in range(101):
        var sample_position: Vector2 = Vector2(float(i) * 0.2, float(i))
        var nearest: Dictionary = navigator.find_nearest_point(sample_position)
        results.append(Vector2(nearest.progress_m, nearest.cross_track_error_m))
    return results
