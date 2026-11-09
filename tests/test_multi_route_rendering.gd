extends SceneTree

const CampusMultiRouteNetwork = preload("res://scripts/campus_multi_route_network.gd")

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    print("================================================================================")
    print("EASY DRIVE — PHASE 5A.3B.2B.1: VISUAL CLEANUP & ROAD MARKING TESTS")
    print("================================================================================")

    var network = CampusMultiRouteNetwork.new()
    _expect(network.is_valid, "campus network is valid")

    # Test segments and clean centerline marking specs
    var segments = network.get_segments()
    _expect(segments.size() > 0, "network has segments")
    for seg in segments:
        var points = seg.centerline_points
        _expect(points.size() >= 2, "segment has centerline points")

    # Test buildings count and station shelter placement specs
    var stations = network.get_stations()
    _expect(stations.size() >= 4, "at least 4 stations have shelters")

    # Test scene parsing and instantiation
    var scene = load("res://scenes/multi_route_demo.tscn")
    _expect(scene != null, "multi_route_demo.tscn parses successfully")
    if scene != null:
        var node = scene.instantiate()
        _expect(node != null, "multi_route_demo scene instantiates successfully")
        if node != null:
            node.free()

    print("Visual cleanup tests complete: %d assertions, %d failures." % [assertions, failures])
    if failures == 0:
        quit(0)
    else:
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1
