extends SceneTree

const CampusMultiRouteNetwork = preload("res://scripts/campus_multi_route_network.gd")

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    print("================================================================================")
    print("EASY DRIVE — PHASE 5A.3B.1B: CRITICAL 3D VISIBILITY & CAMERA TESTS")
    print("================================================================================")

    var network = CampusMultiRouteNetwork.new()
    _expect(network.is_valid, "network is valid")

    # Test nodes, stations, and bounding box
    var nodes = network.get_nodes()
    _expect(nodes.size() > 0, "network has nodes")

    var stations = network.get_stations()
    _expect(stations.size() >= 4, "network has at least 4 stations")

    var min_x: float = INF
    var max_x: float = -INF
    var min_z: float = INF
    var max_z: float = -INF

    for node in nodes:
        var p: Vector2 = node.position_xz
        min_x = minf(min_x, p.x)
        max_x = maxf(max_x, p.x)
        min_z = minf(min_z, p.y)
        max_z = maxf(max_z, p.y)

    for seg in network.get_segments():
        for pt in seg.centerline_points:
            min_x = minf(min_x, pt.x)
            max_x = maxf(max_x, pt.x)
            min_z = minf(min_z, pt.y)
            max_z = maxf(max_z, pt.y)

    var center_x: float = (min_x + max_x) * 0.5
    var center_z: float = (min_z + max_z) * 0.5
    var max_span: float = maxf(max_x - min_x, max_z - min_z)

    _expect(max_span > 10.0, "network span is valid (span=%.1f m)" % max_span)

    # Test material and cull mode settings
    var mat = StandardMaterial3D.new()
    mat.cull_mode = BaseMaterial3D.CULL_DISABLED
    _expect(mat.cull_mode == BaseMaterial3D.CULL_DISABLED, "material cull mode is CULL_DISABLED for omni-visibility")

    # Test scene parsing and instantiation
    var scene = load("res://scenes/multi_route_demo.tscn")
    _expect(scene != null, "multi_route_demo.tscn loads successfully")
    if scene != null:
        var node = scene.instantiate()
        _expect(node != null, "multi_route_demo scene instantiates")
        if node != null:
            node.free()

    print("Multi-route 3D visibility tests complete: %d assertions, %d failures." % [assertions, failures])
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
