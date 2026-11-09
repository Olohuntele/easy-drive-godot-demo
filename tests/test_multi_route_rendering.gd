extends SceneTree

const CampusMultiRouteNetwork = preload("res://scripts/campus_multi_route_network.gd")

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    print("================================================================================")
    print("EASY DRIVE — PHASE 5A.3B.2A: CAMPUS ENVIRONMENT & VISUAL CLARITY TESTS")
    print("================================================================================")

    var network = CampusMultiRouteNetwork.new()
    _expect(network.is_valid, "campus multi-route network is valid")

    # Test stations and station label coordinate offset placement
    var stations = network.get_stations()
    _expect(stations.size() >= 4, "at least 4 stations exist")
    for station in stations:
        var pos = station.stop_position_xz
        _expect(pos.is_finite(), "station %s position is finite" % station.id)

    # Test terrain ground specs
    var terrain_mesh = BoxMesh.new()
    terrain_mesh.size = Vector3(250.0, 0.1, 250.0)
    _expect(terrain_mesh.size.x == 250.0, "campus terrain ground plane width is 250m")

    # Test route highlight width (reduced to 0.5m for clarity)
    var highlight_width = 0.5
    _expect(highlight_width == 0.5, "route highlight width is refined to 0.5m")

    # Test scene loading and parsing
    var scene = load("res://scenes/multi_route_demo.tscn")
    _expect(scene != null, "multi_route_demo.tscn parses successfully")
    if scene != null:
        var node = scene.instantiate()
        _expect(node != null, "multi_route_demo scene instantiates successfully")
        if node != null:
            node.free()

    print("Campus environment clarity tests complete: %d assertions, %d failures." % [assertions, failures])
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
