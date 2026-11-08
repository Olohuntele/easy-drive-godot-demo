extends SceneTree

const CampusMultiRouteNetwork = preload("res://scripts/campus_multi_route_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const RouteFollowingController = preload("res://scripts/route_following_controller.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    print("================================================================================")
    print("EASY DRIVE — PHASE 5A.3A: MULTI-ROUTE DEMO & SCENE TESTS")
    print("================================================================================")

    var network = CampusMultiRouteNetwork.new()
    _expect(network.is_valid, "multi-route network is valid for demo scene")

    var planner = CampusRoutePlanner.new(network)
    var r1 = planner.plan_route("north_gate", "library")
    _expect(r1.get("valid", false), "demo route 1 plans successfully")

    # Test scene loading / parsing
    var scene = load("res://scenes/multi_route_demo.tscn")
    _expect(scene != null, "multi_route_demo.tscn loads successfully without parser errors")

    if scene != null:
        var node = scene.instantiate()
        _expect(node != null, "multi_route_demo scene instantiates successfully")
        if node != null:
            node.free()

    print("Multi-route demo tests complete: %d assertions, %d failures." % [assertions, failures])
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
