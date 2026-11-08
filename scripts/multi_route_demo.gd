extends Node3D

const CampusMultiRouteNetwork = preload("res://scripts/campus_multi_route_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const RouteFollowingController = preload("res://scripts/route_following_controller.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")

const FIXED_DELTA: float = 1.0 / 60.0

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

var network: RefCounted
var dynamics = VehicleDynamics.new()
var navigator = RouteNavigator.new()
var controller = RouteFollowingController.new()

var active_route_index: int = 1
var simulation_running: bool = true
var route_active: bool = false
var current_route_name: String = ""
var current_route_segments: PackedStringArray = PackedStringArray()

@onready var ego_car: Node3D = $EgoCar
@onready var follow_camera: Camera3D = $FollowCamera
@onready var hud_label: Label = $CanvasLayer/TelemetryPanel/HUD
@onready var road_root: Node3D = $RoadRoot
@onready var station_root: Node3D = $StationRoot

func _ready() -> void:
    network = CampusMultiRouteNetwork.new()
    if not network.is_valid:
        push_error("Multi-route demo network invalid: %s" % "; ".join(network.get_validation_errors()))
        return

    _build_network_visuals()
    _select_route(1)
    print("[MULTI-ROUTE DEMO] Initialized successfully. Press 1-4 to select routes, Space to toggle run, R to reset.")

func _input(event: InputEvent) -> void:
    if event is InputEventKey and event.pressed and not event.echo:
        match event.keycode:
            KEY_1, KEY_KP_1:
                _select_route(1)
            KEY_2, KEY_KP_2:
                _select_route(2)
            KEY_3, KEY_KP_3:
                _select_route(3)
            KEY_4, KEY_KP_4:
                _select_route(4)
            KEY_R:
                _select_route(active_route_index)
            KEY_SPACE:
                simulation_running = !simulation_running

func _process(delta: float) -> void:
    _update_camera(delta)
    _update_hud()

func _physics_process(delta: float) -> void:
    if not simulation_running || not route_active:
        return

    var control: Dictionary = controller.update(dynamics, navigator, FIXED_DELTA)
    var steering_rad: float = float(control.get("requested_steering_rad", 0.0))
    var acceleration_mps2: float = float(control.get("requested_acceleration_mps2", 0.0))

    if control.status == "ARRIVED" && control.get("arrival_certified", false):
        route_active = false
        print("[MULTI-ROUTE DEMO] Route ARRIVED: %s" % current_route_name)
    elif control.status == "SAFE STOP / INVALID NAVIGATION" || control.status == "MISSED DESTINATION":
        route_active = false
        print("[MULTI-ROUTE DEMO] Route terminated: %s (%s)" % [current_route_name, control.status])

    dynamics.step(acceleration_mps2, FIXED_DELTA, steering_rad)

    if ego_car:
        ego_car.position = Vector3(dynamics.position_x_m, 0.2, dynamics.position_z_m)
        ego_car.rotation.y = dynamics.heading_rad

func _select_route(index: int) -> void:
    active_route_index = index
    var planner_network: RefCounted = network
    var unavailable := PackedStringArray()

    match index:
        1:
            current_route_name = "North Gate -> Library (Original Ring)"
            unavailable = PackedStringArray()
        2:
            current_route_name = "North Gate -> Library (Bypass Left)"
            unavailable = PackedStringArray([
                "north_to_east",
                "bypass_straight_to_library",
                "bypass_right_to_library"
            ])
        3:
            current_route_name = "North Gate -> Library (Bypass Right)"
            unavailable = PackedStringArray([
                "north_to_east",
                "bypass_straight_to_library",
                "bypass_left_to_library"
            ])
        4:
            current_route_name = "Library -> Innovation Centre (Multi-Segment Bypass)"
            unavailable = PackedStringArray(["east_to_south", "east_to_north"])

    if not unavailable.is_empty():
        planner_network = SegmentFilteredNetwork.new(network, unavailable)

    var origin_id: String = "north_gate" if index <= 3 else "library"
    var dest_id: String = "library" if index <= 3 else "innovation_centre"

    var plan: Dictionary = CampusRoutePlanner.new(planner_network).plan_route(origin_id, dest_id)
    if not plan.get("valid", false):
        push_error("Failed to plan route for index %d: %s" % [index, plan.get("failure_reason", "unknown")])
        route_active = false
        return

    current_route_segments = plan.segment_ids
    if not navigator.set_route(plan.centerline_points):
        push_error("RouteNavigator rejected centerline for route %d" % index)
        route_active = false
        return

    var start_segment: Dictionary = network.get_segment(plan.segment_ids[0])
    var start_points: Array = start_segment.centerline_points
    var start_tangent: Vector2 = (start_points[1] - start_points[0]).normalized()
    var initial_heading: float = atan2(start_tangent.x, start_tangent.y)
    var start_pos: Vector2 = plan.centerline_points[0]

    dynamics.reset(start_pos.y, 0.0, start_pos.x, initial_heading)
    controller.reset()
    route_active = true
    simulation_running = true
    print("[MULTI-ROUTE DEMO] Selected Route %d: %s (Segments: [%s])" % [index, current_route_name, ", ".join(current_route_segments)])

func _build_network_visuals() -> void:
    var line_mat = StandardMaterial3D.new()
    line_mat.albedo_color = Color(0.95, 0.95, 0.95, 1.0)

    for segment in network.get_segments():
        var points: Array = segment.centerline_points
        if points.size() < 2:
            continue
        var immediate_mesh = ImmediateMesh.new()
        immediate_mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP, line_mat)
        for pt in points:
            immediate_mesh.surface_add_vertex(Vector3(pt.x, 0.05, pt.y))
        immediate_mesh.surface_end()

        var mesh_instance = MeshInstance3D.new()
        mesh_instance.mesh = immediate_mesh
        road_root.add_child(mesh_instance)

    for station in network.get_stations():
        var pos: Vector2 = station.stop_position_xz
        var cyl = MeshInstance3D.new()
        var cyl_mesh = CylinderMesh.new()
        cyl_mesh.top_radius = 1.2
        cyl_mesh.bottom_radius = 1.2
        cyl_mesh.height = 0.2
        var cyl_mat = StandardMaterial3D.new()
        cyl_mat.albedo_color = Color(0.1, 0.6, 0.9, 1.0)
        cyl_mesh.material = cyl_mat
        cyl.mesh = cyl_mesh
        cyl.position = Vector3(pos.x, 0.1, pos.y)
        station_root.add_child(cyl)

func _update_camera(delta: float) -> void:
    if not ego_car:
        return
    var target_pos = ego_car.position + Vector3(0.0, 4.0, -10.0)
    follow_camera.position = follow_camera.position.lerp(target_pos, delta * 5.0)
    follow_camera.look_at(ego_car.position + Vector3(0.0, 1.0, 2.0), Vector3.UP)

func _update_hud() -> void:
    if not hud_label:
        return
    var status_str = "RUNNING" if simulation_running else "PAUSED"
    if not route_active:
        status_str = "ARRIVED / STOPPED"
    var total_len: float = navigator.total_length_m if navigator else 0.0
    var text = (
        "EASY DRIVE — MULTI-ROUTE DEMO (Phase 5A.3A)\n"
        + "Status: %s | Active Route: [%d] %s\n"
        + "Segments: [%s]\n"
        + "Speed: %.2f m/s | Total Route Length: %.1f m\n"
        + "CONTROLS:\n"
        + "  [1] North Gate -> Library (Original Ring)\n"
        + "  [2] North Gate -> Library (Bypass Left)\n"
        + "  [3] North Gate -> Library (Bypass Right)\n"
        + "  [4] Library -> Innovation Centre (Bypass)\n"
        + "  [Space] Toggle Pause | [R] Reset Route"
    ) % [
        status_str,
        active_route_index,
        current_route_name,
        ", ".join(current_route_segments),
        dynamics.speed_mps,
        total_len
    ]
    hud_label.text = text
