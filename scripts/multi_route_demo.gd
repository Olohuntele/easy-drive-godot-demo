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

var overview_camera: Camera3D = null
var camera_mode_overview: bool = true

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
    print("[MULTI-ROUTE DEMO] Initialized successfully. Press 1-4 to select routes, Space to toggle run, R to reset, C to toggle camera.")

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
            KEY_C:
                _toggle_camera()
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
    _update_active_route_visuals()
    print("[MULTI-ROUTE DEMO] Selected Route %d: %s (Segments: [%s])" % [index, current_route_name, ", ".join(current_route_segments)])

var active_route_mesh_instance: MeshInstance3D = null

func _build_network_visuals() -> void:
    # 1. Calculate bounding box for overview camera and layout
    var min_x: float = INF
    var max_x: float = -INF
    var min_z: float = INF
    var max_z: float = -INF

    for node in network.get_nodes():
        var p: Vector2 = node.position_xz
        min_x = minf(min_x, p.x)
        max_x = maxf(max_x, p.x)
        min_z = minf(min_z, p.y)
        max_z = maxf(max_z, p.y)

    for segment in network.get_segments():
        for pt in segment.centerline_points:
            min_x = minf(min_x, pt.x)
            max_x = maxf(max_x, pt.x)
            min_z = minf(min_z, pt.y)
            max_z = maxf(max_z, pt.y)

    var center_x: float = (min_x + max_x) * 0.5
    var center_z: float = (min_z + max_z) * 0.5
    var span_x: float = max_x - min_x
    var span_z: float = max_z - min_z
    var max_span: float = maxf(span_x, span_z)

    # Setup overview camera framing the entire network (added to tree FIRST, then set current)
    overview_camera = Camera3D.new()
    overview_camera.name = "OverviewCamera"
    var overview_height: float = maxf(60.0, max_span * 1.1)
    overview_camera.position = Vector3(center_x, overview_height, center_z + max_span * 0.2)
    add_child(overview_camera)
    overview_camera.look_at(Vector3(center_x, 0.0, center_z), Vector3.UP)
    overview_camera.current = true
    camera_mode_overview = true

    # 2. Appropriately sized junction connection plates (radius 2.0m matching road half-width, CULL_DISABLED)
    var junction_mat = StandardMaterial3D.new()
    junction_mat.albedo_color = Color(0.16, 0.18, 0.20, 1.0)
    junction_mat.roughness = 0.8
    junction_mat.cull_mode = BaseMaterial3D.CULL_DISABLED

    for node in network.get_nodes():
        var pos: Vector2 = node.position_xz
        var cyl = MeshInstance3D.new()
        var cyl_mesh = CylinderMesh.new()
        cyl_mesh.top_radius = 2.0
        cyl_mesh.bottom_radius = 2.0
        cyl_mesh.height = 0.02
        cyl.mesh = cyl_mesh
        cyl.material_override = junction_mat
        cyl.position = Vector3(pos.x, 0.015, pos.y)
        road_root.add_child(cyl)

    # 3. Asphalt road segments (4.0m width) with edge markings and centerline (CULL_DISABLED)
    var asphalt_mat = StandardMaterial3D.new()
    asphalt_mat.albedo_color = Color(0.18, 0.20, 0.22, 1.0)
    asphalt_mat.roughness = 0.8
    asphalt_mat.metallic = 0.05
    asphalt_mat.cull_mode = BaseMaterial3D.CULL_DISABLED

    var marking_mat = StandardMaterial3D.new()
    marking_mat.albedo_color = Color(0.95, 0.95, 0.95, 1.0)
    marking_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
    marking_mat.cull_mode = BaseMaterial3D.CULL_DISABLED

    for segment in network.get_segments():
        var points: Array = segment.centerline_points
        if points.size() < 2:
            continue

        # Road surface (4.0m total width, half-width 2.0m)
        var road_mesh = _make_route_strip_mesh(points, 0.0, 4.0, 0.025)
        var road_instance = MeshInstance3D.new()
        road_instance.mesh = road_mesh
        road_instance.material_override = asphalt_mat
        road_root.add_child(road_instance)

        # Centerline marking (white strip, 0.15m width)
        var center_marking = _make_route_strip_mesh(points, 0.0, 0.15, 0.035)
        var center_inst = MeshInstance3D.new()
        center_inst.mesh = center_marking
        center_inst.material_override = marking_mat
        road_root.add_child(center_inst)

        # Left edge shoulder line (0.1m width at offset -1.9m)
        var left_edge = _make_route_strip_mesh(points, -1.9, 0.1, 0.035)
        var left_inst = MeshInstance3D.new()
        left_inst.mesh = left_edge
        left_inst.material_override = marking_mat
        road_root.add_child(left_inst)

        # Right edge shoulder line (0.1m width at offset +1.9m)
        var right_edge = _make_route_strip_mesh(points, 1.9, 0.1, 0.035)
        var right_inst = MeshInstance3D.new()
        right_inst.mesh = right_edge
        right_inst.material_override = marking_mat
        road_root.add_child(right_inst)

    # 4. Station markers and readable Label3Ds
    for station in network.get_stations():
        var pos: Vector2 = station.stop_position_xz
        var cyl = MeshInstance3D.new()
        var cyl_mesh = CylinderMesh.new()
        cyl_mesh.top_radius = 1.2
        cyl_mesh.bottom_radius = 1.2
        cyl_mesh.height = 0.2
        var cyl_mat = StandardMaterial3D.new()
        cyl_mat.albedo_color = Color(0.1, 0.6, 0.9, 1.0)
        cyl_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
        cyl_mesh.material = cyl_mat
        cyl.mesh = cyl_mesh
        cyl.position = Vector3(pos.x, 0.1, pos.y)
        station_root.add_child(cyl)

        var label = Label3D.new()
        label.text = str(station.display_name).replace(" Stop", "")
        label.position = Vector3(pos.x, 2.5, pos.y)
        label.font_size = 36
        label.outline_size = 8
        label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
        station_root.add_child(label)

func _toggle_camera() -> void:
    camera_mode_overview = !camera_mode_overview
    if camera_mode_overview:
        if overview_camera:
            overview_camera.current = true
        print("[MULTI-ROUTE DEMO] Switched to Overview Camera")
    else:
        if follow_camera:
            follow_camera.current = true
        print("[MULTI-ROUTE DEMO] Switched to Follow Camera")

func _update_active_route_visuals() -> void:
    if active_route_mesh_instance == null:
        active_route_mesh_instance = MeshInstance3D.new()
        active_route_mesh_instance.name = "ActiveRouteHighlight"
        var active_mat = StandardMaterial3D.new()
        active_mat.albedo_color = Color(1.0, 0.65, 0.15, 0.85)
        active_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
        active_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
        active_route_mesh_instance.material_override = active_mat
        road_root.add_child(active_route_mesh_instance)

    var combined_points: Array[Vector2] = []
    for seg_id in current_route_segments:
        var seg = network.get_segment(seg_id)
        if not seg.is_empty():
            for pt in seg.centerline_points:
                if combined_points.is_empty() or combined_points[combined_points.size() - 1].distance_to(pt) > 0.01:
                    combined_points.append(pt)

    if combined_points.size() >= 2:
        active_route_mesh_instance.mesh = _make_route_strip_mesh(combined_points, 0.0, 1.2, 0.045)
        active_route_mesh_instance.visible = true
    else:
        active_route_mesh_instance.visible = false

func _make_route_strip_mesh(
    centerline_samples: Array,
    lateral_offset_m: float,
    strip_width_m: float,
    height_m: float
) -> ArrayMesh:
    var vertices := PackedVector3Array()
    var normals := PackedVector3Array()
    var indices := PackedInt32Array()
    var half_strip_width: float = strip_width_m * 0.5
    for sample_index in range(centerline_samples.size()):
        var previous_index: int = maxi(sample_index - 1, 0)
        var next_index: int = mini(sample_index + 1, centerline_samples.size() - 1)
        var tangent: Vector2 = (
            centerline_samples[next_index] - centerline_samples[previous_index]
        ).normalized()
        if tangent.length_squared() < 0.0001:
            tangent = Vector2(0, 1)
        var right: Vector2 = Vector2(tangent.y, -tangent.x)
        var center: Vector2 = centerline_samples[sample_index] + right * lateral_offset_m
        var left_edge: Vector2 = center - right * half_strip_width
        var right_edge: Vector2 = center + right * half_strip_width
        vertices.append(Vector3(left_edge.x, height_m, left_edge.y))
        vertices.append(Vector3(right_edge.x, height_m, right_edge.y))
        normals.append(Vector3.UP)
        normals.append(Vector3.UP)
        if sample_index > 0:
            var base: int = (sample_index - 1) * 2
            indices.append_array(PackedInt32Array([
                base, base + 2, base + 1,
                base + 1, base + 2, base + 3
            ]))

    var arrays: Array = []
    arrays.resize(Mesh.ARRAY_MAX)
    arrays[Mesh.ARRAY_VERTEX] = vertices
    arrays[Mesh.ARRAY_NORMAL] = normals
    arrays[Mesh.ARRAY_INDEX] = indices
    var mesh := ArrayMesh.new()
    mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
    return mesh

func _update_camera(delta: float) -> void:
    if camera_mode_overview:
        return
    if not ego_car or not follow_camera:
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
    var cam_str = "OVERVIEW" if camera_mode_overview else "FOLLOW"
    var total_len: float = navigator.total_length_m if navigator else 0.0
    var text = (
        "EASY DRIVE — MULTI-ROUTE DEMO (Phase 5A.3B.1B)\n"
        + "Status: %s | Camera: %s | Route: [%d] %s\n"
        + "Segments: [%s]\n"
        + "Speed: %.2f m/s | Total Length: %.1f m\n"
        + "CONTROLS:\n"
        + "  [1]-[3] North Gate -> Library (Ring/Bypasses)\n"
        + "  [4] Library -> Innovation Centre (Bypass)\n"
        + "  [C] Toggle Camera (%s) | [Space] Pause | [R] Reset"
    ) % [
        status_str,
        cam_str,
        active_route_index,
        current_route_name,
        ", ".join(current_route_segments),
        dynamics.speed_mps,
        total_len,
        "Overview" if camera_mode_overview else "Follow"
    ]
    hud_label.text = text
