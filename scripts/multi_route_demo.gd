extends Node3D

const CampusMultiRouteNetwork = preload("res://scripts/campus_multi_route_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")
const RoadDimensions = preload("res://scripts/road_dimensions.gd")
const CampusServiceCoordinator = preload("res://scripts/campus_service_coordinator.gd")
const PassengerBookingPanel = preload("res://scripts/passenger_booking_panel.gd")
const RouteFollowingController = preload("res://scripts/route_following_controller.gd")

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

var service_coordinator: RefCounted = null
var booking_panel: Control = null
var obstacle_controls: VBoxContainer = null
var place_obstacle_button: Button = null
var clear_obstacle_button: Button = null
var obstacle_visual: MeshInstance3D = null

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
@onready var canvas_layer: CanvasLayer = $CanvasLayer

func _ready() -> void:
    network = CampusMultiRouteNetwork.new()
    if not network.is_valid:
        push_error("Multi-route demo network invalid: %s" % "; ".join(network.get_validation_errors()))
        return

    # Initialize Service Coordinator & Booking Panel
    service_coordinator = CampusServiceCoordinator.new()
    service_coordinator.network = network
    service_coordinator.planner = CampusRoutePlanner.new(network)
    service_coordinator.defer_destination_routing = false
    service_coordinator.defer_return_routing = false
    var north_stop = network.get_station_stop("north_gate")
    var launch_segment: Dictionary = network.get_segment("north_to_east")
    var launch_points: Array = launch_segment.get("centerline_points", [])
    var launch_tangent: Vector2 = launch_points[1] - launch_points[0]
    var launch_heading_rad: float = atan2(launch_tangent.x, launch_tangent.y)
    service_coordinator.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)

    booking_panel = PassengerBookingPanel.new()
    booking_panel.name = "PassengerBookingPanel"
    canvas_layer.add_child(booking_panel)
    booking_panel.configure(network, service_coordinator.get_service_snapshot())
    booking_panel.booking_requested.connect(_on_passenger_booking_requested)
    booking_panel.boarding_requested.connect(_on_passenger_boarding_requested)
    booking_panel.passenger_exit_requested.connect(_on_passenger_passenger_exit_requested)
    _build_obstacle_controls()

    _build_network_visuals()
    _select_route(1)
    print("[MULTI-ROUTE DEMO + PASSENGER SERVICE] Initialized successfully. Press 1-4 for manual routes, B-panel for bookings, C for camera, Space to pause, R to reset.")

func _input(event: InputEvent) -> void:
    if event is InputEventKey and event.pressed and not event.echo:
        match event.keycode:
            KEY_1, KEY_KP_1:
                if _is_passenger_service_active():
                    print("[DEMO] Manual route 1 ignored: Passenger service is active.")
                else:
                    _select_route(1)
            KEY_2, KEY_KP_2:
                if _is_passenger_service_active():
                    print("[DEMO] Manual route 2 ignored: Passenger service is active.")
                else:
                    _select_route(2)
            KEY_3, KEY_KP_3:
                if _is_passenger_service_active():
                    print("[DEMO] Manual route 3 ignored: Passenger service is active.")
                else:
                    _select_route(3)
            KEY_4, KEY_KP_4:
                if _is_passenger_service_active():
                    print("[DEMO] Manual route 4 ignored: Passenger service is active.")
                else:
                    _select_route(4)
            KEY_C:
                _toggle_camera()
            KEY_R:
                _reset_simulation()
            KEY_SPACE:
                simulation_running = !simulation_running

func _is_passenger_service_active() -> bool:
    if service_coordinator == null:
        return false
    var state = str(service_coordinator.get_service_snapshot().get("service_state", "IDLE"))
    return state != "IDLE"

func _process(delta: float) -> void:
    _update_camera(delta)
    _update_hud()

func _physics_process(delta: float) -> void:
    if not simulation_running:
        return

    if _is_passenger_service_active():
        var step_res = service_coordinator.step(FIXED_DELTA)
        var snapshot = service_coordinator.get_service_snapshot()

        # Sync vehicle dynamics from coordinator snapshot
        dynamics._position_x_m = snapshot.position_xz.x
        dynamics._position_z_m = snapshot.position_xz.y
        dynamics._heading_rad = snapshot.heading_rad
        dynamics._speed_mps = snapshot.speed_mps

        if ego_car:
            ego_car.position = Vector3(dynamics.position_x_m, 0.2, dynamics.position_z_m)
            ego_car.rotation.y = dynamics.heading_rad

        if booking_panel:
            booking_panel.update_service_snapshot(snapshot)
        _update_obstacle_visual(snapshot)
        return

    if not route_active:
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

func _on_passenger_booking_requested(pickup_station_id: String, destination_station_id: String) -> void:
    if service_coordinator == null:
        return
    var result = service_coordinator.request_booking(pickup_station_id, destination_station_id)
    var snapshot = service_coordinator.get_service_snapshot()
    if booking_panel:
        booking_panel.show_booking_result(result, snapshot)
        booking_panel.update_service_snapshot(snapshot)
    print("[PASSENGER SERVICE] Booking requested (%s -> %s): valid=%s" % [pickup_station_id, destination_station_id, result.get("valid", false)])

func _on_passenger_boarding_requested() -> void:
    if service_coordinator == null:
        return
    var result = service_coordinator.confirm_boarding()
    var snapshot = service_coordinator.get_service_snapshot()
    if booking_panel:
        booking_panel.show_action_result("boarding", result, snapshot)
        booking_panel.update_service_snapshot(snapshot)

func _on_passenger_passenger_exit_requested() -> void:
    if service_coordinator == null:
        return
    var result = service_coordinator.confirm_passenger_exit()
    var snapshot = service_coordinator.get_service_snapshot()
    if booking_panel:
        booking_panel.show_action_result("exit", result, snapshot)
        booking_panel.update_service_snapshot(snapshot)

func _build_obstacle_controls() -> void:
    obstacle_controls = VBoxContainer.new()
    obstacle_controls.name = "ObstacleControls"
    obstacle_controls.anchor_left = 1.0
    obstacle_controls.anchor_right = 1.0
    obstacle_controls.offset_left = -500.0
    obstacle_controls.offset_right = -326.0
    obstacle_controls.offset_top = 300.0
    obstacle_controls.offset_bottom = 380.0
    obstacle_controls.add_theme_constant_override("separation", 5)
    canvas_layer.add_child(obstacle_controls)

    place_obstacle_button = Button.new()
    place_obstacle_button.text = "Place obstacle"
    place_obstacle_button.pressed.connect(_place_obstacle_pressed)
    obstacle_controls.add_child(place_obstacle_button)

    clear_obstacle_button = Button.new()
    clear_obstacle_button.text = "Clear obstacle"
    clear_obstacle_button.pressed.connect(_clear_obstacle_pressed)
    obstacle_controls.add_child(clear_obstacle_button)

func _place_obstacle_pressed() -> void:
    if service_coordinator == null || not service_coordinator.navigator.route_valid:
        return
    var navigation: Dictionary = service_coordinator.navigator.find_nearest_point(
        service_coordinator.get_service_snapshot().position_xz
    )
    if not navigation.get("valid", false):
        return
    var obstacle_distance_m: float = minf(
        float(navigation.progress_m) + 18.0,
        service_coordinator.navigator.total_length_m
            - RouteFollowingController.STOP_GAP_M
            - 0.5
    )
    if obstacle_distance_m <= float(navigation.progress_m):
        return
    service_coordinator.place_obstacle(obstacle_distance_m)
    _update_obstacle_visual(service_coordinator.get_service_snapshot())

func _clear_obstacle_pressed() -> void:
    if service_coordinator == null:
        return
    service_coordinator.clear_obstacle()
    _update_obstacle_visual(service_coordinator.get_service_snapshot())

func _update_obstacle_visual(snapshot: Dictionary) -> void:
    var obstacle: RefCounted = service_coordinator.active_obstacle \
        if service_coordinator != null else null
    if obstacle == null || not obstacle.active || not service_coordinator.navigator.route_valid:
        if obstacle_visual:
            obstacle_visual.visible = false
        return
    if obstacle_visual == null:
        obstacle_visual = MeshInstance3D.new()
        obstacle_visual.name = "RouteObstacle"
        var mesh := BoxMesh.new()
        mesh.size = Vector3(obstacle.width_m, 1.0, obstacle.length_m)
        obstacle_visual.mesh = mesh
        var material := StandardMaterial3D.new()
        material.albedo_color = Color(0.82, 0.05, 0.04, 1.0)
        material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
        obstacle_visual.material_override = material
        road_root.add_child(obstacle_visual)
    var point: Vector2 = service_coordinator.navigator.get_point_at_distance(
        obstacle.distance_along_route_m
    )
    var before: Vector2 = service_coordinator.navigator.get_point_at_distance(
        maxf(obstacle.distance_along_route_m - 0.1, 0.0)
    )
    var after: Vector2 = service_coordinator.navigator.get_point_at_distance(
        minf(
            obstacle.distance_along_route_m + 0.1,
            service_coordinator.navigator.total_length_m
        )
    )
    var tangent: Vector2 = (after - before).normalized()
    obstacle_visual.position = Vector3(point.x, 0.7, point.y)
    obstacle_visual.rotation.y = atan2(tangent.x, tangent.y)
    obstacle_visual.visible = true

func _reset_simulation() -> void:
    if service_coordinator != null:
        service_coordinator.clear_obstacle()
    if obstacle_visual:
        obstacle_visual.visible = false
    if service_coordinator != null:
        var north_stop = network.get_station_stop("north_gate")
        var launch_segment: Dictionary = network.get_segment("north_to_east")
        var launch_points: Array = launch_segment.get("centerline_points", [])
        var launch_tangent: Vector2 = launch_points[1] - launch_points[0]
        var launch_heading_rad: float = atan2(launch_tangent.x, launch_tangent.y)
        service_coordinator = CampusServiceCoordinator.new()
        service_coordinator.network = network
        service_coordinator.planner = CampusRoutePlanner.new(network)
        service_coordinator.defer_destination_routing = false
        service_coordinator.defer_return_routing = false
        service_coordinator.initialize_service("north_gate", north_stop.position_xz, launch_heading_rad)
        if booking_panel:
            booking_panel.configure(network, service_coordinator.get_service_snapshot())
    _select_route(active_route_index)

var active_route_mesh_instance: MeshInstance3D = null

func _build_network_visuals() -> void:
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

    overview_camera = Camera3D.new()
    overview_camera.name = "OverviewCamera"
    var overview_height: float = maxf(60.0, max_span * 1.1)
    overview_camera.position = Vector3(center_x, overview_height, center_z + max_span * 0.2)
    add_child(overview_camera)
    overview_camera.look_at(Vector3(center_x, 0.0, center_z), Vector3.UP)
    overview_camera.current = true
    camera_mode_overview = true

    var terrain_mesh = BoxMesh.new()
    terrain_mesh.size = Vector3(300.0, 0.1, 300.0)
    var terrain_inst = MeshInstance3D.new()
    terrain_inst.mesh = terrain_mesh
    var terrain_mat = StandardMaterial3D.new()
    terrain_mat.albedo_color = Color(0.24, 0.42, 0.28, 1.0)
    terrain_mat.roughness = 0.9
    terrain_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
    terrain_inst.material_override = terrain_mat
    terrain_inst.position = Vector3(center_x, -0.05, center_z)
    road_root.add_child(terrain_inst)

    var junction_mat = StandardMaterial3D.new()
    junction_mat.albedo_color = Color(0.16, 0.18, 0.20, 1.0)
    junction_mat.roughness = 0.8
    junction_mat.cull_mode = BaseMaterial3D.CULL_DISABLED

    for node in network.get_nodes():
        var pos: Vector2 = node.position_xz
        var cyl = MeshInstance3D.new()
        var cyl_mesh = CylinderMesh.new()
        cyl_mesh.top_radius = RoadDimensions.ROAD_WIDTH_M * 0.5 + 0.3
        cyl_mesh.bottom_radius = RoadDimensions.ROAD_WIDTH_M * 0.5 + 0.3
        cyl_mesh.height = 0.02
        cyl.mesh = cyl_mesh
        cyl.material_override = junction_mat
        cyl.position = Vector3(pos.x, 0.015, pos.y)
        road_root.add_child(cyl)

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

        var road_mesh = _make_route_strip_mesh(
            points,
            0.0,
            RoadDimensions.ROAD_WIDTH_M,
            0.025
        )
        var road_instance = MeshInstance3D.new()
        road_instance.mesh = road_mesh
        road_instance.material_override = asphalt_mat
        road_root.add_child(road_instance)

        var center_marking = _make_route_strip_mesh(points, 0.0, 0.15, 0.035)
        var center_inst = MeshInstance3D.new()
        center_inst.mesh = center_marking
        center_inst.material_override = marking_mat
        road_root.add_child(center_inst)

    for station in network.get_stations():
        var pos: Vector2 = station.stop_position_xz
        var cyl = MeshInstance3D.new()
        var cyl_mesh = CylinderMesh.new()
        cyl_mesh.top_radius = 1.4
        cyl_mesh.bottom_radius = 1.4
        cyl_mesh.height = 0.15
        var cyl_mat = StandardMaterial3D.new()
        cyl_mat.albedo_color = Color(0.1, 0.65, 0.95, 1.0)
        cyl_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
        cyl_mesh.material = cyl_mat
        cyl.mesh = cyl_mesh
        cyl.position = Vector3(pos.x, 0.05, pos.y)
        station_root.add_child(cyl)

        var label = Label3D.new()
        label.text = str(station.display_name).replace(" Stop", "")
        label.position = Vector3(pos.x + 2.5, 3.2, pos.y + 2.5)
        label.font_size = 32
        label.outline_size = 8
        label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
        station_root.add_child(label)

    _build_campus_scenery()

func _build_campus_scenery() -> void:
    var scenery_root = Node3D.new()
    scenery_root.name = "CampusScenery"
    road_root.add_child(scenery_root)

    var bldgs = [
        {"pos": Vector3(0.0, 3.0, -22.0), "size": Vector3(14.0, 6.0, 8.0), "color": Color(0.72, 0.76, 0.82, 1.0), "name": "North Gate Admin"},
        {"pos": Vector3(68.0, 3.5, 68.0), "size": Vector3(16.0, 7.0, 12.0), "color": Color(0.82, 0.68, 0.52, 1.0), "name": "University Library"},
        {"pos": Vector3(-16.0, 3.2, 108.0), "size": Vector3(18.0, 6.0, 14.0), "color": Color(0.52, 0.62, 0.76, 1.0), "name": "Sports Hall"},
        {"pos": Vector3(-68.0, 4.0, 68.0), "size": Vector3(14.0, 8.0, 12.0), "color": Color(0.42, 0.72, 0.86, 1.0), "name": "Innovation Hub"}
    ]

    for b in bldgs:
        var box = MeshInstance3D.new()
        var bmesh = BoxMesh.new()
        bmesh.size = b.size
        box.mesh = bmesh
        var bmat = StandardMaterial3D.new()
        bmat.albedo_color = b.color
        bmat.roughness = 0.6
        bmat.cull_mode = BaseMaterial3D.CULL_DISABLED
        box.material_override = bmat
        box.position = b.pos
        scenery_root.add_child(box)

        var blabel = Label3D.new()
        blabel.text = b.name
        blabel.position = Vector3(b.pos.x, b.pos.y + (b.size.y * 0.5) + 1.0, b.pos.z)
        blabel.font_size = 28
        blabel.outline_size = 6
        blabel.billboard = BaseMaterial3D.BILLBOARD_ENABLED
        scenery_root.add_child(blabel)

    for station in network.get_stations():
        var pos: Vector2 = station.stop_position_xz
        var shelter_offset_m: float = RoadDimensions.ROAD_WIDTH_M * 0.5 + 2.2
        var shelter_pos = Vector3(
            pos.x + shelter_offset_m,
            0.0,
            pos.y + shelter_offset_m
        )

        var roof = MeshInstance3D.new()
        var rmesh = BoxMesh.new()
        rmesh.size = Vector3(3.2, 0.18, 2.2)
        roof.mesh = rmesh
        var rmat = StandardMaterial3D.new()
        rmat.albedo_color = Color(0.22, 0.28, 0.35, 1.0)
        rmat.cull_mode = BaseMaterial3D.CULL_DISABLED
        roof.material_override = rmat
        roof.position = shelter_pos + Vector3(0.0, 2.4, 0.0)
        scenery_root.add_child(roof)

        for dx in [-1.4, 1.4]:
            for dz in [-0.9, 0.9]:
                var post = MeshInstance3D.new()
                var pmesh = CylinderMesh.new()
                pmesh.top_radius = 0.06
                pmesh.bottom_radius = 0.06
                pmesh.height = 2.4
                post.mesh = pmesh
                var pmat = StandardMaterial3D.new()
                pmat.albedo_color = Color(0.15, 0.15, 0.15, 1.0)
                pmat.cull_mode = BaseMaterial3D.CULL_DISABLED
                post.material_override = pmat
                post.position = shelter_pos + Vector3(dx, 1.2, dz)
                scenery_root.add_child(post)

        var bench = MeshInstance3D.new()
        var emesh = BoxMesh.new()
        emesh.size = Vector3(2.0, 0.4, 0.6)
        bench.mesh = emesh
        var emat = StandardMaterial3D.new()
        emat.albedo_color = Color(0.55, 0.35, 0.2, 1.0)
        emat.cull_mode = BaseMaterial3D.CULL_DISABLED
        bench.material_override = emat
        bench.position = shelter_pos + Vector3(0.0, 0.2, 0.0)
        scenery_root.add_child(bench)

        var walk = MeshInstance3D.new()
        var wmesh = BoxMesh.new()
        wmesh.size = Vector3(1.4, 0.02, 3.5)
        walk.mesh = wmesh
        var wmat = StandardMaterial3D.new()
        wmat.albedo_color = Color(0.6, 0.62, 0.65, 1.0)
        wmat.cull_mode = BaseMaterial3D.CULL_DISABLED
        walk.material_override = wmat
        walk.position = Vector3(
            pos.x + shelter_offset_m * 0.5,
            0.02,
            pos.y + shelter_offset_m * 0.5
        )
        scenery_root.add_child(walk)

    var tree_positions = [
        Vector2(-30, -30), Vector2(30, -30), Vector2(80, 20), Vector2(80, 80),
        Vector2(20, 115), Vector2(-20, 115), Vector2(-80, 80), Vector2(-80, 20),
        Vector2(0, 50), Vector2(50, 0), Vector2(0, 130), Vector2(-50, 95)
    ]

    for tp in tree_positions:
        var tree_node = Node3D.new()
        tree_node.position = Vector3(tp.x, 0.0, tp.y)

        var trunk = MeshInstance3D.new()
        var tmesh = CylinderMesh.new()
        tmesh.top_radius = 0.3
        tmesh.bottom_radius = 0.35
        tmesh.height = 2.2
        trunk.mesh = tmesh
        var tmat = StandardMaterial3D.new()
        tmat.albedo_color = Color(0.42, 0.28, 0.18, 1.0)
        tmat.cull_mode = BaseMaterial3D.CULL_DISABLED
        trunk.material_override = tmat
        trunk.position = Vector3(0.0, 1.1, 0.0)
        tree_node.add_child(trunk)

        var foliage = MeshInstance3D.new()
        var fmesh = SphereMesh.new()
        fmesh.radius = 1.8
        fmesh.height = 3.0
        foliage.mesh = fmesh
        var fmat = StandardMaterial3D.new()
        fmat.albedo_color = Color(0.22, 0.52, 0.28, 1.0)
        fmat.cull_mode = BaseMaterial3D.CULL_DISABLED
        foliage.material_override = fmat
        foliage.position = Vector3(0.0, 3.0, 0.0)
        tree_node.add_child(foliage)

        scenery_root.add_child(tree_node)

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
        active_mat.albedo_color = Color(1.0, 0.65, 0.15, 0.8)
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
        active_route_mesh_instance.mesh = _make_route_strip_mesh(combined_points, 0.0, 0.5, 0.045)
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
    if service_coordinator != null:
        var snapshot: Dictionary = service_coordinator.get_service_snapshot()
        var state = str(snapshot.get("service_state", "IDLE"))
        if state != "IDLE":
            status_str = "SERVICE: " + state
        if bool(snapshot.get("lane_change_active", false)):
            status_str += " | CHANGING LANE"
        elif bool(snapshot.get("no_way_hold_active", false)):
            status_str += " | NO CLEAR PATH - WAITING"
        elif bool(snapshot.get("obstacle_hold", false)):
            status_str += " | OBSTACLE AHEAD - HOLDING"
    var cam_str = "OVERVIEW" if camera_mode_overview else "FOLLOW"
    var total_len: float = navigator.total_length_m if navigator else 0.0
    var text = (
        "EASY DRIVE â€” PASSENGER SERVICE (5B.1)\n"
        + "Status: %s | Cam: %s\n"
        + "CONTROLS: [1]-[3] Manual Routes (Idle only) | [C] Cam | [Space] Pause | [R] Reset"
    ) % [
        status_str,
        cam_str
    ]
    hud_label.text = text
