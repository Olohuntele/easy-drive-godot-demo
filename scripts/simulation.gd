extends Node3D

const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")
const DrivingController = preload("res://scripts/driving_controller.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const RouteFollowingController = preload("res://scripts/route_following_controller.gd")
const CampusServiceCoordinator = preload("res://scripts/campus_service_coordinator.gd")
const CampusRouteMap = preload("res://scripts/campus_route_map.gd")
const PassengerBookingPanel = preload("res://scripts/passenger_booking_panel.gd")
const RoadDimensions = preload("res://scripts/road_dimensions.gd")

const ROUTE_LEFT: String = "LEFT"
const ROUTE_RIGHT: String = "RIGHT"
const ROUTE_S_CURVE: String = "S-CURVE"
const ROUTE_ROAD_WIDTH_M: float = RoadDimensions.ROAD_WIDTH_M
const ROUTE_BOUNDARY_OFFSET_M: float = ROUTE_ROAD_WIDTH_M * 0.5 - 0.18
const ROUTE_BOUNDARY_WIDTH_M: float = 0.16
const ROUTE_CENTERLINE_WIDTH_M: float = 0.12
const MODE4_CAMERA_POSITION_RESPONSE: float = 8.0
const MODE4_CAMERA_HEADING_RESPONSE: float = 6.0
const MODE4_CAMERA_FOLLOW_DISTANCE_M: float = 10.0
const MODE4_CAMERA_LOOKAHEAD_M: float = 15.0

@export var ego_max_accel: float = 3.5
@export var ego_max_brake: float = 6.0
@export var ego_max_speed: float = 10.0

var vehicle_dynamics = VehicleDynamics.new()
var driving_controller = DrivingController.new()
var route_navigator = RouteNavigator.new()
var route_following_controller = RouteFollowingController.new()
var lead_pos: float = 60.0
var lead_speed: float = 0.0

var target_b: float = 100.0
var scenario_blocked_road: bool = true
var steering_test_mode: bool = false
var autonomous_route_mode: bool = false
var passenger_service_mode: bool = false
var route_navigation_ready: bool = false
var route_variant: String = ROUTE_RIGHT
var route_visual_centerline_xz: PackedVector2Array = PackedVector2Array()
var key3_pressed_last_frame: bool = false
var key4_pressed_last_frame: bool = false
var key5_pressed_last_frame: bool = false
var key6_pressed_last_frame: bool = false
var key7_pressed_last_frame: bool = false
var space_pressed_last_frame: bool = false
var service_booking_key_pressed_last_frame: bool = false
var service_boarding_key_pressed_last_frame: bool = false
var service_exit_key_pressed_last_frame: bool = false
var service_reset_key_pressed_last_frame: bool = false
var service_key1_pressed_last_frame: bool = false
var service_key2_pressed_last_frame: bool = false
var ego_length: float = 4.0
var lead_length: float = 4.0

@onready var ego_car: Node3D = $EgoCar
@onready var lead_car: Node3D = $LeadCar
@onready var follow_camera: Camera3D = $FollowCamera
@onready var hud_label: Label = $CanvasLayer/TelemetryPanel/HUD
@onready var straight_road: MeshInstance3D = $Road
@onready var straight_centerline: MeshInstance3D = $CenterLine
@onready var straight_roadside_scenery: Node3D = $RoadsideScenery
@onready var canvas_layer: CanvasLayer = $CanvasLayer

var route_visual_root: Node3D
var route_surface_mesh: ArrayMesh
var route_left_boundary_mesh: ArrayMesh
var route_right_boundary_mesh: ArrayMesh
var route_centerline_mesh: ArrayMesh
var mode4_camera_heading_rad: float = 0.0
var mode4_camera_initialized: bool = false
var service_coordinator: RefCounted
var service_road_root: Node3D
var service_active_route: MeshInstance3D
var service_route_map: Control
var booking_panel
var service_visual_leg: String = ""
var service_mode_message: String = ""

func _ready() -> void:
    print(
        "[MODE5 DIAG] Simulation startup: scene='%s' script='%s'"
            % [get_scene_file_path(), get_script().resource_path]
    )
    vehicle_dynamics.max_acceleration = ego_max_accel
    vehicle_dynamics.max_braking = ego_max_brake
    vehicle_dynamics.max_speed = ego_max_speed
    reset_simulation()

func reset_simulation() -> void:
    driving_controller.reset()
    route_following_controller.reset()
    if autonomous_route_mode:
        route_navigation_ready = route_navigator.set_route(_route_waypoints())
        if route_navigation_ready:
            var samples: PackedVector2Array = route_navigator.get_centerline_samples()
            _update_route_visuals(samples)
            var initial_tangent: Vector2 = samples[1] - samples[0]
            var initial_heading: float = atan2(initial_tangent.x, initial_tangent.y)
            vehicle_dynamics.reset(
                samples[0].y,
                0.0,
                samples[0].x,
                initial_heading
            )
            _update_mode4_camera(
                Vector3(samples[0].x, 0.4, samples[0].y),
                initial_heading,
                0.0,
                true
            )
        else:
            push_error("Could not initialize the autonomous route-following route.")
            vehicle_dynamics.reset()
    else:
        route_navigation_ready = false
        route_navigator.clear_route()
        route_visual_centerline_xz = PackedVector2Array()
        if route_visual_root:
            route_visual_root.visible = false
        if straight_road:
            straight_road.visible = true
        if straight_centerline:
            straight_centerline.visible = true
        if straight_roadside_scenery:
            straight_roadside_scenery.visible = true
        vehicle_dynamics.reset()
    if lead_car:
        lead_car.visible = scenario_blocked_road && not steering_test_mode && not autonomous_route_mode
    if scenario_blocked_road:
        lead_pos = 60.0
        lead_speed = 0.0
    else:
        lead_pos = 999.0
        lead_speed = 0.0

func select_blocked_road_mode() -> void:
    if not _can_leave_passenger_service():
        return
    _leave_passenger_service_mode()
    steering_test_mode = false
    autonomous_route_mode = false
    scenario_blocked_road = true
    reset_simulation()

func select_clear_road_mode() -> void:
    if not _can_leave_passenger_service():
        return
    _leave_passenger_service_mode()
    steering_test_mode = false
    autonomous_route_mode = false
    scenario_blocked_road = false
    reset_simulation()

func toggle_steering_test_mode() -> void:
    if not _can_leave_passenger_service():
        return
    _leave_passenger_service_mode()
    steering_test_mode = not steering_test_mode
    autonomous_route_mode = false
    reset_simulation()

func select_route_following_mode() -> void:
    if not _can_leave_passenger_service():
        return
    _leave_passenger_service_mode()
    steering_test_mode = false
    autonomous_route_mode = true
    scenario_blocked_road = false
    reset_simulation()

func select_route_variant(variant_name: String) -> void:
    if not autonomous_route_mode:
        return
    if variant_name != ROUTE_LEFT && variant_name != ROUTE_RIGHT && variant_name != ROUTE_S_CURVE:
        push_error("Unsupported autonomous route variant: " + variant_name)
        return
    route_variant = variant_name
    reset_simulation()

func select_passenger_service_mode() -> Dictionary:
    print(
        "[MODE5 DIAG] Entry request: active=%s position=(%.3f, %.3f) speed=%.3f m/s"
            % [
                passenger_service_mode,
                vehicle_dynamics.position_x_m,
                vehicle_dynamics.position_z_m,
                vehicle_dynamics.speed_mps
            ]
    )
    if passenger_service_mode:
        print("[MODE5 DIAG] Entry request ignored: Mode 5 is already active.")
        return {"valid": true, "failure_reason": ""}
    if service_coordinator != null && not _can_leave_passenger_service():
        print("[MODE5 DIAG] Entry rejected: %s" % service_mode_message)
        return {"valid": false, "failure_reason": service_mode_message}
    var coordinator = CampusServiceCoordinator.new()
    var network: RefCounted = coordinator.network
    var service_stop: Dictionary = network.get_station_stop("north_gate")
    if not service_stop.get("valid", false):
        service_mode_message = "Mode 5 could not find the North Gate service station."
        push_warning("Mode 5 entry rejected: " + service_mode_message)
        print("[MODE5 DIAG] Entry rejected: " + service_mode_message)
        _update_legacy_hud_message()
        return {"valid": false, "failure_reason": "service_station_unavailable"}
    var service_start_position: Vector2 = service_stop.position_xz

    var launch_segment: Dictionary = network.get_segment("north_to_east")
    var launch_points: Array = launch_segment.get("centerline_points", [])
    if launch_points.size() < 2:
        service_mode_message = "Mode 5 could not determine the North Gate road heading."
        push_warning("Mode 5 entry rejected: " + service_mode_message)
        print("[MODE5 DIAG] Entry rejected: " + service_mode_message)
        _update_legacy_hud_message()
        return {"valid": false, "failure_reason": "service_launch_tangent_unavailable"}
    var launch_tangent: Vector2 = launch_points[1] - launch_points[0]
    if launch_tangent.length_squared() <= 0.000001:
        service_mode_message = "Mode 5 could not determine the North Gate road heading."
        push_warning("Mode 5 entry rejected: " + service_mode_message)
        print("[MODE5 DIAG] Entry rejected: " + service_mode_message)
        _update_legacy_hud_message()
        return {"valid": false, "failure_reason": "service_launch_tangent_unavailable"}
    var launch_heading_rad: float = atan2(launch_tangent.x, launch_tangent.y)
    coordinator.dynamics.max_acceleration = vehicle_dynamics.max_acceleration
    coordinator.dynamics.max_braking = vehicle_dynamics.max_braking
    coordinator.dynamics.max_speed = vehicle_dynamics.max_speed
    var initialized: Dictionary = coordinator.initialize_service(
        "north_gate",
        service_start_position,
        launch_heading_rad
    )
    if not initialized.get("valid", false):
        service_mode_message = "Mode 5 could not initialize: %s" % initialized.get(
            "failure_reason",
            "unknown initialization error"
        )
        push_warning("Mode 5 entry rejected: " + service_mode_message)
        print("[MODE5 DIAG] Entry rejected: " + service_mode_message)
        _update_legacy_hud_message()
        return initialized

    service_coordinator = coordinator
    passenger_service_mode = true
    steering_test_mode = false
    autonomous_route_mode = false
    service_mode_message = ""
    var visuals_ready: bool = _ensure_service_visuals(service_coordinator.network)
    if booking_panel:
        booking_panel.visible = true
    if straight_road:
        straight_road.visible = false
    if straight_centerline:
        straight_centerline.visible = false
    if straight_roadside_scenery:
        straight_roadside_scenery.visible = false
    if route_visual_root:
        route_visual_root.visible = false
    if lead_car:
        lead_car.visible = false
    var pose := Vector3(service_start_position.x, 0.4, service_start_position.y)
    ego_car.position = pose
    ego_car.rotation.y = launch_heading_rad
    _update_mode4_camera(pose, launch_heading_rad, 0.0, true)
    _update_passenger_service_hud(service_coordinator.get_service_snapshot())
    if visuals_ready:
        print(
            "[MODE5 DIAG] Visual initialization success: roads=%d stations=%d map=%s HUD=%s"
                % [
                    service_coordinator.network.get_segment_ids().size(),
                    service_coordinator.network.get_station_ids().size(),
                    service_route_map != null,
                    hud_label != null
                ]
        )
    else:
        push_error("[MODE5 DIAG] Visual initialization failure: required scene visuals are missing.")
    print(
        "[MODE5 DIAG] Mode 5 accepted: state=%s service_station=%s"
            % [service_coordinator.journey.state, service_coordinator.service_station_id]
    )
    return {"valid": true, "failure_reason": ""}

func request_service_demo_booking() -> Dictionary:
    return request_passenger_booking("sports_hall", "innovation_centre")

func request_passenger_booking(
    pickup_station_id: String,
    destination_station_id: String
) -> Dictionary:
    if not passenger_service_mode || service_coordinator == null:
        return {"valid": false, "failure_reason": "passenger_service_mode_not_active"}
    var result: Dictionary = service_coordinator.request_booking(
        pickup_station_id,
        destination_station_id
    )
    service_mode_message = (
        "Booking accepted."
        if result.get("valid", false)
        else "Booking rejected: %s" % _humanize_failure(str(result.get("failure_reason", "")))
    )
    var snapshot: Dictionary = service_coordinator.get_service_snapshot()
    _sync_passenger_service_visuals(snapshot)
    _update_passenger_service_hud(snapshot)
    if booking_panel:
        booking_panel.show_booking_result(result, snapshot)
    return result

func confirm_service_demo_boarding() -> Dictionary:
    if not passenger_service_mode || service_coordinator == null:
        return {"valid": false, "failure_reason": "passenger_service_mode_not_active"}
    var result: Dictionary = service_coordinator.confirm_boarding()
    service_mode_message = (
        "Boarding confirmed."
        if result.get("valid", false)
        else "Boarding not accepted: %s" % _humanize_failure(
            str(result.get("failure_reason", ""))
        )
    )
    var snapshot: Dictionary = service_coordinator.get_service_snapshot()
    _sync_passenger_service_visuals(snapshot)
    _update_passenger_service_hud(snapshot)
    if booking_panel:
        booking_panel.show_action_result("boarding", result, snapshot)
    return result

func confirm_service_demo_exit() -> Dictionary:
    if not passenger_service_mode || service_coordinator == null:
        return {"valid": false, "failure_reason": "passenger_service_mode_not_active"}
    var result: Dictionary = service_coordinator.confirm_passenger_exit()
    service_mode_message = (
        "Passenger exit confirmed."
        if result.get("valid", false)
        else "Exit not accepted: %s" % _humanize_failure(
            str(result.get("failure_reason", ""))
        )
    )
    var snapshot: Dictionary = service_coordinator.get_service_snapshot()
    _sync_passenger_service_visuals(snapshot)
    _update_passenger_service_hud(snapshot)
    if booking_panel:
        booking_panel.show_action_result("exit", result, snapshot)
    return result

func restart_service_demo() -> Dictionary:
    if not passenger_service_mode || service_coordinator == null:
        return {"valid": false, "failure_reason": "passenger_service_mode_not_active"}
    var snapshot: Dictionary = service_coordinator.get_service_snapshot()
    if snapshot.service_state != "IDLE" \
        || service_coordinator.dispatcher.has_active_booking():
        service_mode_message = (
            "Restart blocked: finish the passenger journey first. "
            + "An occupied or blocked booking is never discarded."
        )
        return {"valid": false, "failure_reason": "active_booking_cannot_be_discarded"}
    if snapshot.speed_mps > 0.1 \
        || snapshot.position_xz.distance_to(
            service_coordinator.network.get_station_stop("north_gate").position_xz
        ) > 1.0:
        service_mode_message = "Restart blocked: shuttle must be stopped at North Gate."
        return {"valid": false, "failure_reason": "restart_pose_not_at_service_station"}

    var replacement = CampusServiceCoordinator.new()
    replacement.dynamics.max_acceleration = service_coordinator.dynamics.max_acceleration
    replacement.dynamics.max_braking = service_coordinator.dynamics.max_braking
    replacement.dynamics.max_speed = service_coordinator.dynamics.max_speed
    var initialized: Dictionary = replacement.initialize_service(
        "north_gate",
        snapshot.position_xz,
        snapshot.heading_rad
    )
    if not initialized.get("valid", false):
        service_mode_message = "Restart failed: %s" % str(initialized.get("failure_reason", ""))
        return initialized
    service_coordinator = replacement
    service_visual_leg = ""
    service_mode_message = "Demo safely reinitialized at North Gate; no active booking was discarded."
    _sync_passenger_service_visuals(service_coordinator.get_service_snapshot())
    _update_passenger_service_hud(service_coordinator.get_service_snapshot())
    return {"valid": true, "failure_reason": ""}

func _can_leave_passenger_service() -> bool:
    if not passenger_service_mode:
        return true
    if service_coordinator == null:
        service_mode_message = "Mode switch blocked: passenger service is not initialized."
        return false
    var snapshot: Dictionary = service_coordinator.get_service_snapshot()
    if snapshot.service_state != "IDLE" \
        || service_coordinator.dispatcher.has_active_booking():
        service_mode_message = (
            "Mode switch blocked: complete the passenger journey and return to North Gate first."
        )
        return false
    return true

func _leave_passenger_service_mode() -> void:
    if not passenger_service_mode:
        return
    if booking_panel && booking_panel.has_keyboard_focus():
        get_viewport().gui_release_focus()
    passenger_service_mode = false
    service_mode_message = ""
    if service_road_root:
        service_road_root.visible = false
    if service_route_map:
        service_route_map.visible = false
    if booking_panel:
        booking_panel.visible = false

func _update_legacy_hud_message() -> void:
    if not hud_label || service_mode_message.is_empty():
        return
    if not hud_label.text.contains(service_mode_message):
        hud_label.text += "\n" + service_mode_message

func _physics_process_passenger_service(delta: float) -> void:
    if booking_panel && booking_panel.visible && booking_panel.has_keyboard_focus():
        _advance_passenger_service(delta)
        return

    var key1_current: bool = Input.is_key_pressed(KEY_1)
    if key1_current && not service_key1_pressed_last_frame:
        if _can_leave_passenger_service():
            _leave_passenger_service_mode()
            select_blocked_road_mode()
            return
    service_key1_pressed_last_frame = key1_current

    var key2_current: bool = Input.is_key_pressed(KEY_2)
    if key2_current && not service_key2_pressed_last_frame:
        if _can_leave_passenger_service():
            _leave_passenger_service_mode()
            select_clear_road_mode()
            return
    service_key2_pressed_last_frame = key2_current

    var key3_current: bool = Input.is_key_pressed(KEY_3)
    if key3_current && not key3_pressed_last_frame:
        if _can_leave_passenger_service():
            _leave_passenger_service_mode()
            toggle_steering_test_mode()
            return
    key3_pressed_last_frame = key3_current

    var key4_current: bool = Input.is_key_pressed(KEY_4)
    if key4_press_edge(key4_current):
        if _can_leave_passenger_service():
            _leave_passenger_service_mode()
            select_route_following_mode()
            return

    var booking_key_current: bool = Input.is_key_pressed(KEY_B)
    if booking_key_current && not service_booking_key_pressed_last_frame:
        request_service_demo_booking()
    service_booking_key_pressed_last_frame = booking_key_current

    var boarding_key_current: bool = Input.is_key_pressed(KEY_E)
    if boarding_key_current && not service_boarding_key_pressed_last_frame:
        confirm_service_demo_boarding()
    service_boarding_key_pressed_last_frame = boarding_key_current

    var exit_key_current: bool = Input.is_key_pressed(KEY_X)
    if exit_key_current && not service_exit_key_pressed_last_frame:
        confirm_service_demo_exit()
    service_exit_key_pressed_last_frame = exit_key_current

    var reset_key_current: bool = Input.is_key_pressed(KEY_R)
    if reset_key_current && not service_reset_key_pressed_last_frame:
        restart_service_demo()
    service_reset_key_pressed_last_frame = reset_key_current

    _advance_passenger_service(delta)

func _advance_passenger_service(delta: float, physics_tick_id: int = -1) -> Dictionary:
    if not passenger_service_mode || service_coordinator == null:
        return {"valid": false, "failure_reason": "passenger_service_mode_not_active"}
    var result: Dictionary = service_coordinator.step(delta, physics_tick_id)
    var snapshot: Dictionary = service_coordinator.get_service_snapshot()
    _sync_passenger_service_visuals(snapshot, delta)
    _update_passenger_service_hud(snapshot)
    return result

func _ensure_service_visuals(network: RefCounted) -> bool:
    print("[MODE5 DIAG] Visual initialization started.")
    if service_road_root == null:
        service_road_root = Node3D.new()
        service_road_root.name = "CampusServiceRoadNetwork"
        add_child(service_road_root)

        var road_material := _route_material(Color(0.16, 0.18, 0.2, 1.0))
        var road_navigator = RouteNavigator.new()
        for segment in network.get_segments():
            var segment_points: Array[Vector2] = []
            for point in segment.centerline_points:
                segment_points.append(point)
            if not road_navigator.set_route(segment_points, 12):
                push_error("Could not render campus segment: " + str(segment.id))
                continue
            var road_instance := MeshInstance3D.new()
            road_instance.name = "Road_" + str(segment.id)
            road_instance.mesh = _make_route_strip_mesh(
                road_navigator.get_centerline_samples(),
                0.0,
                7.0,
                0.035
            )
            road_instance.material_override = road_material
            service_road_root.add_child(road_instance)

        service_active_route = MeshInstance3D.new()
        service_active_route.name = "ActivePassengerRoute"
        service_active_route.material_override = _route_material(Color(1.0, 0.62, 0.12, 1.0))
        service_road_root.add_child(service_active_route)

        var marker_material := _route_material(Color(0.08, 0.66, 0.84, 1.0))
        for station in network.get_stations():
            var marker_mesh := CylinderMesh.new()
            marker_mesh.top_radius = 1.8
            marker_mesh.bottom_radius = 1.8
            marker_mesh.height = 0.16
            var marker := MeshInstance3D.new()
            marker.name = "StationMarker_" + str(station.id)
            marker.mesh = marker_mesh
            marker.material_override = marker_material
            marker.position = Vector3(
                station.stop_position_xz.x,
                0.12,
                station.stop_position_xz.y
            )
            service_road_root.add_child(marker)

            var station_label := Label3D.new()
            station_label.name = "StationLabel_" + str(station.id)
            station_label.text = str(station.display_name).replace(" Stop", "")
            station_label.position = Vector3(
                station.stop_position_xz.x,
                1.5,
                station.stop_position_xz.y
            )
            station_label.font_size = 36
            station_label.outline_size = 8
            station_label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
            service_road_root.add_child(station_label)

    service_road_root.visible = true
    if service_route_map == null:
        service_route_map = CampusRouteMap.new()
        service_route_map.name = "CampusRouteMap"
        canvas_layer.add_child(service_route_map)
    service_route_map.configure(network)
    if booking_panel == null:
        booking_panel = PassengerBookingPanel.new()
        booking_panel.name = "PassengerBookingPanel"
        canvas_layer.add_child(booking_panel)
        booking_panel.configure(network, service_coordinator.get_service_snapshot())
        booking_panel.booking_requested.connect(_on_passenger_booking_requested)
        booking_panel.boarding_requested.connect(_on_passenger_boarding_requested)
        booking_panel.passenger_exit_requested.connect(_on_passenger_exit_requested)
    booking_panel.visible = false
    var road_count: int = 0
    var station_label_count: int = 0
    for child in service_road_root.get_children():
        if child.name.begins_with("Road_"):
            road_count += 1
        elif child.name.begins_with("StationLabel_"):
            station_label_count += 1
    var visuals_ready: bool = road_count == network.get_segment_ids().size() \
        && station_label_count == network.get_station_ids().size() \
        && service_active_route != null \
        && service_route_map != null
    if not visuals_ready:
        push_error(
            "[MODE5 DIAG] Visual initialization failure: roads=%d/%d stations=%d/%d route=%s map=%s"
                % [
                    road_count,
                    network.get_segment_ids().size(),
                    station_label_count,
                    network.get_station_ids().size(),
                    service_active_route != null,
                    service_route_map != null
                ]
        )
    return visuals_ready

func _sync_passenger_service_visuals(snapshot: Dictionary, delta: float = 0.0) -> void:
    if not passenger_service_mode || service_coordinator == null:
        return
    if ego_car:
        var position_xz: Vector2 = snapshot.position_xz
        ego_car.position = Vector3(position_xz.x, 0.4, position_xz.y)
        ego_car.rotation.y = float(snapshot.heading_rad)
    if follow_camera:
        var position := Vector3(
            snapshot.position_xz.x,
            0.4,
            snapshot.position_xz.y
        )
        _update_mode4_camera(position, float(snapshot.heading_rad), delta)

    var active_leg: String = str(snapshot.current_leg)
    if active_leg != service_visual_leg:
        service_visual_leg = active_leg
        var route_samples := PackedVector2Array()
        if not active_leg.is_empty() && service_coordinator.navigator.route_valid:
            route_samples = service_coordinator.navigator.get_centerline_samples()
        if service_active_route:
            service_active_route.mesh = _make_route_strip_mesh(
                route_samples,
                0.0,
                0.55,
                0.11
            ) if route_samples.size() >= 2 else null
        if service_route_map:
            service_route_map.set_active_route(route_samples)

func _update_passenger_service_hud(snapshot: Dictionary) -> void:
    if not hud_label:
        return
    if booking_panel:
        booking_panel.update_service_snapshot(snapshot)
    var booking: Dictionary = snapshot.get("active_booking", {})
    var pickup_name: String = "—"
    var destination_name: String = "—"
    if not booking.is_empty():
        pickup_name = _station_display_name(
            service_coordinator.network,
            str(booking.get("origin_station_id", ""))
        )
        destination_name = _station_display_name(
            service_coordinator.network,
            str(booking.get("destination_station_id", ""))
        )
    var service_status: String
    if snapshot.get("blocked", false):
        service_status = "BLOCKED — reservation retained"
    elif not booking.is_empty():
        service_status = "Occupied"
    else:
        service_status = "Available"

    var journey_state: String = str(snapshot.get("service_state", "UNINITIALIZED"))
    var state_label: String = _humanize_service_state(journey_state)
    var waiting_text: String = ""
    if journey_state == "BOARDING":
        waiting_text = "Waiting for boarding confirmation at %s." % pickup_name
    elif journey_state == "DROPOFF":
        waiting_text = "Waiting for passenger exit confirmation at %s." % destination_name
    elif journey_state == "BLOCKED":
        waiting_text = "Journey blocked: %s" % _humanize_failure(
            str(snapshot.get("blocked_reason", ""))
        )

    var arrival_text: String = "No station arrival certified."
    if journey_state == "BOARDING":
        arrival_text = "%s arrival certified." % pickup_name
    elif journey_state == "DROPOFF":
        arrival_text = "%s arrival certified." % destination_name
    elif journey_state == "IDLE" && snapshot.get("arrival_certified", false):
        arrival_text = "Return to North Gate certified."

    var route_leg: String = str(snapshot.get("current_leg", ""))
    route_leg = "Waiting" if route_leg.is_empty() else _humanize_route_leg(route_leg)
    var target_id: String = str(snapshot.get("destination_station_id", ""))
    var target_name: String = (
        _station_display_name(service_coordinator.network, target_id)
        if not target_id.is_empty()
        else "—"
    )
    hud_label.text = (
        "[MODE 5 — PASSENGER SERVICE DEMO]\n"
        + "Service: %s | Journey: %s\n" % [service_status, state_label]
        + "Booking: %s → %s\n" % [pickup_name, destination_name]
        + "Route leg: %s | Target: %s\n" % [route_leg, target_name]
        + "Shuttle speed: %.2f m/s\n" % float(snapshot.get("speed_mps", 0.0))
        + "%s\n" % arrival_text
        + "%s\n" % waiting_text
        + "Legacy demo is suspended; only the service shuttle advances.\n"
        + "Controls: [B] Request demo | Panel Board/Exit or [E]/[X]\n"
        + "[R] Restart only when available | [1–4] Leave after return | [WASD] disabled"
    )
    if not service_mode_message.is_empty():
        hud_label.text += "\n" + service_mode_message

func _station_display_name(network: RefCounted, station_id: String) -> String:
    if network == null || station_id.is_empty():
        return "—"
    var station: Dictionary = network.get_station(station_id)
    return str(station.get("display_name", station_id)).replace(" Stop", "")

func _humanize_service_state(state: String) -> String:
    match state:
        "IDLE":
            return "Ready / available"
        "ASSIGNED":
            return "Booking accepted"
        "TO_PICKUP":
            return "Driving to pickup"
        "BOARDING":
            return "At pickup — waiting to board"
        "TO_DESTINATION":
            return "Driving to destination"
        "DROPOFF":
            return "At destination — waiting for exit"
        "RETURNING_TO_SERVICE":
            return "Returning to North Gate"
        "BLOCKED":
            return "Blocked — reservation retained"
    return state.replace("_", " ").capitalize()

func _humanize_route_leg(leg: String) -> String:
    match leg:
        "pickup":
            return "Pickup"
        "destination":
            return "Passenger destination"
        "return":
            return "Service return"
    return leg.capitalize()

func _humanize_failure(reason: String) -> String:
    return reason.replace("_", " ").capitalize()

func _route_waypoints() -> Array[Vector2]:
    if route_variant == ROUTE_LEFT:
        var left_waypoints: Array[Vector2] = []
        for point in _right_route_waypoints():
            left_waypoints.append(Vector2(-point.x, point.y))
        return left_waypoints
    if route_variant == ROUTE_S_CURVE:
        return [
            Vector2(0.0, 0.0),
            Vector2(0.0, 18.0),
            Vector2(2.5, 34.0),
            Vector2(8.0, 48.0),
            Vector2(10.0, 62.0),
            Vector2(6.0, 77.0),
            Vector2(-2.0, 92.0),
            Vector2(-8.0, 108.0),
            Vector2(-8.0, 124.0)
        ]
    return _right_route_waypoints()

func _right_route_waypoints() -> Array[Vector2]:
    return [
        Vector2(0.0, 0.0),
        Vector2(0.3, 8.0),
        Vector2(2.2, 20.0),
        Vector2(6.5, 36.0),
        Vector2(15.0, 56.0),
        Vector2(29.0, 76.0),
        Vector2(45.0, 94.0)
    ]

func _update_route_visuals(centerline_samples: PackedVector2Array) -> void:
    route_visual_centerline_xz = centerline_samples.duplicate()
    if not route_visual_root:
        route_visual_root = Node3D.new()
        route_visual_root.name = "GeneratedMode4Road"
        add_child(route_visual_root)

        var surface_instance := MeshInstance3D.new()
        surface_instance.name = "RouteRoadSurface"
        surface_instance.material_override = _route_material(Color(0.13, 0.14, 0.16, 1.0))
        route_visual_root.add_child(surface_instance)

        var left_boundary_instance := MeshInstance3D.new()
        left_boundary_instance.name = "RouteLeftBoundary"
        left_boundary_instance.material_override = _route_material(Color(0.94, 0.94, 0.88, 1.0))
        route_visual_root.add_child(left_boundary_instance)

        var right_boundary_instance := MeshInstance3D.new()
        right_boundary_instance.name = "RouteRightBoundary"
        right_boundary_instance.material_override = _route_material(Color(0.94, 0.94, 0.88, 1.0))
        route_visual_root.add_child(right_boundary_instance)

        var centerline_instance := MeshInstance3D.new()
        centerline_instance.name = "RouteCenterline"
        centerline_instance.material_override = _route_material(Color(1.0, 0.78, 0.16, 1.0))
        route_visual_root.add_child(centerline_instance)

    route_surface_mesh = _make_route_strip_mesh(
        centerline_samples,
        0.0,
        ROUTE_ROAD_WIDTH_M,
        0.055
    )
    route_left_boundary_mesh = _make_route_strip_mesh(
        centerline_samples,
        -ROUTE_BOUNDARY_OFFSET_M,
        ROUTE_BOUNDARY_WIDTH_M,
        0.068
    )
    route_right_boundary_mesh = _make_route_strip_mesh(
        centerline_samples,
        ROUTE_BOUNDARY_OFFSET_M,
        ROUTE_BOUNDARY_WIDTH_M,
        0.068
    )
    route_centerline_mesh = _make_route_strip_mesh(
        centerline_samples,
        0.0,
        ROUTE_CENTERLINE_WIDTH_M,
        0.069
    )
    route_visual_root.get_node("RouteRoadSurface").mesh = route_surface_mesh
    route_visual_root.get_node("RouteLeftBoundary").mesh = route_left_boundary_mesh
    route_visual_root.get_node("RouteRightBoundary").mesh = route_right_boundary_mesh
    route_visual_root.get_node("RouteCenterline").mesh = route_centerline_mesh
    route_visual_root.visible = true
    if straight_road:
        straight_road.visible = false
    if straight_centerline:
        straight_centerline.visible = false
    if straight_roadside_scenery:
        straight_roadside_scenery.visible = false

func _route_material(color: Color) -> StandardMaterial3D:
    var material := StandardMaterial3D.new()
    material.albedo_color = color
    material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
    material.cull_mode = BaseMaterial3D.CULL_DISABLED
    return material

func _make_route_strip_mesh(
    centerline_samples: PackedVector2Array,
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

func key4_press_edge(is_pressed: bool) -> bool:
    var just_pressed: bool = is_pressed && not key4_pressed_last_frame
    key4_pressed_last_frame = is_pressed
    return just_pressed

func _input(event: InputEvent) -> void:
    if event is InputEventKey && event.pressed && not event.echo \
        && event.keycode == KEY_ESCAPE \
        && booking_panel && booking_panel.visible && booking_panel.has_keyboard_focus():
        get_viewport().gui_release_focus()
        get_viewport().set_input_as_handled()
        return
    if not event is InputEventKey || not event.pressed || event.echo:
        return
    var key_event: InputEventKey = event
    if key_event.keycode != KEY_8 \
        && key_event.physical_keycode != KEY_8 \
        && key_event.keycode != KEY_KP_8 \
        && key_event.physical_keycode != KEY_KP_8:
        return
    print(
        "[MODE5 DIAG] Key 8 detected: keycode=%s physical_keycode=%s scene='%s' active=%s"
            % [
                OS.get_keycode_string(key_event.keycode),
                OS.get_keycode_string(key_event.physical_keycode),
                get_scene_file_path(),
                passenger_service_mode
            ]
    )
    if passenger_service_mode:
        print("[MODE5 DIAG] Key 8 ignored: Mode 5 is already active.")
        return
    select_passenger_service_mode()

func _on_passenger_booking_requested(
    pickup_station_id: String,
    destination_station_id: String
) -> void:
    request_passenger_booking(pickup_station_id, destination_station_id)

func _on_passenger_boarding_requested() -> void:
    confirm_service_demo_boarding()

func _on_passenger_exit_requested() -> void:
    confirm_service_demo_exit()

func _physics_process(delta: float) -> void:
    # --- 1. Keyboard Controls & Mode Selection ---
    if passenger_service_mode:
        _physics_process_passenger_service(delta)
        return

    if Input.is_key_pressed(KEY_1):
        select_blocked_road_mode()
    elif Input.is_key_pressed(KEY_2):
        select_clear_road_mode()

    var key3_current: bool = Input.is_key_pressed(KEY_3)
    if key3_current && not key3_pressed_last_frame:
        toggle_steering_test_mode()
    key3_pressed_last_frame = key3_current

    var key4_current: bool = Input.is_key_pressed(KEY_4)
    if key4_press_edge(key4_current):
        select_route_following_mode()

    var key5_current: bool = Input.is_key_pressed(KEY_5)
    if key5_current && not key5_pressed_last_frame:
        select_route_variant(ROUTE_LEFT)
    key5_pressed_last_frame = key5_current

    var key6_current: bool = Input.is_key_pressed(KEY_6)
    if key6_current && not key6_pressed_last_frame:
        select_route_variant(ROUTE_RIGHT)
    key6_pressed_last_frame = key6_current

    var key7_current: bool = Input.is_key_pressed(KEY_7)
    if key7_current && not key7_pressed_last_frame:
        select_route_variant(ROUTE_S_CURVE)
    key7_pressed_last_frame = key7_current

    var space_current: bool = Input.is_key_pressed(KEY_SPACE)
    if Input.is_action_just_pressed("ui_accept") || (space_current && not space_pressed_last_frame):
        driving_controller.auto_drive_enabled = not driving_controller.auto_drive_enabled
    space_pressed_last_frame = space_current

    if Input.is_key_pressed(KEY_R):
        reset_simulation()

    var ego_pos: float = vehicle_dynamics.position_z_m
    var ego_speed: float = vehicle_dynamics.speed_mps
    var requested_acceleration: float = 0.0
    var requested_steering: float = 0.0
    var braking_state: String = "COASTING"
    var obstacle_state_str: String = "CLEAR"
    var manual_input: float = 0.0
    var manual_steering_input: float = 0.0

    if Input.is_action_pressed("ui_up") or Input.is_key_pressed(KEY_W):
        manual_input = 1.0
    elif Input.is_action_pressed("ui_down") or Input.is_key_pressed(KEY_S):
        manual_input = -1.0
    if Input.is_key_pressed(KEY_A):
        manual_steering_input = -vehicle_dynamics.steering_limit_rad
    elif Input.is_key_pressed(KEY_D):
        manual_steering_input = vehicle_dynamics.steering_limit_rad

    var route_decision: Dictionary = {}
    if autonomous_route_mode:
        if route_navigation_ready:
            var manual_intervention: bool = not driving_controller.auto_drive_enabled \
                || manual_input != 0.0 || manual_steering_input != 0.0
            route_decision = route_following_controller.update(
                vehicle_dynamics,
                route_navigator,
                delta,
                manual_intervention
            )
            requested_acceleration = route_decision.requested_acceleration_mps2
            requested_steering = route_decision.requested_steering_rad
            braking_state = route_decision.reason
            obstacle_state_str = "NOT EVALUATED - ROUTE FOLLOWING"
            if route_decision.status != "SAFE STOP / INVALID NAVIGATION" \
                && route_decision.status != "MISSED DESTINATION" \
                && manual_intervention:
                if not driving_controller.auto_drive_enabled:
                    requested_acceleration = manual_input * (
                        ego_max_accel if manual_input > 0.0 else ego_max_brake
                    ) if manual_input != 0.0 else -0.5
                    requested_steering = manual_steering_input
                    braking_state = "MANUAL ROUTE OVERRIDE"
                elif manual_input != 0.0:
                    requested_acceleration = manual_input * (
                        ego_max_accel if manual_input > 0.0 else ego_max_brake
                    )
                    braking_state = "MANUAL ROUTE OVERRIDE"
                if manual_steering_input != 0.0:
                    requested_steering = manual_steering_input
            obstacle_state_str = route_decision.status
        else:
            route_decision = {
                "status": "SAFE STOP / INVALID NAVIGATION",
                "reason": "route_not_initialized",
                "route_progress_m": 0.0,
                "cross_track_error_m": 0.0,
                "remaining_distance_m": INF,
                "endpoint_error_m": INF,
                "requested_acceleration_mps2": -ego_max_brake,
                "requested_steering_rad": 0.0,
                "arrival_certified": false
            }
            requested_acceleration = -ego_max_brake
            requested_steering = 0.0
            braking_state = "SAFE STOP - ROUTE NOT INITIALIZED"
            obstacle_state_str = route_decision.status
    elif steering_test_mode:
        # Manual kinematic demonstration only; no obstacle or AEB evaluation.
        if manual_input > 0.0:
            requested_acceleration = ego_max_accel
            braking_state = "MANUAL ACCEL"
        elif manual_input < 0.0:
            requested_acceleration = -ego_max_brake
            braking_state = "MANUAL BRAKE"

        requested_steering = manual_steering_input
        obstacle_state_str = "NOT EVALUATED"
    elif scenario_blocked_road:
        var blocked_decision: Dictionary = driving_controller.request_blocked_road_acceleration(
            ego_pos,
            ego_speed,
            lead_pos,
            ego_length,
            lead_length,
            delta,
            manual_input,
            ego_max_accel,
            ego_max_brake,
            ego_max_speed
        )
        requested_acceleration = blocked_decision.acceleration
        braking_state = blocked_decision.braking_state
        obstacle_state_str = blocked_decision.get("obstacle_state", "CLEAR")
    else:
        var clear_road_decision: Dictionary = driving_controller.request_clear_road_acceleration(
            ego_pos,
            ego_speed,
            target_b,
            delta,
            manual_input,
            ego_max_accel,
            ego_max_brake,
            ego_max_speed
        )
        requested_acceleration = clear_road_decision.acceleration
        braking_state = clear_road_decision.braking_state
        obstacle_state_str = "CLEAR ROAD"

    vehicle_dynamics.step(requested_acceleration, delta, requested_steering)
    if scenario_blocked_road && not steering_test_mode && not autonomous_route_mode:
        lead_pos += lead_speed * delta

    var state_x: float = vehicle_dynamics.position_x_m
    var state_z: float = vehicle_dynamics.position_z_m
    var state_heading: float = vehicle_dynamics.heading_rad
    var state_speed: float = vehicle_dynamics.speed_mps
    var state_acceleration: float = vehicle_dynamics.actual_acceleration_mps2
    var state_steering: float = vehicle_dynamics.steering_rad
    var ego_position: Vector3 = Vector3(state_x, 0.4, state_z)
    var forward: Vector3 = Vector3(sin(state_heading), 0.0, cos(state_heading))

    # --- 3. Render From Authoritative Vehicle State ---
    if ego_car:
        ego_car.position = ego_position
        ego_car.rotation.y = state_heading
    if lead_car && scenario_blocked_road && not steering_test_mode && not autonomous_route_mode:
        lead_car.position = Vector3(0.0, 0.4, lead_pos)

    if follow_camera:
        if autonomous_route_mode:
            _update_mode4_camera(ego_position, state_heading, delta)
        else:
            follow_camera.position = ego_position + Vector3.UP * 3.6 - forward * 10.0
            follow_camera.look_at(ego_position + Vector3.UP * 0.6 + forward * 15.0, Vector3.UP)

    # --- 4. Scenario Status ---
    var raw_bumper_sep: float = 0.0
    var contact_status: String
    if autonomous_route_mode:
        contact_status = route_decision.get("status", "SAFE STOP / INVALID NAVIGATION")
    elif steering_test_mode:
        contact_status = "NOT EVALUATED"
    elif scenario_blocked_road:
        var lead_rear: float = lead_pos - lead_length / 2.0
        var ego_front: float = state_z + ego_length / 2.0
        raw_bumper_sep = lead_rear - ego_front
        if raw_bumper_sep <= 0.0:
            contact_status = "!! CONTACT / COLLISION DETECTED !!"
        elif raw_bumper_sep < 2.0:
            contact_status = "! WARNING: CLEARANCE < 2.0m !"
        elif state_speed == 0.0 && state_z > 10.0:
            contact_status = "SUCCESS: STOPPED SAFELY (CL >= 2.0m)"
        else:
            contact_status = "SAFE DISTANCE"
    else:
        raw_bumper_sep = target_b - state_z
        contact_status = driving_controller.clear_road_arrival_status(
            target_b,
            state_z,
            state_speed,
            manual_input
        )

    # --- 5. HUD ---
    if hud_label:
        var control_mode: String
        if steering_test_mode:
            control_mode = "STEERING TEST (MANUAL KINEMATIC DEMO)"
        elif driving_controller.auto_drive_enabled && manual_input == 0.0:
            control_mode = "AUTONOMOUS"
        elif not driving_controller.auto_drive_enabled:
            control_mode = "MANUAL - AUTO DESTINATION CONTROL DISABLED"
        else:
            control_mode = "MANUAL OVERRIDE - AUTO DESTINATION CONTROL PAUSED"

        if autonomous_route_mode:
            var route_length_m: float = route_navigator.total_length_m if route_navigation_ready else 0.0
            var route_progress_m: float = route_decision.get("route_progress_m", 0.0)
            var route_remaining_m: float = route_decision.get("remaining_distance_m", INF)
            var route_cross_track_m: float = route_decision.get("cross_track_error_m", 0.0)
            var route_endpoint_error_m: float = route_decision.get("endpoint_error_m", INF)
            var route_status: String = route_decision.get("status", "SAFE STOP / INVALID NAVIGATION")
            var route_localization: String = route_decision.get("localization_quality", "LOST")
            var route_heading_rad: float = vehicle_dynamics.heading_rad
            hud_label.text = "[MODE 4 - AUTONOMOUS ROUTE FOLLOWING]\n" + \
                "Kinematic simulation; not a validated safety system.\n" + \
                "Control Mode: %s\n" % [
                    "AUTONOMOUS" if driving_controller.auto_drive_enabled && manual_input == 0.0 && manual_steering_input == 0.0 \
                    else "MANUAL OVERRIDE - AUTONOMOUS ARRIVAL NOT CERTIFIED"
                ] + \
                "Position: X %.2f m | Z %.2f m | Heading: %.1f deg\n" % [
                    state_x, state_z, rad_to_deg(route_heading_rad)
                ] + \
                "Route Progress: %.2f / %.2f m | Remaining: %.2f m\n" % [
                    route_progress_m, route_length_m, route_remaining_m
                ] + \
                "Cross-Track Error: %.3f m | Endpoint Error: %.2f m\n" % [
                    route_cross_track_m, route_endpoint_error_m
                ] + \
                "Speed: %.2f m/s | Steering: %.1f deg | Accel: %.2f m/s²\n" % [
                    state_speed, rad_to_deg(state_steering), state_acceleration
                ] + \
                "Localization: %s | Navigation: %s\n" % [route_localization, route_status] + \
                "Braking State: %s\n" % [braking_state] + \
                "Route: %s | [5] Left | [6] Right | [7] S-Curve\n" % [route_variant] + \
                "Controls: [1] Blocked | [2] Clear | [3] Steering Test | [4] Route Follow | [8] Passenger Demo | [SPACE] Auto/Manual | [W/S] Drive/Brake | [A/D] Steer | [R] Reset"
        elif steering_test_mode:
            hud_label.text = "[STEERING TEST - KINEMATIC DEMONSTRATION]\n" + \
                "Not autonomous lane following; obstacle/AEB safety is not evaluated.\n" + \
                "Position: X %.2f m | Z %.2f m\n" % [state_x, state_z] + \
                "Heading: %.2f rad (%.1f deg) | Steering: %.1f deg\n" % [
                    state_heading, rad_to_deg(state_heading), rad_to_deg(state_steering)
                ] + \
                "Speed: %.2f m/s (%.1f km/h) | Acceleration: %.2f m/s²\n" % [
                    state_speed, state_speed * 3.6, state_acceleration
                ] + \
                "Controls: [W/Up] Accelerate | [S/Down] Brake | [A] Left | [D] Right | [R] Reset | [3] Exit | [8] Passenger Demo"
        else:
            var scenario_name: String = "BLOCKED-ROAD (Obstacle at Z=60)" if scenario_blocked_road else "CLEAR-ROAD (Destination B at Z=100)"
            hud_label.text = "[EASY DRIVE GODOT DEMO]\n" + \
                "Scenario: %s\n" % [scenario_name] + \
                "Control Mode: %s\n" % [control_mode] + \
                "Ego Position: X %.2f m | Z %.2f m | Target B: %.1f m\n" % [state_x, state_z, target_b] + \
                "Heading: %.2f rad | Steering: %.1f deg\n" % [state_heading, rad_to_deg(state_steering)] + \
                "Ego Speed: %.1f m/s (%.1f km/h)\n" % [state_speed, state_speed * 3.6] + \
                "Actual Acceleration: %.2f m/s²\n" % [state_acceleration] + \
                "Signed Clearance / Distance: %.2f m\n" % [raw_bumper_sep] + \
                "Obstacle State: %s\n" % [obstacle_state_str] + \
                "Braking State: %s\n" % [braking_state] + \
                "Destination Status: %s\n" % [contact_status] + \
                "----------------------------------------\n" + \
                "Controls: [1] Blocked-Road | [2] Clear-Road | [3] Steering Test | [4] Route Following | [8] Passenger Demo | [SPACE] Auto/Man | [R] Reset"

        if not service_mode_message.is_empty():
            hud_label.text += "\n" + service_mode_message

func _update_mode4_camera(
    vehicle_position: Vector3,
    vehicle_heading_rad: float,
    delta: float,
    snap_position: bool = false
) -> void:
    if not follow_camera:
        return
    if not mode4_camera_initialized:
        mode4_camera_heading_rad = vehicle_heading_rad
        mode4_camera_initialized = true
    elif delta > 0.0:
        var heading_weight: float = 1.0 - exp(-MODE4_CAMERA_HEADING_RESPONSE * delta)
        mode4_camera_heading_rad = lerp_angle(
            mode4_camera_heading_rad,
            vehicle_heading_rad,
            heading_weight
        )

    var camera_forward := Vector3(
        sin(mode4_camera_heading_rad),
        0.0,
        cos(mode4_camera_heading_rad)
    )
    var desired_position: Vector3 = vehicle_position + Vector3.UP * 3.6 \
        - camera_forward * MODE4_CAMERA_FOLLOW_DISTANCE_M
    if snap_position:
        follow_camera.global_position = desired_position
    else:
        var position_weight: float = 1.0 - exp(-MODE4_CAMERA_POSITION_RESPONSE * maxf(delta, 0.0))
        follow_camera.global_position = follow_camera.global_position.lerp(
            desired_position,
            position_weight
        )
    follow_camera.look_at(
        vehicle_position + Vector3.UP * 0.6 + camera_forward * MODE4_CAMERA_LOOKAHEAD_M,
        Vector3.UP
    )
