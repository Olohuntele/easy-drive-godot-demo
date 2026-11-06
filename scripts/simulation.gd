extends Node3D

const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")
const DrivingController = preload("res://scripts/driving_controller.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const RouteFollowingController = preload("res://scripts/route_following_controller.gd")

const ROUTE_LEFT: String = "LEFT"
const ROUTE_RIGHT: String = "RIGHT"
const ROUTE_S_CURVE: String = "S-CURVE"
const ROUTE_ROAD_HALF_WIDTH_M: float = 4.0
const ROUTE_BOUNDARY_OFFSET_M: float = 3.82
const ROUTE_BOUNDARY_WIDTH_M: float = 0.16
const ROUTE_CENTERLINE_WIDTH_M: float = 0.12

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
var route_navigation_ready: bool = false
var route_variant: String = ROUTE_RIGHT
var route_visual_centerline_xz: PackedVector2Array = PackedVector2Array()
var key3_pressed_last_frame: bool = false
var key4_pressed_last_frame: bool = false
var key5_pressed_last_frame: bool = false
var key6_pressed_last_frame: bool = false
var key7_pressed_last_frame: bool = false
var space_pressed_last_frame: bool = false
var ego_length: float = 4.0
var lead_length: float = 4.0

@onready var ego_car: Node3D = $EgoCar
@onready var lead_car: Node3D = $LeadCar
@onready var follow_camera: Camera3D = $FollowCamera
@onready var hud_label: Label = $CanvasLayer/TelemetryPanel/HUD
@onready var straight_road: MeshInstance3D = $Road
@onready var straight_centerline: MeshInstance3D = $CenterLine
@onready var straight_roadside_scenery: Node3D = $RoadsideScenery

var route_visual_root: Node3D
var route_surface_mesh: ArrayMesh
var route_left_boundary_mesh: ArrayMesh
var route_right_boundary_mesh: ArrayMesh
var route_centerline_mesh: ArrayMesh

func _ready() -> void:
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
    steering_test_mode = false
    autonomous_route_mode = false
    scenario_blocked_road = true
    reset_simulation()

func select_clear_road_mode() -> void:
    steering_test_mode = false
    autonomous_route_mode = false
    scenario_blocked_road = false
    reset_simulation()

func toggle_steering_test_mode() -> void:
    steering_test_mode = not steering_test_mode
    autonomous_route_mode = false
    reset_simulation()

func select_route_following_mode() -> void:
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

func _route_waypoints() -> Array[Vector2]:
    if route_variant == ROUTE_LEFT:
        var left_waypoints: Array[Vector2] = []
        for point in RouteNavigator.create_broad_bend_waypoints():
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
    return RouteNavigator.create_broad_bend_waypoints()

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

    route_surface_mesh = _make_route_strip_mesh(centerline_samples, 0.0, ROUTE_ROAD_HALF_WIDTH_M * 2.0, 0.055)
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

func _physics_process(delta: float) -> void:
    # --- 1. Keyboard Controls & Mode Selection ---
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
            var fixed_forward: Vector3 = Vector3(0.0, 0.0, 1.0)
            follow_camera.position = ego_position + Vector3.UP * 3.6 - fixed_forward * 10.0
            follow_camera.look_at(
                ego_position + Vector3.UP * 0.6 + fixed_forward * 15.0,
                Vector3.UP
            )
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
                "Controls: [1] Blocked | [2] Clear | [3] Steering Test | [4] Route Follow | [SPACE] Auto/Manual | [W/S] Drive/Brake | [A/D] Steer | [R] Reset"
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
                "Controls: [W/Up] Accelerate | [S/Down] Brake | [A] Left | [D] Right | [R] Reset | [3] Exit"
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
                "Controls: [1] Blocked-Road | [2] Clear-Road | [3] Steering Test | [4] Route Following | [SPACE] Auto/Man | [R] Reset"
