extends Node3D

const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")
const DrivingController = preload("res://scripts/driving_controller.gd")

@export var ego_max_accel: float = 3.5
@export var ego_max_brake: float = 6.0
@export var ego_max_speed: float = 10.0

var vehicle_dynamics = VehicleDynamics.new()
var driving_controller = DrivingController.new()
var lead_pos: float = 60.0
var lead_speed: float = 0.0

var target_b: float = 100.0
var scenario_blocked_road: bool = true
var steering_test_mode: bool = false
var key3_pressed_last_frame: bool = false
var ego_length: float = 4.0
var lead_length: float = 4.0

@onready var ego_car: Node3D = $EgoCar
@onready var lead_car: Node3D = $LeadCar
@onready var follow_camera: Camera3D = $FollowCamera
@onready var hud_label: Label = $CanvasLayer/TelemetryPanel/HUD

func _ready() -> void:
    vehicle_dynamics.max_acceleration = ego_max_accel
    vehicle_dynamics.max_braking = ego_max_brake
    vehicle_dynamics.max_speed = ego_max_speed
    reset_simulation()

func reset_simulation() -> void:
    driving_controller.reset()
    vehicle_dynamics.reset()
    if lead_car:
        lead_car.visible = scenario_blocked_road && not steering_test_mode
    if scenario_blocked_road:
        lead_pos = 60.0
        lead_speed = 0.0
    else:
        lead_pos = 999.0
        lead_speed = 0.0

func _physics_process(delta: float) -> void:
    # --- 1. Keyboard Controls & Mode Selection ---
    if Input.is_key_pressed(KEY_1):
        steering_test_mode = false
        scenario_blocked_road = true
        reset_simulation()
    elif Input.is_key_pressed(KEY_2):
        steering_test_mode = false
        scenario_blocked_road = false
        reset_simulation()

    var key3_current: bool = Input.is_key_pressed(KEY_3)
    if key3_current && not key3_pressed_last_frame:
        steering_test_mode = not steering_test_mode
        reset_simulation()
    key3_pressed_last_frame = key3_current

    if Input.is_action_just_pressed("ui_accept") or Input.is_key_pressed(KEY_SPACE):
        driving_controller.auto_drive_enabled = not driving_controller.auto_drive_enabled

    if Input.is_key_pressed(KEY_R):
        reset_simulation()

    var ego_pos: float = vehicle_dynamics.position_z_m
    var ego_speed: float = vehicle_dynamics.speed_mps
    var requested_acceleration: float = 0.0
    var requested_steering: float = 0.0
    var braking_state: String = "COASTING"
    var obstacle_state_str: String = "CLEAR"
    var manual_input: float = 0.0

    if Input.is_action_pressed("ui_up") or Input.is_key_pressed(KEY_W):
        manual_input = 1.0
    elif Input.is_action_pressed("ui_down") or Input.is_key_pressed(KEY_S):
        manual_input = -1.0

    if steering_test_mode:
        # Manual kinematic demonstration only; no obstacle or AEB evaluation.
        if manual_input > 0.0:
            requested_acceleration = ego_max_accel
            braking_state = "MANUAL ACCEL"
        elif manual_input < 0.0:
            requested_acceleration = -ego_max_brake
            braking_state = "MANUAL BRAKE"

        if Input.is_key_pressed(KEY_A):
            requested_steering = -vehicle_dynamics.steering_limit_rad
        elif Input.is_key_pressed(KEY_D):
            requested_steering = vehicle_dynamics.steering_limit_rad
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
    if scenario_blocked_road && not steering_test_mode:
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
    if lead_car && scenario_blocked_road && not steering_test_mode:
        lead_car.position = Vector3(0.0, 0.4, lead_pos)

    if follow_camera:
        follow_camera.position = ego_position + Vector3.UP * 3.6 - forward * 10.0
        follow_camera.look_at(ego_position + Vector3.UP * 0.6 + forward * 15.0, Vector3.UP)

    # --- 4. Scenario Status ---
    var raw_bumper_sep: float = 0.0
    var contact_status: String
    if steering_test_mode:
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

        if steering_test_mode:
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
                "Controls: [1] Blocked-Road | [2] Clear-Road | [3] Steering Test | [SPACE] Auto/Man | [R] Reset"
