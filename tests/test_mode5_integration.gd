extends SceneTree

const FIXED_STEP: float = 1.0 / 60.0
const MAX_TICKS_PER_LEG: int = 12000
const PassengerJourney = preload("res://scripts/passenger_journey.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")

var failures: int = 0
var assertions: int = 0
var simulation: Node
var next_tick_id: int = 1

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    await _create_simulation()
    _test_mode_entry_and_map()
    _test_booking_controls_and_mode_isolation()
    _test_complete_visual_service_journey()
    _test_safe_restart_and_existing_modes()
    await _test_blocked_journey_cannot_be_discarded()
    simulation.queue_free()
    await process_frame
    if failures == 0:
        print("Mode 5 integration tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d Mode 5 integration assertions failed." % [failures, assertions])
        quit(1)

func _create_simulation() -> void:
    var packed_scene: PackedScene = load("res://scenes/main.tscn")
    simulation = packed_scene.instantiate()
    root.add_child(simulation)
    simulation.set_physics_process(false)
    await process_frame

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _test_mode_entry_and_map() -> void:
    var initial_position := Vector2(
        simulation.vehicle_dynamics.position_x_m,
        simulation.vehicle_dynamics.position_z_m
    )
    _expect(
        initial_position.distance_to(Vector2.ZERO) <= 0.001
            && simulation.vehicle_dynamics.speed_mps <= 0.1,
        "fresh legacy scene starts at the North Gate reset pose and stopped"
    )
    var key8_event := InputEventKey.new()
    key8_event.keycode = KEY_8
    key8_event.physical_keycode = KEY_8
    key8_event.pressed = true
    key8_event.echo = true
    simulation._input(key8_event)
    _expect(
        not simulation.passenger_service_mode,
        "key-repeat events do not retrigger Mode 5"
    )
    key8_event.echo = false
    key8_event.pressed = false
    simulation._input(key8_event)
    _expect(
        not simulation.passenger_service_mode,
        "key release events do not activate Mode 5"
    )
    key8_event.pressed = true
    simulation._input(key8_event)
    _expect(
        simulation.passenger_service_mode,
        "a top-row 8 key event enters Mode 5 from a fresh simulation"
    )
    _expect(
        simulation.service_coordinator.initialized
            && simulation.service_coordinator.dynamics != simulation.vehicle_dynamics,
        "Mode 5 uses its initialized coordinator as a separate motion authority"
    )
    _expect(
        is_equal_approx(PassengerJourney.POSITION_TOLERANCE_M, 1.0)
            && is_equal_approx(PassengerJourney.HEADING_TOLERANCE_RAD, deg_to_rad(12.0)),
        "route handoff retains the existing one-metre and twelve-degree validation tolerances"
    )
    var launch_segment: Dictionary = simulation.service_coordinator.network.get_segment(
        "north_to_east"
    )
    var launch_points: Array = launch_segment.centerline_points
    var launch_tangent: Vector2 = launch_points[1] - launch_points[0]
    var expected_launch_heading: float = atan2(launch_tangent.x, launch_tangent.y)
    _expect(
        is_equal_approx(
            simulation.service_coordinator.dynamics.heading_rad,
            expected_launch_heading
        )
            && Vector2(
                simulation.ego_car.position.x,
                simulation.ego_car.position.z
            ) == Vector2(
                simulation.service_coordinator.dynamics.position_x_m,
                simulation.service_coordinator.dynamics.position_z_m
            ),
        "Mode 5 starts at the same North Gate pose aligned to the network launch tangent"
    )
    _expect(
        simulation.service_road_root.visible
            && simulation.service_road_root.get_node_or_null("Road_north_to_east") != null
            && simulation.service_road_root.get_node_or_null("StationLabel_innovation_centre") != null
            && simulation.service_route_map.visible,
        "campus roads, station labels, and route map are generated from CampusNetwork"
    )
    var rendered_roads: int = 0
    var rendered_station_labels: int = 0
    for child in simulation.service_road_root.get_children():
        if child.name.begins_with("Road_"):
            rendered_roads += 1
        if child.name.begins_with("StationLabel_"):
            rendered_station_labels += 1
    _expect(
        rendered_roads == simulation.service_coordinator.network.get_segment_ids().size()
            && rendered_station_labels
                == simulation.service_coordinator.network.get_station_ids().size(),
        "every directed campus segment and station receives a matching visual"
    )
    var network_station_count: int = simulation.service_coordinator.network.get_stations().size()
    var map_station_count: int = 0
    for station in simulation.service_coordinator.network.get_stations():
        var map_position: Vector2 = simulation.service_route_map._world_to_map(
            station.stop_position_xz
        )
        if Rect2(Vector2.ZERO, simulation.service_route_map.size).has_point(map_position):
            map_station_count += 1
    _expect(
        network_station_count == 4 && map_station_count == network_station_count,
        "all four CampusNetwork station positions are represented inside the route map"
    )
    _expect(
        simulation.hud_label.text.contains("MODE 5")
            && simulation.hud_label.text.contains("Available")
            && simulation.hud_label.text.contains("[B] Request demo"),
        "Mode 5 HUD reports live service availability and demonstration controls"
    )

func _test_booking_controls_and_mode_isolation() -> void:
    var before_position := Vector2(
        simulation.vehicle_dynamics.position_x_m,
        simulation.vehicle_dynamics.position_z_m
    )
    var early_boarding: Dictionary = simulation.confirm_service_demo_boarding()
    var early_exit: Dictionary = simulation.confirm_service_demo_exit()
    _expect(
        not early_boarding.get("valid", false)
            && not early_exit.get("valid", false)
            && simulation.service_coordinator.journey.state == PassengerJourney.IDLE,
        "boarding and passenger exit are rejected in the wrong journey state"
    )
    var booking: Dictionary = simulation.request_service_demo_booking()
    _expect(
        booking.get("valid", false)
            && simulation.service_coordinator.journey.state == PassengerJourney.TO_PICKUP
            && simulation.service_coordinator.get_service_snapshot().active_booking.origin_station_id
                == "sports_hall",
        "demo booking reserves Sports Hall to Innovation Centre with a pickup route"
    )
    var route_samples: PackedVector2Array = simulation.service_coordinator.navigator.get_centerline_samples()
    _expect(
        simulation.service_active_route.mesh != null
            && simulation.service_route_map._active_route.size() == route_samples.size(),
        "active road highlight uses the coordinator's installed navigator route"
    )

    var accepted_ticks_before: int = simulation.service_coordinator.coordinator_tick_count
    var service_steps_before: int = simulation.service_coordinator.dynamics_step_count
    var camera_before: Vector3 = simulation.follow_camera.global_position
    var legacy_pose_before := Vector2(
        simulation.vehicle_dynamics.position_x_m,
        simulation.vehicle_dynamics.position_z_m
    )
    var accepted_tick: Dictionary = simulation._advance_passenger_service(
        FIXED_STEP,
        next_tick_id
    )
    next_tick_id += 1
    var snapshot: Dictionary = simulation.service_coordinator.get_service_snapshot()
    _expect(
        accepted_tick.get("valid", false)
            && simulation.service_coordinator.coordinator_tick_count == accepted_ticks_before + 1
            && simulation.service_coordinator.dynamics_step_count == service_steps_before + 1,
        "one active Mode 5 physics tick advances only the coordinator once"
    )
    _expect(
        Vector2(simulation.vehicle_dynamics.position_x_m, simulation.vehicle_dynamics.position_z_m)
                == legacy_pose_before
            && Vector2(simulation.ego_car.position.x, simulation.ego_car.position.z)
                == Vector2(snapshot.position_xz.x, snapshot.position_xz.y)
            && is_equal_approx(simulation.ego_car.rotation.y, snapshot.heading_rad),
        "visible shuttle follows coordinator pose while legacy dynamics remain untouched"
    )
    _expect(
        simulation.follow_camera.global_position.distance_to(camera_before) > 0.0,
        "existing smoothed follow camera tracks the Mode 5 coordinator vehicle"
    )
    var duplicate: Dictionary = simulation._advance_passenger_service(
        FIXED_STEP,
        next_tick_id - 1
    )
    _expect(
        duplicate.get("duplicate_tick", false)
            && simulation.service_coordinator.coordinator_tick_count == accepted_ticks_before + 1
            && simulation.service_coordinator.dynamics_step_count == service_steps_before + 1,
        "duplicate Mode 5 physics tick does not double-step either vehicle"
    )
    var active_coordinator = simulation.service_coordinator
    var active_booking_id: String = str(
        active_coordinator.get_service_snapshot().active_booking.booking_id
    )
    var key8_during_booking := InputEventKey.new()
    key8_during_booking.keycode = KEY_8
    key8_during_booking.physical_keycode = KEY_8
    key8_during_booking.pressed = true
    simulation._input(key8_during_booking)
    _expect(
        simulation.service_coordinator == active_coordinator
            && simulation.service_coordinator.journey.state == PassengerJourney.TO_PICKUP
            && str(
                simulation.service_coordinator.get_service_snapshot().active_booking.booking_id
            ) == active_booking_id,
        "pressing 8 during an active booking cannot restart or discard the journey"
    )
    var rejected_restart: Dictionary = simulation.restart_service_demo()
    simulation.select_clear_road_mode()
    _expect(
        not rejected_restart.get("valid", false)
            && simulation.passenger_service_mode
            && simulation.service_coordinator.dispatcher.has_active_booking()
            && simulation.service_coordinator.journey.state == PassengerJourney.TO_PICKUP,
        "active booking cannot be reset or switched away from"
    )
    _expect(
        simulation.service_coordinator.get_service_snapshot().position_xz.distance_to(
            before_position
        ) > 0.0,
        "coordinator motion does not depend on the legacy vehicle dynamics"
    )

func _test_complete_visual_service_journey() -> void:
    var reached_pickup: bool = _drive_until(PassengerJourney.BOARDING, "Sports Hall pickup")
    _expect(
        reached_pickup
            && simulation.service_coordinator.get_service_snapshot().arrival_certified
            && simulation.hud_label.text.contains("Sports Hall arrival certified")
            && simulation.hud_label.text.contains("Waiting for boarding confirmation"),
        "HUD reports certified pickup and waits for boarding"
    )
    var pickup_handoff_before: Dictionary = simulation.service_coordinator.get_service_snapshot()
    var boarded: Dictionary = simulation.confirm_service_demo_boarding()
    var pickup_handoff_after: Dictionary = simulation.service_coordinator.get_service_snapshot()
    _expect(
        boarded.get("valid", false)
            && simulation.service_coordinator.journey.state == PassengerJourney.TO_DESTINATION
            && simulation.service_coordinator.current_destination_station_id == "innovation_centre",
        "boarding confirmation installs the planned Innovation Centre route"
    )
    _expect(
        pickup_handoff_before.position_xz == pickup_handoff_after.position_xz
            && is_equal_approx(
                pickup_handoff_before.heading_rad,
                pickup_handoff_after.heading_rad
            )
            && is_equal_approx(pickup_handoff_before.speed_mps, pickup_handoff_after.speed_mps),
        "pickup-to-destination route handoff preserves shuttle pose, heading, and speed"
    )
    var reached_destination: bool = _drive_until(PassengerJourney.DROPOFF, "Innovation Centre")
    _expect(
        reached_destination
            && simulation.service_coordinator.get_service_snapshot().arrival_certified
            && simulation.hud_label.text.contains("Innovation Centre arrival certified")
            && simulation.hud_label.text.contains("Waiting for passenger exit confirmation"),
        "HUD reports certified destination and waits for passenger exit"
    )
    var destination_handoff_before: Dictionary = simulation.service_coordinator.get_service_snapshot()
    var exited: Dictionary = simulation.confirm_service_demo_exit()
    var destination_handoff_after: Dictionary = simulation.service_coordinator.get_service_snapshot()
    _expect(
        exited.get("valid", false)
            && simulation.service_coordinator.journey.state == PassengerJourney.RETURNING_TO_SERVICE,
        "passenger exit confirmation installs the North Gate return route"
    )
    _expect(
        destination_handoff_before.position_xz == destination_handoff_after.position_xz
            && is_equal_approx(
                destination_handoff_before.heading_rad,
                destination_handoff_after.heading_rad
            )
            && is_equal_approx(
                destination_handoff_before.speed_mps,
                destination_handoff_after.speed_mps
            ),
        "destination-to-return route handoff preserves shuttle pose, heading, and speed"
    )
    var returned: bool = _drive_until(PassengerJourney.IDLE, "North Gate return")
    var final_snapshot: Dictionary = simulation.service_coordinator.get_service_snapshot()
    _expect(
        returned
            && final_snapshot.service_state == PassengerJourney.IDLE
            && final_snapshot.active_booking.is_empty()
            && final_snapshot.arrival_certified
            && simulation.hud_label.text.contains("Available"),
        "certified return alone releases the booking and displays Available"
    )
    _expect(
        final_snapshot.route_leg_sequence == PackedStringArray([
            "pickup",
            "destination",
            "return"
        ]),
        "visual demonstration follows pickup, destination, and return route legs"
    )

func _test_safe_restart_and_existing_modes() -> void:
    var pose_before_restart: Vector2 = simulation.service_coordinator.get_service_snapshot().position_xz
    var restarted: Dictionary = simulation.restart_service_demo()
    var pose_after_restart: Vector2 = simulation.service_coordinator.get_service_snapshot().position_xz
    _expect(
        restarted.get("valid", false)
            && pose_after_restart.distance_to(pose_before_restart) <= 0.000001
            && not simulation.service_coordinator.dispatcher.has_active_booking(),
        "idle demo restart reinitializes at the actual returned pose without a snap"
    )

    simulation.select_blocked_road_mode()
    _expect(
        not simulation.passenger_service_mode
            && simulation.scenario_blocked_road
            && simulation.vehicle_dynamics.position_x_m == 0.0
            && simulation.vehicle_dynamics.position_z_m == 0.0,
        "Mode 1 remains selectable with its original reset behavior"
    )
    simulation.select_clear_road_mode()
    _expect(
        not simulation.scenario_blocked_road && not simulation.autonomous_route_mode,
        "Mode 2 remains selectable"
    )
    simulation.toggle_steering_test_mode()
    _expect(
        simulation.steering_test_mode && not simulation.autonomous_route_mode,
        "Mode 3 remains selectable"
    )
    simulation.select_route_following_mode()
    _expect(
        simulation.autonomous_route_mode && simulation.route_navigation_ready,
        "Mode 4 still initializes its existing autonomous route"
    )
    var blocked := VehicleDynamics.new()
    blocked.reset(0.0, 0.0)
    _expect(
        blocked.speed_mps == 0.0 && simulation.route_navigator.route_valid,
        "existing Modes 1-4 remain isolated from the passenger coordinator"
    )

func _test_blocked_journey_cannot_be_discarded() -> void:
    var keypad_simulation: Node = load("res://scenes/main.tscn").instantiate()
    root.add_child(keypad_simulation)
    keypad_simulation.set_physics_process(false)
    await process_frame
    var keypad_event := InputEventKey.new()
    keypad_event.keycode = KEY_KP_8
    keypad_event.physical_keycode = KEY_KP_8
    keypad_event.pressed = true
    keypad_simulation._input(keypad_event)
    _expect(
        keypad_simulation.passenger_service_mode,
        "numeric keypad 8 also enters Mode 5"
    )
    keypad_simulation.queue_free()
    await process_frame

    var far_simulation: Node = load("res://scenes/main.tscn").instantiate()
    root.add_child(far_simulation)
    far_simulation.set_physics_process(false)
    await process_frame
    far_simulation.vehicle_dynamics.reset(25.0, 0.0, 14.0, 0.7, 0.12)
    var far_legacy_before: Dictionary = _legacy_dynamics_snapshot(far_simulation)
    var far_entry: Dictionary = far_simulation.select_passenger_service_mode()
    var far_service_snapshot: Dictionary = far_simulation.service_coordinator.get_service_snapshot()
    _expect(
        far_entry.get("valid", false)
            && far_simulation.passenger_service_mode
            && far_service_snapshot.position_xz
                == far_simulation.service_coordinator.network
                    .get_station_stop("north_gate").position_xz,
        "Mode 5 enters from a stationary legacy pose farther than one metre from North Gate"
    )
    var far_launch_segment: Dictionary = far_simulation.service_coordinator.network.get_segment(
        "north_to_east"
    )
    var far_launch_points: Array = far_launch_segment.centerline_points
    var far_tangent: Vector2 = far_launch_points[1] - far_launch_points[0]
    var far_expected_heading: float = atan2(far_tangent.x, far_tangent.y)
    _expect(
        far_service_snapshot.position_xz
                == far_simulation.service_coordinator.network
                    .get_station_stop("north_gate").position_xz
            && far_service_snapshot.speed_mps == 0.0
            && is_equal_approx(far_service_snapshot.heading_rad, far_expected_heading),
        "independent service shuttle starts stopped at North Gate with legal launch heading"
    )
    _expect(
        _legacy_dynamics_snapshot(far_simulation) == far_legacy_before,
        "Mode 5 initialization leaves the far-away legacy dynamics state unchanged"
    )
    var far_booking: Dictionary = far_simulation.request_service_demo_booking()
    far_simulation._physics_process(FIXED_STEP)
    _expect(
        far_booking.get("valid", false)
            && _legacy_dynamics_snapshot(far_simulation) == far_legacy_before
            && far_simulation.service_coordinator.dynamics_step_count == 1,
        "Mode 5 advances only coordinator dynamics after entry"
    )
    far_simulation.queue_free()
    await process_frame

    var moving_simulation: Node = load("res://scenes/main.tscn").instantiate()
    root.add_child(moving_simulation)
    moving_simulation.set_physics_process(false)
    await process_frame
    moving_simulation.vehicle_dynamics.reset(31.0, 1.7, -8.0, -0.4, -0.08)
    var moving_legacy_before: Dictionary = _legacy_dynamics_snapshot(moving_simulation)
    var moving_entry: Dictionary = moving_simulation.select_passenger_service_mode()
    var moving_service_snapshot: Dictionary = moving_simulation.service_coordinator.get_service_snapshot()
    var moving_booking: Dictionary = moving_simulation.request_service_demo_booking()
    moving_simulation._physics_process(FIXED_STEP)
    _expect(
        moving_entry.get("valid", false)
            && moving_simulation.passenger_service_mode
            && moving_booking.get("valid", false)
            && moving_service_snapshot.speed_mps == 0.0
            && moving_service_snapshot.position_xz == Vector2.ZERO,
        "Mode 5 safely suspends a moving legacy demo and starts its own stopped shuttle"
    )
    _expect(
        _legacy_dynamics_snapshot(moving_simulation) == moving_legacy_before
            && moving_simulation.service_coordinator.dynamics_step_count == 1,
        "suspended legacy motion is unchanged while the independent service shuttle advances"
    )
    moving_simulation.queue_free()
    await process_frame

    simulation.reset_simulation()
    var entered: Dictionary = simulation.select_passenger_service_mode()
    _expect(entered.get("valid", false), "Mode 5 can be entered after resetting a legacy mode")
    var booking: Dictionary = simulation.request_service_demo_booking()
    _expect(booking.get("valid", false), "second service session accepts a fresh booking")
    simulation.service_coordinator.interrupt_service("test interruption")
    var restart: Dictionary = simulation.restart_service_demo()
    simulation.select_clear_road_mode()
    _expect(
        not restart.get("valid", false)
            && simulation.passenger_service_mode
            && simulation.service_coordinator.journey.state == PassengerJourney.BLOCKED
            && simulation.service_coordinator.dispatcher.has_active_booking()
            && not simulation.service_coordinator.get_service_snapshot().arrival_certified,
        "interrupted journey stays blocked and cannot falsely become IDLE or release booking"
    )

func _legacy_dynamics_snapshot(target: Node) -> Dictionary:
    return {
        "position_x_m": target.vehicle_dynamics.position_x_m,
        "position_z_m": target.vehicle_dynamics.position_z_m,
        "heading_rad": target.vehicle_dynamics.heading_rad,
        "speed_mps": target.vehicle_dynamics.speed_mps,
        "acceleration_mps2": target.vehicle_dynamics.actual_acceleration_mps2,
        "steering_rad": target.vehicle_dynamics.steering_rad
    }

func _drive_until(target_state: String, description: String) -> bool:
    for _tick in range(MAX_TICKS_PER_LEG):
        if simulation.service_coordinator.journey.state == target_state:
            return true
        if simulation.service_coordinator.journey.state == PassengerJourney.BLOCKED:
            break
        var result: Dictionary = simulation._advance_passenger_service(
            FIXED_STEP,
            next_tick_id
        )
        next_tick_id += 1
        if not result.get("valid", false) && result.get("status", "") != "ARRIVED":
            break
    print(
        "Mode 5 leg %s ended in %s (%s)"
            % [
                description,
                simulation.service_coordinator.journey.state,
                simulation.service_coordinator.get_service_snapshot().last_failure_reason
            ]
    )
    return simulation.service_coordinator.journey.state == target_state
