extends SceneTree

const PassengerJourney = preload("res://scripts/passenger_journey.gd")

var assertions: int = 0
var failures: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    root.size = Vector2i(1024, 576)
    var visibility_simulation: Node = await _create_simulation()
    _expect(
        visibility_simulation.booking_panel == null,
        "booking panel is not created or shown in legacy mode"
    )
    visibility_simulation.select_passenger_service_mode()
    var visibility_panel = visibility_simulation.booking_panel
    _expect(
        visibility_panel != null && visibility_panel.visible,
        "booking panel becomes visible when Mode 5 is entered"
    )
    visibility_simulation.select_clear_road_mode()
    _expect(
        not visibility_panel.visible,
        "booking panel is hidden after safely leaving idle Mode 5"
    )
    visibility_simulation.queue_free()
    await process_frame

    var shortcut_simulation: Node = await _create_simulation()
    shortcut_simulation.select_passenger_service_mode()
    var shortcut_booking: Dictionary = shortcut_simulation.request_service_demo_booking()
    var shortcut_snapshot: Dictionary = shortcut_simulation.service_coordinator.get_service_snapshot()
    _expect(
        shortcut_booking.get("valid", false)
            && shortcut_snapshot.active_booking.get("origin_station_id", "") == "sports_hall"
            && shortcut_snapshot.active_booking.get("destination_station_id", "")
                == "innovation_centre",
        "backward-compatible B demo booking still uses its original coordinator request"
    )
    shortcut_simulation.queue_free()
    await process_frame

    var simulation: Node = await _create_simulation()
    simulation.select_passenger_service_mode()
    var panel = simulation.booking_panel
    var network: RefCounted = simulation.service_coordinator.network
    var stations: Array[Dictionary] = network.get_stations()
    await process_frame
    var panel_rect: Rect2 = panel.get_global_rect()
    var map_rect: Rect2 = simulation.service_route_map.get_global_rect()
    var telemetry_rect: Rect2 = simulation.get_node(
        "CanvasLayer/TelemetryPanel"
    ).get_global_rect()
    _expect(
        not panel_rect.intersects(map_rect)
            && not panel_rect.intersects(telemetry_rect)
            && panel_rect.end.y <= get_root().get_visible_rect().size.y,
        "compact booking panel fits the viewport without overlapping the map or telemetry HUD"
    )
    _expect(
        panel.pickup_selector.item_count == stations.size()
            && panel.destination_selector.item_count == stations.size(),
        "both selectors contain every station from CampusNetwork"
    )
    var all_options_match_network: bool = true
    for index in range(stations.size()):
        var station_id: String = str(stations[index].id)
        all_options_match_network = all_options_match_network \
            && str(panel.pickup_selector.get_item_metadata(index)) == station_id \
            && str(panel.destination_selector.get_item_metadata(index)) == station_id \
            && panel.pickup_selector.get_item_text(index) \
                == str(stations[index].display_name).replace(" Stop", "") \
            && panel.destination_selector.get_item_text(index) \
                == str(stations[index].display_name).replace(" Stop", "")
    _expect(
        all_options_match_network,
        "selector labels and station IDs match actual CampusNetwork entries"
    )

    _select_station(panel.pickup_selector, stations[0].id)
    _select_station(panel.destination_selector, stations[0].id)
    var dynamics_before_validation: int = simulation.service_coordinator.dynamics_step_count
    panel.request_button.pressed.emit()
    _expect(
        simulation.service_coordinator.journey.state == PassengerJourney.IDLE
            && simulation.service_coordinator.dispatcher.get_active_booking().is_empty()
            && panel.feedback_label.text.contains("must be different"),
        "same-station selection shows validation and does not create a booking"
    )
    _expect(
        simulation.service_coordinator.dynamics_step_count == dynamics_before_validation,
        "booking form validation does not advance vehicle dynamics"
    )

    var pickup_id: String = "sports_hall"
    var destination_id: String = "library"
    _select_station(panel.pickup_selector, pickup_id)
    _select_station(panel.destination_selector, destination_id)
    var service_pose_before: Dictionary = simulation.service_coordinator.get_service_snapshot()
    var legacy_pose_before: Dictionary = _legacy_dynamics_snapshot(simulation)
    var dynamics_steps_before: int = simulation.service_coordinator.dynamics_step_count
    panel.request_button.pressed.emit()
    var accepted_snapshot: Dictionary = simulation.service_coordinator.get_service_snapshot()
    _expect(
        accepted_snapshot.active_booking.get("origin_station_id", "") == pickup_id
            && accepted_snapshot.active_booking.get("destination_station_id", "")
                == destination_id
            && accepted_snapshot.service_state == PassengerJourney.TO_PICKUP,
        "selected station IDs are submitted to and accepted by the coordinator"
    )
    _expect(
        panel.feedback_label.text.contains("Request accepted")
            && panel.service_status_label.text.contains("occupied")
            && panel.request_button.disabled,
        "accepted booking reports coordinator-backed occupied status and disables requests"
    )
    _expect(
        accepted_snapshot.position_xz == service_pose_before.position_xz
            && accepted_snapshot.heading_rad == service_pose_before.heading_rad
            && accepted_snapshot.speed_mps == service_pose_before.speed_mps
            && simulation.service_coordinator.dynamics_step_count == dynamics_steps_before
            && _legacy_dynamics_snapshot(simulation) == legacy_pose_before,
        "booking submission neither advances nor resets service or legacy dynamics"
    )

    var first_booking_id: String = str(accepted_snapshot.active_booking.booking_id)
    panel.request_button.pressed.emit()
    var second_booking_id: String = str(
        simulation.service_coordinator.get_service_snapshot().active_booking.booking_id
    )
    _expect(
        second_booking_id == first_booking_id,
        "repeated button activation cannot create a duplicate active booking"
    )
    var busy_result: Dictionary = simulation.request_passenger_booking(
        "library",
        "innovation_centre"
    )
    _expect(
        not busy_result.get("valid", false)
            && busy_result.get("failure_reason", "") == "shuttle_occupied"
            && panel.feedback_label.text.contains("busy"),
        "busy booking rejection is reported from the coordinator response"
    )
    _expect(
        panel.request_button.disabled
            && panel.service_status_label.text.contains("occupied"),
        "new UI bookings remain disabled while the passenger journey is active"
    )

    panel.pickup_selector.grab_focus()
    _expect(panel.has_keyboard_focus(), "booking controls retain normal keyboard focus")
    simulation._physics_process_passenger_service(1.0 / 60.0)
    _expect(
        simulation.service_coordinator.journey.state == PassengerJourney.TO_PICKUP
            && simulation.service_coordinator.dynamics_step_count == dynamics_steps_before + 1,
        "focused UI does not trigger shortcuts or reset state while physics keeps running"
    )

    simulation.service_coordinator.interrupt_service("test UI blocked status")
    var blocked_snapshot: Dictionary = simulation.service_coordinator.get_service_snapshot()
    panel.update_service_snapshot(blocked_snapshot)
    var blocked_result: Dictionary = simulation.request_passenger_booking(
        "library",
        "innovation_centre"
    )
    _expect(
        not blocked_result.get("valid", false)
            && blocked_result.get("failure_reason", "") == "shuttle_occupied"
            && panel.feedback_label.text.contains("service is blocked")
            && panel.request_button.disabled
            && panel.service_status_label.text.contains("blocked"),
        "blocked coordinator rejection explains that the active reservation is retained"
    )

    simulation.queue_free()
    await process_frame
    if failures == 0:
        print("Passenger booking panel tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d passenger booking panel assertions failed." % [failures, assertions])
        quit(1)

func _create_simulation() -> Node:
    var simulation: Node = load("res://scenes/main.tscn").instantiate()
    root.add_child(simulation)
    simulation.set_physics_process(false)
    await process_frame
    return simulation

func _select_station(selector: OptionButton, station_id: String) -> void:
    for index in range(selector.item_count):
        if str(selector.get_item_metadata(index)) == station_id:
            selector.select(index)
            return
    push_error("Station option not found: " + station_id)

func _legacy_dynamics_snapshot(simulation: Node) -> Dictionary:
    return {
        "position_x_m": simulation.vehicle_dynamics.position_x_m,
        "position_z_m": simulation.vehicle_dynamics.position_z_m,
        "heading_rad": simulation.vehicle_dynamics.heading_rad,
        "speed_mps": simulation.vehicle_dynamics.speed_mps,
        "acceleration_mps2": simulation.vehicle_dynamics.actual_acceleration_mps2,
        "steering_rad": simulation.vehicle_dynamics.steering_rad
    }

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1
