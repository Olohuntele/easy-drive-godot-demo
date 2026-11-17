extends SceneTree

const PassengerJourney = preload("res://scripts/passenger_journey.gd")
const FIXED_STEP: float = 1.0 / 60.0
const MAX_TICKS_PER_LEG: int = 12000

var assertions: int = 0
var failures: int = 0
var next_tick_id: int = 1
var boarding_signal_count: int = 0
var exit_signal_count: int = 0

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    root.size = Vector2i(1024, 576)
    await _test_visibility_and_demo_shortcut()
    await _test_journey_panel_and_actions()
    await _test_blocked_feedback()
    if failures == 0:
        print("Passenger booking panel tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d passenger booking panel assertions failed." % [failures, assertions])
        quit(1)

func _test_visibility_and_demo_shortcut() -> void:
    var visibility_simulation: Node = await _create_simulation()
    _expect(
        visibility_simulation.booking_panel == null,
        "booking panel is not created in a legacy mode"
    )
    visibility_simulation.select_passenger_service_mode()
    var visibility_panel = visibility_simulation.booking_panel
    _expect(
        visibility_panel != null && visibility_panel.visible
            && visibility_panel.request_button.visible,
        "booking form appears when Mode 5 is entered idle"
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
        "B fallback still creates its original coordinator-backed demo booking"
    )
    _expect(
        not shortcut_simulation.booking_panel.request_button.visible
            && shortcut_simulation.booking_panel.journey_status_label.visible,
        "B fallback also switches the panel to journey tracking"
    )
    shortcut_simulation.queue_free()
    await process_frame

func _test_journey_panel_and_actions() -> void:
    var simulation: Node = await _create_simulation()
    simulation.select_passenger_service_mode()
    var panel = simulation.booking_panel
    var coordinator = simulation.service_coordinator
    var network: RefCounted = coordinator.network
    var stations: Array[Dictionary] = network.get_stations()
    await process_frame
    var panel_rect: Rect2 = panel.get_global_rect()
    var map_rect: Rect2 = simulation.service_route_map.get_global_rect()
    var telemetry_rect: Rect2 = simulation.get_node(
        "CanvasLayer/TelemetryPanel"
    ).get_global_rect()
    var booking_content: VBoxContainer = panel.get_child(0).get_child(0)
    _expect(
        not panel_rect.intersects(map_rect)
            && not panel_rect.intersects(telemetry_rect)
            && panel_rect.end.y <= get_root().get_visible_rect().size.y
            && booking_content.get_combined_minimum_size().y <= panel.size.y,
        "journey panel fits the viewport without overlapping the map or telemetry HUD"
    )
    _expect(
        panel.pickup_selector.item_count == stations.size()
            && panel.destination_selector.item_count == stations.size(),
        "booking selectors list all actual CampusNetwork stations"
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
    _expect(all_options_match_network, "station selector labels and IDs match the network")

    _select_station(panel.pickup_selector, "north_gate")
    _select_station(panel.destination_selector, "north_gate")
    panel.request_button.pressed.emit()
    _expect(
        coordinator.journey.state == PassengerJourney.IDLE
            && coordinator.dispatcher.get_active_booking().is_empty()
            && panel.feedback_label.text.contains("must be different"),
        "same-station requests are rejected without creating a reservation"
    )

    var legacy_before: Dictionary = _legacy_dynamics_snapshot(simulation)
    var steps_before: int = coordinator.dynamics_step_count
    _select_station(panel.pickup_selector, "sports_hall")
    _select_station(panel.destination_selector, "innovation_centre")
    panel.request_button.pressed.emit()
    var snapshot: Dictionary = coordinator.get_service_snapshot()
    _expect(
        snapshot.service_state == PassengerJourney.TO_PICKUP
            && snapshot.active_booking.get("origin_station_id", "") == "sports_hall"
            && snapshot.active_booking.get("destination_station_id", "")
                == "innovation_centre",
        "Sports Hall to Innovation Centre booking is accepted by the coordinator"
    )
    _expect(
        not panel.request_button.visible
            && panel.journey_summary_label.visible
            && panel.journey_summary_label.text.contains("Sports Hall")
            && panel.journey_summary_label.text.contains("Innovation Centre")
            && panel.journey_status_label.text.contains("Traveling to pickup"),
        "accepted booking changes the panel to tracking with the selected stations"
    )
    await process_frame
    var tracking_content: VBoxContainer = panel.get_child(0).get_child(0)
    _expect(
        tracking_content.get_combined_minimum_size().y <= panel.size.y,
        "journey tracking content fits within the panel without vertical clipping"
    )
    _expect(
        coordinator.dynamics_step_count == steps_before
            && _legacy_dynamics_snapshot(simulation) == legacy_before,
        "booking request does not step or reset either vehicle"
    )

    panel.boarding_requested.connect(_record_boarding_signal)
    panel.passenger_exit_requested.connect(_record_exit_signal)
    _expect(
        not panel.board_button.visible && not panel.exit_button.visible,
        "boarding and exit actions are hidden before certified arrival"
    )
    _expect(
        not simulation.confirm_service_demo_boarding().get("valid", false)
            && not simulation.confirm_service_demo_exit().get("valid", false),
        "coordinator rejects early boarding and passenger exit"
    )

    _expect(
        _drive_until(simulation, PassengerJourney.BOARDING),
        "coordinator reaches certified pickup state"
    )
    snapshot = coordinator.get_service_snapshot()
    panel.update_service_snapshot(snapshot)
    _expect(
        snapshot.arrival_certified
            && panel.board_button.visible
            && not panel.board_button.disabled
            && not panel.exit_button.visible
            && panel.journey_status_label.text.contains("Waiting for boarding"),
        "Board Shuttle appears only at certified pickup arrival"
    )
    var leg_count_before_boarding: int = snapshot.route_leg_sequence.size()
    panel.board_button.pressed.emit()
    snapshot = coordinator.get_service_snapshot()
    _expect(
        boarding_signal_count == 1
            && snapshot.service_state == PassengerJourney.TO_DESTINATION
            && snapshot.route_leg_sequence.size() == leg_count_before_boarding + 1,
        "Board Shuttle calls the existing coordinator API once and installs one next leg"
    )
    panel.board_button.pressed.emit()
    _expect(
        boarding_signal_count == 1
            && coordinator.get_service_snapshot().service_state == PassengerJourney.TO_DESTINATION,
        "repeated or stale boarding clicks cannot confirm a second time"
    )
    _expect(
        panel.journey_status_label.text.contains("Traveling to destination")
            && is_equal_approx(panel.journey_progress.value, 65.0),
        "journey status and progress follow the coordinator's destination state"
    )

    _expect(
        _drive_until(simulation, PassengerJourney.DROPOFF),
        "coordinator reaches certified destination state"
    )
    snapshot = coordinator.get_service_snapshot()
    panel.update_service_snapshot(snapshot)
    _expect(
        snapshot.arrival_certified
            && panel.exit_button.visible
            && not panel.exit_button.disabled
            && not panel.board_button.visible
            && panel.journey_status_label.text.contains("Waiting for passenger exit"),
        "Exit Shuttle appears only at certified destination arrival"
    )
    var leg_count_before_exit: int = snapshot.route_leg_sequence.size()
    panel.exit_button.pressed.emit()
    snapshot = coordinator.get_service_snapshot()
    _expect(
        exit_signal_count == 1
            && snapshot.service_state == PassengerJourney.RETURNING_TO_SERVICE
            && snapshot.route_leg_sequence.size() == leg_count_before_exit + 1,
        "Exit Shuttle calls the existing coordinator API once and installs one return leg"
    )
    panel.exit_button.pressed.emit()
    _expect(
        exit_signal_count == 1
            && coordinator.get_service_snapshot().service_state
                == PassengerJourney.RETURNING_TO_SERVICE,
        "repeated or stale exit clicks cannot confirm a second time"
    )
    _expect(
        panel.journey_status_label.text.contains("Returning to North Gate"),
        "return status follows the coordinator journey snapshot"
    )
    _expect(
        _drive_until(simulation, PassengerJourney.IDLE),
        "coordinator completes the certified return to service"
    )
    snapshot = coordinator.get_service_snapshot()
    panel.update_service_snapshot(snapshot)
    _expect(
        snapshot.arrival_certified
            && snapshot.active_booking.is_empty()
            && panel.request_button.visible
            && panel.pickup_selector.visible
            && not panel.journey_summary_label.visible
            && panel.service_status_label.text.contains("available")
            && panel.feedback_label.text.contains("completed"),
        "certified return restores booking form and reports available service"
    )

    _test_station_pair(simulation, panel, "library", "north_gate")
    simulation.queue_free()
    await process_frame

func _test_station_pair(
    simulation: Node,
    panel,
    pickup_id: String,
    destination_id: String
) -> void:
    _select_station(panel.pickup_selector, pickup_id)
    _select_station(panel.destination_selector, destination_id)
    var result: Dictionary = simulation.request_passenger_booking(pickup_id, destination_id)
    var booking: Dictionary = simulation.service_coordinator.get_service_snapshot().active_booking
    _expect(
        result.get("valid", false)
            && booking.get("origin_station_id", "") == pickup_id
            && booking.get("destination_station_id", "") == destination_id,
        "previously verified %s to %s station pair remains supported"
            % [pickup_id, destination_id]
    )
    _expect(
        _drive_until(simulation, PassengerJourney.BOARDING)
            && simulation.service_coordinator.get_service_snapshot().arrival_certified,
        "%s pickup reaches certified boarding state" % pickup_id
    )
    panel.board_button.pressed.emit()
    _expect(
        simulation.service_coordinator.get_service_snapshot().service_state
            == PassengerJourney.TO_DESTINATION,
        "boarding action starts selected destination leg"
    )
    _expect(
        _drive_until(simulation, PassengerJourney.DROPOFF)
            && simulation.service_coordinator.get_service_snapshot().arrival_certified,
        "%s destination reaches certified exit state" % destination_id
    )
    panel.exit_button.pressed.emit()
    var exit_snapshot: Dictionary = simulation.service_coordinator.get_service_snapshot()
    var return_started_or_completed: bool = (
        exit_snapshot.service_state == PassengerJourney.RETURNING_TO_SERVICE
        || (
            destination_id == "north_gate"
            && exit_snapshot.service_state == PassengerJourney.IDLE
            && exit_snapshot.arrival_certified
        )
    )
    _expect(
        return_started_or_completed,
        "passenger exit starts the service return or certifies a stationary North Gate return"
    )
    _expect(
        (
            exit_snapshot.service_state == PassengerJourney.IDLE
            || _drive_until(simulation, PassengerJourney.IDLE)
        )
            && simulation.service_coordinator.get_service_snapshot().arrival_certified
            && panel.request_button.visible
            && panel.feedback_label.text.contains("completed"),
        "%s to %s return restores the booking form after certification"
            % [pickup_id, destination_id]
    )

func _test_blocked_feedback() -> void:
    var simulation: Node = await _create_simulation()
    simulation.select_passenger_service_mode()
    var panel = simulation.booking_panel
    simulation.request_service_demo_booking()
    simulation.service_coordinator.interrupt_service("test UI blocked status")
    simulation._advance_passenger_service(FIXED_STEP, next_tick_id)
    next_tick_id += 1
    var snapshot: Dictionary = simulation.service_coordinator.get_service_snapshot()
    _expect(
        snapshot.service_state == PassengerJourney.BLOCKED
            && panel.journey_status_label.text.contains("blocked")
            && panel.service_status_label.text.contains("blocked")
            && panel.request_button.visible == false
            && not panel.board_button.visible
            && not panel.exit_button.visible,
        "blocked coordinator state is shown without exposing journey actions"
    )
    var result: Dictionary = simulation.request_passenger_booking(
        "library",
        "innovation_centre"
    )
    _expect(
        not result.get("valid", false)
            && panel.feedback_label.text.contains("blocked")
            && panel.feedback_label.text.contains("reservation is retained"),
        "rejected requests accurately report blocked service and retained reservation"
    )
    simulation.queue_free()
    await process_frame

func _drive_until(simulation: Node, target_state: String) -> bool:
    for _tick in range(MAX_TICKS_PER_LEG):
        if simulation.service_coordinator.journey.state == target_state:
            return true
        if simulation.service_coordinator.journey.state == PassengerJourney.BLOCKED:
            return false
        var result: Dictionary = simulation._advance_passenger_service(
            FIXED_STEP,
            next_tick_id
        )
        next_tick_id += 1
        if not result.get("valid", false) && result.get("status", "") != "ARRIVED":
            return false
    return simulation.service_coordinator.journey.state == target_state

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

func _record_boarding_signal() -> void:
    boarding_signal_count += 1

func _record_exit_signal() -> void:
    exit_signal_count += 1

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1
