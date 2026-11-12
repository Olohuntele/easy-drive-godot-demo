extends PanelContainer

signal booking_requested(pickup_station_id: String, destination_station_id: String)
signal boarding_requested
signal passenger_exit_requested

const PANEL_SIZE: Vector2 = Vector2(286.0, 264.0)
const JourneyStates = preload("res://scripts/passenger_journey.gd")

var pickup_selector: OptionButton
var destination_selector: OptionButton
var request_button: Button
var pickup_label: Label
var destination_label: Label
var service_status_label: Label
var feedback_label: Label
var journey_summary_label: Label
var journey_status_label: Label
var journey_progress: ProgressBar
var board_button: Button
var exit_button: Button
var _booking_controls: Array[Control] = []
var _network: RefCounted
var _request_in_flight: bool = false
var _action_in_flight: bool = false

func _ready() -> void:
    custom_minimum_size = PANEL_SIZE
    size = PANEL_SIZE
    anchor_left = 1.0
    anchor_right = 1.0
    offset_left = -306.0
    offset_right = -20.0
    offset_top = 300.0
    offset_bottom = 564.0
    mouse_filter = Control.MOUSE_FILTER_STOP

    var panel_style := StyleBoxFlat.new()
    panel_style.bg_color = Color(0.035, 0.055, 0.075, 0.94)
    panel_style.border_color = Color(0.4, 0.62, 0.72, 0.95)
    panel_style.set_border_width_all(1)
    panel_style.set_corner_radius_all(7)
    add_theme_stylebox_override("panel", panel_style)

    var margin := MarginContainer.new()
    add_child(margin)
    margin.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
    margin.add_theme_constant_override("margin_left", 10)
    margin.add_theme_constant_override("margin_top", 8)
    margin.add_theme_constant_override("margin_right", 10)
    margin.add_theme_constant_override("margin_bottom", 8)

    var content := VBoxContainer.new()
    content.add_theme_constant_override("separation", 4)
    margin.add_child(content)

    var title := Label.new()
    title.text = "EASY DRIVE â€” BOOK A SHUTTLE"
    title.add_theme_font_size_override("font_size", 15)
    content.add_child(title)

    pickup_label = Label.new()
    pickup_label.text = "Pickup station"
    pickup_label.add_theme_font_size_override("font_size", 12)
    content.add_child(pickup_label)
    _booking_controls.append(pickup_label)
    pickup_selector = OptionButton.new()
    pickup_selector.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    pickup_selector.focus_mode = Control.FOCUS_ALL
    content.add_child(pickup_selector)
    _booking_controls.append(pickup_selector)

    destination_label = Label.new()
    destination_label.text = "Destination"
    destination_label.add_theme_font_size_override("font_size", 12)
    content.add_child(destination_label)
    _booking_controls.append(destination_label)
    destination_selector = OptionButton.new()
    destination_selector.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    destination_selector.focus_mode = Control.FOCUS_ALL
    content.add_child(destination_selector)
    _booking_controls.append(destination_selector)

    request_button = Button.new()
    request_button.text = "Request Shuttle"
    request_button.focus_mode = Control.FOCUS_ALL
    request_button.pressed.connect(_on_request_pressed)
    content.add_child(request_button)
    _booking_controls.append(request_button)

    journey_summary_label = Label.new()
    journey_summary_label.add_theme_font_size_override("font_size", 12)
    journey_summary_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
    content.add_child(journey_summary_label)

    journey_status_label = Label.new()
    journey_status_label.add_theme_font_size_override("font_size", 13)
    journey_status_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
    content.add_child(journey_status_label)

    journey_progress = ProgressBar.new()
    journey_progress.min_value = 0.0
    journey_progress.max_value = 100.0
    journey_progress.show_percentage = true
    journey_progress.custom_minimum_size.y = 14.0
    content.add_child(journey_progress)

    board_button = Button.new()
    board_button.text = "Board Shuttle"
    board_button.focus_mode = Control.FOCUS_ALL
    board_button.visible = false
    board_button.pressed.connect(_on_board_pressed)
    content.add_child(board_button)

    exit_button = Button.new()
    exit_button.text = "Exit Shuttle"
    exit_button.focus_mode = Control.FOCUS_ALL
    exit_button.visible = false
    exit_button.pressed.connect(_on_exit_pressed)
    content.add_child(exit_button)

    service_status_label = Label.new()
    service_status_label.add_theme_font_size_override("font_size", 12)
    service_status_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
    content.add_child(service_status_label)

    feedback_label = Label.new()
    feedback_label.add_theme_font_size_override("font_size", 12)
    feedback_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
    content.add_child(feedback_label)

    var controls_hint := Label.new()
    controls_hint.text = "B demo booking Â· E board Â· X exit Â· Esc releases UI focus"
    controls_hint.add_theme_font_size_override("font_size", 10)
    controls_hint.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
    content.add_child(controls_hint)

func configure(network: RefCounted, initial_snapshot: Dictionary) -> void:
    _network = network
    pickup_selector.clear()
    destination_selector.clear()
    for station in network.get_stations():
        var station_id: String = str(station.id)
        var display_name: String = str(station.display_name).replace(" Stop", "")
        pickup_selector.add_item(display_name)
        pickup_selector.set_item_metadata(pickup_selector.item_count - 1, station_id)
        destination_selector.add_item(display_name)
        destination_selector.set_item_metadata(destination_selector.item_count - 1, station_id)
    if pickup_selector.item_count > 0:
        pickup_selector.select(0)
    if destination_selector.item_count > 1:
        destination_selector.select(1)
    update_service_snapshot(initial_snapshot)
    feedback_label.text = "Choose two different campus stops."

func update_service_snapshot(snapshot: Dictionary) -> void:
    var active_booking: Dictionary = snapshot.get("active_booking", {})
    var journey_state: String = str(snapshot.get("service_state", "UNINITIALIZED"))
    var blocked: bool = bool(snapshot.get("blocked", false)) || journey_state == "BLOCKED"
    var busy: bool = not active_booking.is_empty() || journey_state != "IDLE"
    var tracking_journey: bool = busy
    for control in _booking_controls:
        control.visible = not tracking_journey

    var pickup_name: String = _station_name(str(active_booking.get("origin_station_id", "")))
    var destination_name: String = _station_name(
        str(active_booking.get("destination_station_id", ""))
    )
    var boarding_confirmed: bool = bool(snapshot.get("boarding_confirmed", false))
    journey_summary_label.visible = tracking_journey && not active_booking.is_empty()
    journey_summary_label.text = "Trip: %s â†’ %s" % [pickup_name, destination_name]
    journey_status_label.visible = tracking_journey
    journey_status_label.text = _journey_status_text(
        journey_state,
        pickup_name,
        destination_name,
        bool(snapshot.get("arrival_certified", false)),
        boarding_confirmed
    )
    journey_progress.visible = tracking_journey
    journey_progress.value = _journey_progress(
        journey_state,
        bool(snapshot.get("arrival_certified", false)),
        boarding_confirmed
    )
    board_button.visible = journey_state == JourneyStates.BOARDING \
        && bool(snapshot.get("arrival_certified", false)) \
        && not boarding_confirmed
    board_button.disabled = _action_in_flight || blocked
    exit_button.visible = journey_state == JourneyStates.DROPOFF \
        && bool(snapshot.get("arrival_certified", false))
    exit_button.disabled = _action_in_flight || blocked

    request_button.disabled = blocked || busy || _request_in_flight
    if blocked:
        service_status_label.text = "Service blocked â€” reservation retained."
    elif busy:
        service_status_label.text = "Shuttle occupied â€” finish the current journey."
    elif journey_state == "IDLE":
        service_status_label.text = "Service available."
    else:
        service_status_label.text = "Service status unavailable."
    service_status_label.visible = true
    if journey_state == "IDLE" && bool(snapshot.get("arrival_certified", false)):
        feedback_label.text = "Journey completed; shuttle returned to service."

func show_booking_result(result: Dictionary, snapshot: Dictionary) -> void:
    _request_in_flight = false
    update_service_snapshot(snapshot)
    if result.get("valid", false):
        feedback_label.text = "Request accepted. The shuttle is on its way."
        return

    var reason: String = str(result.get("failure_reason", "booking_rejected"))
    if snapshot.get("blocked", false) || reason == "journey_blocked":
        feedback_label.text = "Request rejected: service is blocked; the reservation is retained."
    elif reason in ["shuttle_occupied", "journey_not_idle"]:
        feedback_label.text = "Request rejected: the shuttle is busy with another journey."
    else:
        feedback_label.text = "Request rejected: %s." % _humanize_failure(reason)

func show_action_result(action: String, result: Dictionary, snapshot: Dictionary) -> void:
    _action_in_flight = false
    update_service_snapshot(snapshot)
    if result.get("valid", false):
        if action == "boarding":
            feedback_label.text = "Boarding confirmed; traveling to destination."
        elif snapshot.get("service_state", "") == JourneyStates.IDLE \
            && snapshot.get("active_booking", {}).is_empty() \
            && bool(snapshot.get("arrival_certified", false)):
            feedback_label.text = "Journey completed; shuttle returned to service."
        else:
            feedback_label.text = "Passenger exit confirmed; shuttle is returning to service."
        return
    feedback_label.text = "%s not accepted: %s." % [
        "Boarding" if action == "boarding" else "Passenger exit",
        _humanize_failure(str(result.get("failure_reason", "action_rejected")))
    ]

func has_keyboard_focus() -> bool:
    var focus_owner: Control = get_viewport().gui_get_focus_owner()
    return focus_owner != null && is_ancestor_of(focus_owner)

func _on_request_pressed() -> void:
    if request_button.disabled || _request_in_flight:
        return
    var pickup_station_id: String = _selected_station_id(pickup_selector)
    var destination_station_id: String = _selected_station_id(destination_selector)
    if pickup_station_id.is_empty() || destination_station_id.is_empty():
        feedback_label.text = "Choose a pickup and destination station."
        return
    if pickup_station_id == destination_station_id:
        feedback_label.text = "Pickup and destination must be different stops."
        return
    _request_in_flight = true
    request_button.disabled = true
    feedback_label.text = "Sending request to shuttle coordinator..."
    booking_requested.emit(pickup_station_id, destination_station_id)

func _on_board_pressed() -> void:
    if not board_button.visible || board_button.disabled || _action_in_flight:
        return
    _action_in_flight = true
    board_button.disabled = true
    boarding_requested.emit()

func _on_exit_pressed() -> void:
    if not exit_button.visible || exit_button.disabled || _action_in_flight:
        return
    _action_in_flight = true
    exit_button.disabled = true
    passenger_exit_requested.emit()

func _selected_station_id(selector: OptionButton) -> String:
    var selected_index: int = selector.selected
    if selected_index < 0:
        return ""
    return str(selector.get_item_metadata(selected_index))

func _humanize_failure(reason: String) -> String:
    return reason.replace("_", " ").capitalize()

func _station_name(station_id: String) -> String:
    if station_id.is_empty() || _network == null:
        return "â€”"
    var station: Dictionary = _network.get_station(station_id)
    return str(station.get("display_name", station_id)).replace(" Stop", "")

func _journey_status_text(
    journey_state: String,
    pickup_name: String,
    destination_name: String,
    arrival_certified: bool,
    boarding_confirmed: bool = false
) -> String:
    match journey_state:
        JourneyStates.ASSIGNED:
            return "Shuttle assigned: %s â†’ %s" % [pickup_name, destination_name]
        JourneyStates.TO_PICKUP:
            return "Traveling to pickup: %s" % pickup_name
        JourneyStates.BOARDING:
            if boarding_confirmed:
                return "Passenger boarded at %s â€” awaiting destination (Phase 5B.2B)." % pickup_name
            return "Waiting for boarding at %s%s" % [
                pickup_name,
                " (arrival certified)" if arrival_certified else ""
            ]
        JourneyStates.TO_DESTINATION:
            return "Traveling to destination: %s" % destination_name
        JourneyStates.DROPOFF:
            return "Waiting for passenger exit at %s%s" % [
                destination_name,
                " (arrival certified)" if arrival_certified else ""
            ]
        JourneyStates.RETURNING_TO_SERVICE:
            return "Returning to North Gate service."
        JourneyStates.BLOCKED:
            return "Journey blocked â€” reservation retained."
        JourneyStates.IDLE:
            if arrival_certified:
                return "Journey completed; shuttle is available."
            return "Choose your pickup and destination."
    return "Journey status unavailable."

func _journey_progress(
    journey_state: String,
    arrival_certified: bool,
    boarding_confirmed: bool = false
) -> float:
    if journey_state == JourneyStates.IDLE:
        return 100.0 if arrival_certified else 0.0
    match journey_state:
        JourneyStates.ASSIGNED:
            return 10.0
        JourneyStates.TO_PICKUP:
            return 25.0
        JourneyStates.BOARDING:
            return 45.0 if boarding_confirmed else 40.0
        JourneyStates.TO_DESTINATION:
            return 55.0
        JourneyStates.DROPOFF:
            return 70.0
        JourneyStates.RETURNING_TO_SERVICE:
            return 85.0
        JourneyStates.BLOCKED:
            return 0.0
    return 0.0
