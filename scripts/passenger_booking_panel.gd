extends PanelContainer

signal booking_requested(pickup_station_id: String, destination_station_id: String)

const PANEL_SIZE: Vector2 = Vector2(286.0, 264.0)

var pickup_selector: OptionButton
var destination_selector: OptionButton
var request_button: Button
var service_status_label: Label
var feedback_label: Label
var _request_in_flight: bool = false

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
    title.text = "EASY DRIVE — BOOK A SHUTTLE"
    title.add_theme_font_size_override("font_size", 15)
    content.add_child(title)

    var pickup_label := Label.new()
    pickup_label.text = "Pickup station"
    pickup_label.add_theme_font_size_override("font_size", 12)
    content.add_child(pickup_label)
    pickup_selector = OptionButton.new()
    pickup_selector.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    pickup_selector.focus_mode = Control.FOCUS_ALL
    content.add_child(pickup_selector)

    var destination_label := Label.new()
    destination_label.text = "Destination"
    destination_label.add_theme_font_size_override("font_size", 12)
    content.add_child(destination_label)
    destination_selector = OptionButton.new()
    destination_selector.size_flags_horizontal = Control.SIZE_EXPAND_FILL
    destination_selector.focus_mode = Control.FOCUS_ALL
    content.add_child(destination_selector)

    request_button = Button.new()
    request_button.text = "Request Shuttle"
    request_button.focus_mode = Control.FOCUS_ALL
    request_button.pressed.connect(_on_request_pressed)
    content.add_child(request_button)

    service_status_label = Label.new()
    service_status_label.add_theme_font_size_override("font_size", 12)
    service_status_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
    content.add_child(service_status_label)

    feedback_label = Label.new()
    feedback_label.add_theme_font_size_override("font_size", 12)
    feedback_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
    content.add_child(feedback_label)

    var controls_hint := Label.new()
    controls_hint.text = "B demo booking · E board · X exit · Esc releases UI focus"
    controls_hint.add_theme_font_size_override("font_size", 10)
    controls_hint.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
    content.add_child(controls_hint)

func configure(network: RefCounted, initial_snapshot: Dictionary) -> void:
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
    request_button.disabled = blocked || busy || _request_in_flight
    if blocked:
        service_status_label.text = "Service blocked — reservation retained."
    elif busy:
        service_status_label.text = "Shuttle occupied — finish the current journey."
    elif journey_state == "IDLE":
        service_status_label.text = "Service available."
    else:
        service_status_label.text = "Service status unavailable."

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

func _selected_station_id(selector: OptionButton) -> String:
    var selected_index: int = selector.selected
    if selected_index < 0:
        return ""
    return str(selector.get_item_metadata(selected_index))

func _humanize_failure(reason: String) -> String:
    return reason.replace("_", " ").capitalize()
