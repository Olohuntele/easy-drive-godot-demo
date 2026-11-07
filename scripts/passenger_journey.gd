extends RefCounted

## Coordinates service state and route handoffs; it never mutates vehicle state or drives.
const POSITION_TOLERANCE_M: float = 1.0
const ARRIVAL_SPEED_TOLERANCE_MPS: float = 0.1
const HEADING_TOLERANCE_RAD: float = deg_to_rad(12.0)
const POSE_HANDOFF_TOLERANCE_M: float = 0.0001
const HEADING_HANDOFF_TOLERANCE_RAD: float = 0.000001
const SPEED_HANDOFF_TOLERANCE_MPS: float = 0.0001

const IDLE: String = "IDLE"
const ASSIGNED: String = "ASSIGNED"
const TO_PICKUP: String = "TO_PICKUP"
const BOARDING: String = "BOARDING"
const TO_DESTINATION: String = "TO_DESTINATION"
const DROPOFF: String = "DROPOFF"
const RETURNING_TO_SERVICE: String = "RETURNING_TO_SERVICE"
const BLOCKED: String = "BLOCKED"

var state: String = IDLE
var blocked_reason: String = ""
var _network: RefCounted
var _dispatcher: RefCounted
var _booking: Dictionary = {}
var _pending_route: Dictionary = {}
var _installed_leg: String = ""
var _boarding_confirmed: bool = false
var _passenger_exit_confirmed: bool = false
var _next_request_number: int = 1

func _init(network: RefCounted, dispatcher: RefCounted) -> void:
    _network = network
    _dispatcher = dispatcher

## Accepts only a valid reservation currently held by the dispatcher.
func assign_booking(booking_result: Dictionary) -> Dictionary:
    if state != IDLE:
        return _failure("journey_not_idle")
    if not booking_result.get("valid", false) \
        || not booking_result.get("booking", null) is Dictionary:
        return _failure("invalid_booking_result")
    var booking: Dictionary = booking_result.booking
    var booking_id: String = str(booking.get("booking_id", ""))
    var routes: Variant = booking.get("routes", null)
    if booking_id.is_empty() || not routes is Dictionary \
        || not _routes_are_valid(routes):
        return _failure("invalid_booking_routes")
    if _dispatcher == null \
        || _dispatcher.get_active_booking().get("booking_id", "") != booking_id:
        return _failure("booking_not_reserved")
    _booking = booking.duplicate(true)
    state = ASSIGNED
    blocked_reason = ""
    _boarding_confirmed = false
    _passenger_exit_confirmed = false
    return {"valid": true, "failure_reason": "", "state": state}

## Returns a route-install request after validating the real pose and heading.
## Supply a pose-planned route to avoid assuming later journey-leg headings in advance.
## The caller installs its centerline in RouteNavigator and then confirms the result.
func request_next_route_installation(
    vehicle_state: Dictionary,
    route_override: Dictionary = {}
) -> Dictionary:
    var leg: String
    match state:
        ASSIGNED:
            leg = "pickup"
        BOARDING:
            if not _boarding_confirmed:
                return _failure("boarding_confirmation_required")
            leg = "destination"
        DROPOFF:
            if not _passenger_exit_confirmed:
                return _failure("passenger_exit_confirmation_required")
            leg = "return"
        _:
            return _failure("route_installation_not_allowed_in_current_state")

    var routes: Dictionary = _booking.get("routes", {})
    var route: Dictionary = route_override if not route_override.is_empty() \
        else routes.get(leg, {})
    if not route.get("valid", false):
        return _block("required_%s_route_unavailable" % leg)
    var target_station_id: String = _target_station_for_leg(leg)
    if route.get("origin_station_id", "") != _source_station_for_leg(leg) \
        || route.get("destination_station_id", "") != target_station_id:
        return _block("route_destination_mismatch")

    var already_at_pickup: bool = leg == "pickup" \
        && _vehicle_is_at_station(vehicle_state, target_station_id)
    if route.get("stationary", false) || already_at_pickup:
        var current_state: String = state
        var stationary_arrival: Dictionary = certify_station_arrival(
            leg,
            target_station_id,
            {},
            vehicle_state,
            true
        )
        stationary_arrival["stationary_route"] = true
        stationary_arrival["previous_state"] = current_state
        return stationary_arrival

    var pose_validation: Dictionary = _validate_vehicle_state(vehicle_state)
    if not pose_validation.valid:
        return _block(str(pose_validation.reason))
    var centerline_value: Variant = route.get("centerline_points", null)
    if not centerline_value is Array || centerline_value.size() < 2:
        return _block("route_centerline_invalid")
    var centerline: Array = centerline_value
    if not centerline[0] is Vector2 || not centerline[1] is Vector2 \
        || not centerline[0].is_finite() || not centerline[1].is_finite():
        return _block("route_start_tangent_invalid")
    var first_edge: Vector2 = centerline[1] - centerline[0]
    if first_edge.length_squared() <= 0.000001:
        return _block("route_start_tangent_invalid")

    var position_error_m: float = vehicle_state.position_xz.distance_to(centerline[0])
    if position_error_m > POSITION_TOLERANCE_M:
        return _block("route_start_pose_incompatible")
    var expected_heading_rad: float = atan2(first_edge.x, first_edge.y)
    var heading_error_rad: float = absf(wrapf(
        float(vehicle_state.heading_rad) - expected_heading_rad,
        -PI,
        PI
    ))
    if heading_error_rad > HEADING_TOLERANCE_RAD:
        return _block("route_start_heading_incompatible")

    var request_id: String = "route-request-%04d" % _next_request_number
    _next_request_number += 1
    _pending_route = {
        "request_id": request_id,
        "leg": leg,
        "target_station_id": target_station_id,
        "route": route.duplicate(true),
        "vehicle_state_before": vehicle_state.duplicate(true)
    }
    return {
        "valid": true,
        "failure_reason": "",
        "request_id": request_id,
        "leg": leg,
        "destination_station_id": target_station_id,
        "centerline_points": centerline.duplicate(),
        "position_error_m": position_error_m,
        "heading_error_rad": heading_error_rad
    }

## Confirms the external RouteNavigator handoff; rejects any vehicle pose mutation.
func confirm_route_installation(request_id: String, installation_result: Dictionary) -> Dictionary:
    if _pending_route.is_empty() || request_id != _pending_route.get("request_id", ""):
        return _block("route_installation_request_mismatch")
    if not installation_result.get("installed", false):
        return _block("route_installation_failed")
    var before: Dictionary = _pending_route.vehicle_state_before
    var after_value: Variant = installation_result.get("vehicle_state", null)
    if not after_value is Dictionary:
        return _block("route_installation_missing_vehicle_state")
    var after: Dictionary = after_value
    var before_validation: Dictionary = _validate_vehicle_state(before)
    var after_validation: Dictionary = _validate_vehicle_state(after)
    if not before_validation.valid || not after_validation.valid:
        return _block("route_installation_invalid_vehicle_state")
    if before.position_xz.distance_to(after.position_xz) > POSE_HANDOFF_TOLERANCE_M \
        || absf(wrapf(
            float(before.heading_rad) - float(after.heading_rad),
            -PI,
            PI
        )) > HEADING_HANDOFF_TOLERANCE_RAD \
        || absf(float(before.speed_mps) - float(after.speed_mps)) \
            > SPEED_HANDOFF_TOLERANCE_MPS:
        return _block("route_installation_changed_vehicle_state")

    var leg: String = str(_pending_route.leg)
    _installed_leg = leg
    _pending_route.clear()
    match leg:
        "pickup":
            state = TO_PICKUP
        "destination":
            state = TO_DESTINATION
        "return":
            state = RETURNING_TO_SERVICE
        _:
            return _block("unknown_route_leg")
    return {"valid": true, "failure_reason": "", "state": state, "leg": leg}

## Single service-layer arrival certification path for pickup, destination, and return.
func certify_station_arrival(
    leg: String,
    station_id: String,
    route_following_result: Dictionary,
    vehicle_state: Dictionary,
    stationary: bool = false
) -> Dictionary:
    var expected_state: String = _expected_arrival_state(leg)
    if expected_state.is_empty() || (state != expected_state
        && not (stationary && leg == "pickup" && state == ASSIGNED)
        && not (stationary && leg == "return" && state == DROPOFF)):
        return _failure("arrival_not_expected_in_current_state")
    if stationary && leg == "return" && state == DROPOFF && not _passenger_exit_confirmed:
        return _failure("passenger_exit_confirmation_required")
    var expected_station_id: String = _target_station_for_leg(leg)
    if station_id != expected_station_id:
        return _block("wrong_station_arrival")
    if not stationary && _installed_leg != leg:
        return _block("arrival_without_installed_route")
    if stationary && leg != "pickup" && leg != "return":
        return _failure("stationary_arrival_not_allowed_for_leg")
    if route_following_result.get("manual_intervention", false) \
        || str(route_following_result.get("reason", "")) == "manual_intervention_seen":
        return _block("manual_intervention")
    if not stationary:
        if route_following_result.get("status", "") != "ARRIVED" \
            || not route_following_result.get("arrival_certified", false):
            return _block("route_arrival_not_certified")
        if not route_following_result.get("localized", false):
            return _block("arrival_localization_invalid")

    var vehicle_validation: Dictionary = _validate_vehicle_state(vehicle_state)
    if not vehicle_validation.valid:
        return _block(str(vehicle_validation.reason))
    if not stationary && not route_following_result.get("localized", false):
        return _block("arrival_localization_invalid")
    var stop: Dictionary = _network.get_station_stop(expected_station_id)
    if not stop.get("valid", false) || not stop.get("position_xz", null) is Vector2:
        return _block("station_stop_unavailable")
    var station_error_m: float = vehicle_state.position_xz.distance_to(stop.position_xz)
    if station_error_m > POSITION_TOLERANCE_M:
        return _block("vehicle_not_at_expected_station")
    if float(vehicle_state.speed_mps) > ARRIVAL_SPEED_TOLERANCE_MPS:
        return _block("vehicle_not_stopped_at_station")

    if stationary && state == ASSIGNED:
        state = TO_PICKUP
    elif stationary && state == DROPOFF:
        state = RETURNING_TO_SERVICE
    _installed_leg = ""
    if leg == "pickup":
        state = BOARDING
        return _arrival_success(leg, station_id, station_error_m)
    if leg == "destination":
        state = DROPOFF
        return _arrival_success(leg, station_id, station_error_m)
    if leg == "return":
        var return_certificate: Dictionary = _arrival_success(
            leg,
            station_id,
            station_error_m
        )
        return_certificate["localized"] = true
        return_certificate["speed_mps"] = float(vehicle_state.speed_mps)
        var release_result: Dictionary = _dispatcher.complete_booking(
            str(_booking.get("booking_id", "")),
            return_certificate
        )
        if not release_result.get("valid", false):
            return _block("booking_release_failed")
        state = IDLE
        _booking.clear()
        _boarding_confirmed = false
        _passenger_exit_confirmed = false
        return _arrival_success(leg, station_id, station_error_m)
    return _block("unknown_arrival_leg")

func confirm_passenger_boarding() -> Dictionary:
    if state != BOARDING:
        return _failure("boarding_confirmation_wrong_state")
    if _boarding_confirmed:
        return _failure("duplicate_boarding_confirmation")
    _boarding_confirmed = true
    return {"valid": true, "failure_reason": "", "state": state}

func confirm_passenger_exit() -> Dictionary:
    if state != DROPOFF:
        return _failure("passenger_exit_confirmation_wrong_state")
    if _passenger_exit_confirmed:
        return _failure("duplicate_passenger_exit_confirmation")
    _passenger_exit_confirmed = true
    return {"valid": true, "failure_reason": "", "state": state}

func report_manual_intervention() -> Dictionary:
    if state == IDLE:
        return _failure("no_active_journey")
    return _block("manual_intervention")

## Blocks an active service when its route executor reports a terminal safety failure.
func block_journey(reason: String) -> Dictionary:
    if state == IDLE:
        return _failure("no_active_journey")
    if reason.strip_edges().is_empty():
        return _failure("block_reason_required")
    return _block(reason.strip_edges())

func get_active_booking() -> Dictionary:
    return _booking.duplicate(true)

func _routes_are_valid(routes: Dictionary) -> bool:
    for leg in ["pickup", "destination", "return"]:
        var route: Variant = routes.get(leg, null)
        if not route is Dictionary || not route.get("valid", false):
            return false
    return true

func _target_station_for_leg(leg: String) -> String:
    match leg:
        "pickup":
            return str(_booking.get("origin_station_id", ""))
        "destination":
            return str(_booking.get("destination_station_id", ""))
        "return":
            return str(_booking.get("service_station_id", ""))
    return ""

func _source_station_for_leg(leg: String) -> String:
    match leg:
        "pickup":
            return str(_booking.get("service_station_id", ""))
        "destination":
            return str(_booking.get("origin_station_id", ""))
        "return":
            return str(_booking.get("destination_station_id", ""))
    return ""

func _expected_arrival_state(leg: String) -> String:
    match leg:
        "pickup":
            return TO_PICKUP
        "destination":
            return TO_DESTINATION
        "return":
            return RETURNING_TO_SERVICE
    return ""

func _validate_vehicle_state(vehicle_state: Dictionary) -> Dictionary:
    var position: Variant = vehicle_state.get("position_xz", null)
    if not position is Vector2 || not position.is_finite() \
        || not _is_finite_number(vehicle_state.get("heading_rad", null)) \
        || not _is_finite_number(vehicle_state.get("speed_mps", null)) \
        || float(vehicle_state.speed_mps) < 0.0:
        return {"valid": false, "reason": "invalid_vehicle_state"}
    if not vehicle_state.get("localized", false):
        return {"valid": false, "reason": "arrival_localization_invalid"}
    if vehicle_state.get("manual_intervention", false):
        return {"valid": false, "reason": "manual_intervention"}
    if float(vehicle_state.speed_mps) > ARRIVAL_SPEED_TOLERANCE_MPS:
        return {"valid": false, "reason": "vehicle_not_stopped_at_station"}
    return {"valid": true, "reason": ""}

func _vehicle_is_at_station(vehicle_state: Dictionary, station_id: String) -> bool:
    var position: Variant = vehicle_state.get("position_xz", null)
    if not position is Vector2 || not position.is_finite():
        return false
    var stop: Dictionary = _network.get_station_stop(station_id)
    if not stop.get("valid", false) || not stop.get("position_xz", null) is Vector2:
        return false
    return position.distance_to(stop.position_xz) <= POSITION_TOLERANCE_M

func _is_finite_number(value: Variant) -> bool:
    return (value is int || value is float) && is_finite(float(value))

func _arrival_success(leg: String, station_id: String, error_m: float) -> Dictionary:
    return {
        "valid": true,
        "failure_reason": "",
        "leg": leg,
        "station_id": station_id,
        "station_error_m": error_m,
        "state": state
    }

func _block(reason: String) -> Dictionary:
    state = BLOCKED
    blocked_reason = reason
    _pending_route.clear()
    return _failure(reason)

func _failure(reason: String) -> Dictionary:
    return {
        "valid": false,
        "failure_reason": reason,
        "state": state
    }
