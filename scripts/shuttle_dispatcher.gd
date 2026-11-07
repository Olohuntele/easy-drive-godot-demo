extends RefCounted

## Validates and reserves one passenger booking; it never advances vehicle motion.
const CampusNetwork = preload("res://scripts/campus_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")

const DEFAULT_SERVICE_STATION_ID: String = "north_gate"

var service_station_id: String
var _network: RefCounted
var _planner: RefCounted
var _active_booking: Dictionary = {}
var _next_booking_number: int = 1

func _init(
    network: RefCounted = null,
    route_planner: RefCounted = null,
    service_station: String = DEFAULT_SERVICE_STATION_ID
) -> void:
    _network = CampusNetwork.new() if network == null else network
    _planner = CampusRoutePlanner.new(_network) if route_planner == null else route_planner
    service_station_id = service_station

## Reserves a booking only if pickup, passenger, and return legs are all available.
func request_booking(origin_station_id: String, destination_station_id: String) -> Dictionary:
    if not _active_booking.is_empty():
        return _failure("shuttle_occupied")
    if _network == null || _network.get_station(service_station_id).is_empty():
        return _failure("unknown_service_station", {"station_id": service_station_id})
    if _network.get_station(origin_station_id).is_empty():
        return _failure("unknown_origin_station", {"station_id": origin_station_id})
    if _network.get_station(destination_station_id).is_empty():
        return _failure("unknown_destination_station", {"station_id": destination_station_id})
    if origin_station_id == destination_station_id:
        return _failure("same_station_not_allowed", {"station_id": origin_station_id})

    var pickup_route: Dictionary = _plan_or_stationary(service_station_id, origin_station_id)
    if not pickup_route.get("valid", false):
        return _failure("pickup_route_unavailable", {"route_failure": pickup_route})
    var passenger_route: Dictionary = _plan_or_stationary(
        origin_station_id,
        destination_station_id
    )
    if not passenger_route.get("valid", false):
        return _failure("destination_route_unavailable", {"route_failure": passenger_route})
    var return_route: Dictionary = _plan_or_stationary(
        destination_station_id,
        service_station_id
    )
    if not return_route.get("valid", false):
        return _failure("return_route_unavailable", {"route_failure": return_route})

    var booking_id: String = "booking-%04d" % _next_booking_number
    _next_booking_number += 1
    var booking := {
        "booking_id": booking_id,
        "origin_station_id": origin_station_id,
        "destination_station_id": destination_station_id,
        "service_station_id": service_station_id,
        "routes": {
            "pickup": pickup_route,
            "destination": passenger_route,
            "return": return_route
        }
    }
    _active_booking = booking.duplicate(true)
    return {
        "valid": true,
        "failure_reason": "",
        "booking": booking.duplicate(true)
    }

func has_active_booking() -> bool:
    return not _active_booking.is_empty()

func get_active_booking() -> Dictionary:
    return _active_booking.duplicate(true)

## Releases only with PassengerJourney's verified service-return certificate.
func complete_booking(booking_id: String, return_certificate: Dictionary = {}) -> Dictionary:
    if _active_booking.is_empty():
        return _failure("no_active_booking")
    if booking_id != str(_active_booking.get("booking_id", "")):
        return _failure("booking_id_mismatch")
    if not return_certificate.get("valid", false) \
        || return_certificate.get("leg", "") != "return" \
        || return_certificate.get("station_id", "") != service_station_id \
        || not return_certificate.get("localized", false) \
        || not _is_finite_number(return_certificate.get("speed_mps", null)) \
        || float(return_certificate.speed_mps) > 0.1:
        return _failure("certified_service_return_required")
    _active_booking.clear()
    return {"valid": true, "failure_reason": ""}

func _plan_or_stationary(origin_station_id: String, destination_station_id: String) -> Dictionary:
    if origin_station_id == destination_station_id:
        return {
            "valid": true,
            "stationary": true,
            "origin_station_id": origin_station_id,
            "destination_station_id": destination_station_id,
            "segment_ids": PackedStringArray(),
            "centerline_points": []
        }
    if _planner == null:
        return {"valid": false, "failure_reason": "route_planner_unavailable"}
    var route: Dictionary = _planner.plan_route(origin_station_id, destination_station_id)
    if not route.get("valid", false):
        return route.duplicate(true)
    if route.get("origin_station_id", "") != origin_station_id \
        || route.get("destination_station_id", "") != destination_station_id \
        || route.get("stationary", false):
        return {
            "valid": false,
            "failure_reason": "route_planner_returned_inconsistent_endpoints"
        }
    return route.duplicate(true)

func _failure(reason: String, details: Dictionary = {}) -> Dictionary:
    return {
        "valid": false,
        "failure_reason": reason,
        "details": details
    }

func _is_finite_number(value: Variant) -> bool:
    return (value is int || value is float) && is_finite(float(value))
