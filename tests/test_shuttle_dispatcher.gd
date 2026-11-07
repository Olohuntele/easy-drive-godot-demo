extends SceneTree

const CampusNetwork = preload("res://scripts/campus_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const ShuttleDispatcher = preload("res://scripts/shuttle_dispatcher.gd")

class SelectiveFailurePlanner extends RefCounted:
    var delegate: RefCounted
    var failed_pair: String = ""

    func _init(network: RefCounted) -> void:
        delegate = CampusRoutePlanner.new(network)

    func plan_route(origin_station_id: String, destination_station_id: String) -> Dictionary:
        if "%s>%s" % [origin_station_id, destination_station_id] == failed_pair:
            return {"valid": false, "failure_reason": "test_route_unavailable"}
        return delegate.plan_route(origin_station_id, destination_station_id)

var failures: int = 0
var assertions: int = 0
var network: RefCounted

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    network = CampusNetwork.new()
    _test_booking_validation_and_occupancy()
    _test_required_routes_and_return_to_service()
    _test_determinism_and_copy_protection()
    if failures == 0:
        print("ShuttleDispatcher tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d ShuttleDispatcher assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _test_booking_validation_and_occupancy() -> void:
    var dispatcher = ShuttleDispatcher.new(network)
    var unknown_origin: Dictionary = dispatcher.request_booking("missing", "library")
    _expect(
        not unknown_origin.valid && unknown_origin.failure_reason == "unknown_origin_station",
        "unknown pickup station is rejected"
    )
    var unknown_destination: Dictionary = dispatcher.request_booking("library", "missing")
    _expect(
        not unknown_destination.valid
            && unknown_destination.failure_reason == "unknown_destination_station",
        "unknown destination station is rejected"
    )
    var same_station: Dictionary = dispatcher.request_booking("library", "library")
    _expect(
        not same_station.valid && same_station.failure_reason == "same_station_not_allowed",
        "same-station passenger booking is rejected"
    )
    _expect(not dispatcher.has_active_booking(), "invalid bookings do not occupy the shuttle")

    var accepted: Dictionary = dispatcher.request_booking("library", "sports_hall")
    _expect(accepted.valid, "valid station booking is accepted")
    _expect(dispatcher.has_active_booking(), "accepted booking reserves the single shuttle")
    var occupied: Dictionary = dispatcher.request_booking("sports_hall", "library")
    _expect(
        not occupied.valid && occupied.failure_reason == "shuttle_occupied",
        "a second booking is rejected while the shuttle is occupied"
    )
    _expect(
        not dispatcher.complete_booking("wrong-booking").valid,
        "a mismatched booking ID cannot release the reservation"
    )
    _expect(
        not dispatcher.complete_booking(accepted.booking.booking_id).valid,
        "an uncertified return cannot release the reservation"
    )
    var return_certificate := {
        "valid": true,
        "leg": "return",
        "station_id": "north_gate",
        "localized": true,
        "speed_mps": 0.0
    }
    _expect(
        dispatcher.complete_booking(
            accepted.booking.booking_id,
            return_certificate
        ).valid,
        "a matching certified return releases the shuttle"
    )
    _expect(not dispatcher.has_active_booking(), "shuttle is available after booking completion")

    var invalid_service_dispatcher = ShuttleDispatcher.new(network, null, "unknown_service")
    var invalid_service: Dictionary = invalid_service_dispatcher.request_booking(
        "library",
        "sports_hall"
    )
    _expect(
        not invalid_service.valid
            && invalid_service.failure_reason == "unknown_service_station",
        "unknown return-to-service station is rejected"
    )

func _test_required_routes_and_return_to_service() -> void:
    for failed_pair in ["north_gate>library", "library>sports_hall", "sports_hall>north_gate"]:
        var failing_planner = SelectiveFailurePlanner.new(network)
        failing_planner.failed_pair = failed_pair
        var dispatcher = ShuttleDispatcher.new(network, failing_planner)
        var result: Dictionary = dispatcher.request_booking("library", "sports_hall")
        _expect(
            not result.valid && not dispatcher.has_active_booking(),
            "booking is rejected without reservation when route %s is unavailable"
                % failed_pair
        )
        _expect(
            result.failure_reason in [
                "pickup_route_unavailable",
                "destination_route_unavailable",
                "return_route_unavailable"
            ],
            "unavailable %s route failure is explicit" % failed_pair
        )

    var dispatcher = ShuttleDispatcher.new(network)
    var result: Dictionary = dispatcher.request_booking("library", "sports_hall")
    _expect(result.valid, "booking includes all three validated legs")
    if result.valid:
        var routes: Dictionary = result.booking.routes
        _expect(
            routes.pickup.origin_station_id == "north_gate"
                && routes.pickup.destination_station_id == "library",
            "pickup route starts at service station and ends at the requested pickup"
        )
        _expect(
            routes.destination.origin_station_id == "library"
                && routes.destination.destination_station_id == "sports_hall",
            "passenger route joins the requested pickup and destination"
        )
        _expect(
            routes.return.origin_station_id == "sports_hall"
                && routes.return.destination_station_id == "north_gate",
            "return route ends at the configured service station"
        )

    var at_service_dispatcher = ShuttleDispatcher.new(network)
    var at_service: Dictionary = at_service_dispatcher.request_booking(
        "north_gate",
        "library"
    )
    _expect(
        at_service.valid && at_service.booking.routes.pickup.stationary,
        "pickup at the current service station is represented as stationary, not a drive route"
    )
    _expect(
        at_service.booking.routes.pickup.centerline_points.is_empty(),
        "stationary pickup does not create degenerate route geometry"
    )

func _test_determinism_and_copy_protection() -> void:
    var first_dispatcher = ShuttleDispatcher.new(network)
    var second_dispatcher = ShuttleDispatcher.new(network)
    var first: Dictionary = first_dispatcher.request_booking("library", "sports_hall")
    var second: Dictionary = second_dispatcher.request_booking("library", "sports_hall")
    _expect(
        first.booking.booking_id == second.booking.booking_id
            && first.booking.routes.destination.segment_ids
                == second.booking.routes.destination.segment_ids,
        "identical fresh dispatchers produce deterministic booking and route results"
    )
    var exposed_booking: Dictionary = first_dispatcher.get_active_booking()
    exposed_booking.routes.destination.centerline_points.clear()
    _expect(
        not first_dispatcher.get_active_booking().routes.destination.centerline_points.is_empty(),
        "returned booking data cannot mutate the dispatcher's reserved route"
    )
