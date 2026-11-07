extends SceneTree

const CampusNetwork = preload("res://scripts/campus_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const PassengerJourney = preload("res://scripts/passenger_journey.gd")
const ShuttleDispatcher = preload("res://scripts/shuttle_dispatcher.gd")

var failures: int = 0
var assertions: int = 0
var network: RefCounted

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    network = CampusNetwork.new()
    _test_complete_journey_and_confirmations()
    _test_stationary_pickup()
    _test_safe_blocking_failures()
    _test_pose_handoff_and_request_validation()
    _test_deterministic_repeated_journeys()
    if failures == 0:
        print("PassengerJourney tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d PassengerJourney assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _new_trip(
    origin_station_id: String = "library",
    destination_station_id: String = "sports_hall",
    service_station_id: String = "north_gate"
) -> Dictionary:
    var planner = CampusRoutePlanner.new(network)
    var dispatcher = ShuttleDispatcher.new(network, planner, service_station_id)
    var booking_result: Dictionary = dispatcher.request_booking(
        origin_station_id,
        destination_station_id
    )
    var journey = PassengerJourney.new(network, dispatcher)
    var assigned: Dictionary = journey.assign_booking(booking_result)
    return {
        "dispatcher": dispatcher,
        "journey": journey,
        "booking_result": booking_result,
        "assigned": assigned
    }

func _vehicle_at_route_start(route: Dictionary) -> Dictionary:
    var points: Array = route.centerline_points
    var tangent: Vector2 = (points[1] - points[0]).normalized()
    return {
        "position_xz": points[0],
        "heading_rad": atan2(tangent.x, tangent.y),
        "speed_mps": 0.0,
        "localized": true,
        "manual_intervention": false
    }

func _vehicle_at_station(station_id: String) -> Dictionary:
    var stop: Dictionary = network.get_station_stop(station_id)
    return {
        "position_xz": stop.position_xz,
        "heading_rad": 0.0,
        "speed_mps": 0.0,
        "localized": true,
        "manual_intervention": false
    }

func _arrived_result() -> Dictionary:
    return {
        "status": "ARRIVED",
        "arrival_certified": true,
        "localized": true,
        "reason": "route_complete"
    }

func _install_leg(journey: RefCounted, route: Dictionary) -> Dictionary:
    var vehicle_state: Dictionary = _vehicle_at_route_start(route)
    var request: Dictionary = journey.request_next_route_installation(vehicle_state)
    if not request.get("valid", false):
        return request
    return journey.confirm_route_installation(
        str(request.request_id),
        {"installed": true, "vehicle_state": vehicle_state.duplicate(true)}
    )

func _test_complete_journey_and_confirmations() -> void:
    var trip: Dictionary = _new_trip()
    var journey: RefCounted = trip.journey
    var dispatcher: RefCounted = trip.dispatcher
    _expect(trip.booking_result.valid && trip.assigned.valid, "validated booking enters ASSIGNED")
    _expect(journey.state == PassengerJourney.ASSIGNED, "journey begins in ASSIGNED")
    _expect(
        not journey.confirm_passenger_boarding().valid,
        "boarding confirmation is rejected before pickup"
    )
    var pickup_route: Dictionary = trip.booking_result.booking.routes.pickup
    var pickup_install: Dictionary = _install_leg(journey, pickup_route)
    _expect(
        pickup_install.valid && journey.state == PassengerJourney.TO_PICKUP,
        "pickup leg starts only after successful route installation"
    )
    var pickup_pose: Dictionary = _vehicle_at_station("library")
    var pickup_arrival: Dictionary = journey.certify_station_arrival(
        "pickup",
        "library",
        _arrived_result(),
        pickup_pose
    )
    _expect(
        pickup_arrival.valid && journey.state == PassengerJourney.BOARDING,
        "certified arrival at the requested pickup permits boarding"
    )
    _expect(
        not journey.request_next_route_installation(pickup_pose).valid,
        "passenger route cannot be installed before simulated boarding confirmation"
    )
    _expect(
        journey.confirm_passenger_boarding().valid,
        "explicit simulated boarding confirmation is accepted"
    )
    _expect(
        not journey.confirm_passenger_boarding().valid,
        "duplicate boarding confirmation is rejected"
    )

    var destination_route: Dictionary = trip.booking_result.booking.routes.destination
    var destination_install: Dictionary = _install_leg(journey, destination_route)
    _expect(
        destination_install.valid && journey.state == PassengerJourney.TO_DESTINATION,
        "destination leg starts after boarding and successful route installation"
    )
    var destination_arrival: Dictionary = journey.certify_station_arrival(
        "destination",
        "sports_hall",
        _arrived_result(),
        _vehicle_at_station("sports_hall")
    )
    _expect(
        destination_arrival.valid && journey.state == PassengerJourney.DROPOFF,
        "certified arrival at the requested destination permits drop-off"
    )
    _expect(
        not journey.request_next_route_installation(_vehicle_at_station("sports_hall")).valid,
        "return route cannot be installed before simulated passenger exit confirmation"
    )
    _expect(
        journey.confirm_passenger_exit().valid,
        "explicit simulated passenger exit confirmation is accepted"
    )
    _expect(
        not journey.confirm_passenger_exit().valid,
        "duplicate passenger exit confirmation is rejected"
    )

    var return_route: Dictionary = trip.booking_result.booking.routes["return"]
    var return_install: Dictionary = _install_leg(journey, return_route)
    _expect(
        return_install.valid && journey.state == PassengerJourney.RETURNING_TO_SERVICE,
        "return-to-service leg starts after passenger exit and route installation"
    )
    var return_arrival: Dictionary = journey.certify_station_arrival(
        "return",
        "north_gate",
        _arrived_result(),
        _vehicle_at_station("north_gate")
    )
    _expect(
        return_arrival.valid && journey.state == PassengerJourney.IDLE,
        "certified service-station arrival completes the journey"
    )
    _expect(not dispatcher.has_active_booking(), "completed return releases the shuttle")
    _expect(journey.get_active_booking().is_empty(), "completed journey clears its booking")

func _test_stationary_pickup() -> void:
    var already_at_pickup_trip: Dictionary = _new_trip()
    var already_at_pickup: Dictionary = \
        already_at_pickup_trip.journey.request_next_route_installation(
            _vehicle_at_station("library")
        )
    _expect(
        already_at_pickup.valid && already_at_pickup.get("stationary_route", false)
            && already_at_pickup_trip.journey.state == PassengerJourney.BOARDING,
        "shuttle already at pickup is certified stationary despite a planned service drive leg"
    )

    var trip: Dictionary = _new_trip("library", "sports_hall", "library")
    var journey: RefCounted = trip.journey
    _expect(trip.assigned.valid, "booking with pickup at service station is assigned")
    var before: Dictionary = _vehicle_at_station("library")
    var request: Dictionary = journey.request_next_route_installation(before)
    _expect(
        request.valid && request.get("stationary_route", false)
            && journey.state == PassengerJourney.BOARDING,
        "at-pickup shuttle records validated stationary arrival without a drive route"
    )
    _expect(
        request.get("station_error_m", 0.0) == 0.0
            && journey.get_active_booking().booking_id == trip.booking_result.booking.booking_id,
        "stationary pickup validates the stop and keeps the booking active"
    )
    _expect(
        journey.get_active_booking().routes.pickup.centerline_points.is_empty(),
        "stationary pickup retains no degenerate centerline"
    )
    _expect(
        journey.confirm_passenger_boarding().valid,
        "stationary pickup still requires explicit boarding confirmation"
    )

    var service_destination_trip: Dictionary = _new_trip(
        "library",
        "north_gate",
        "north_gate"
    )
    var service_journey: RefCounted = service_destination_trip.journey
    var service_pickup: Dictionary = service_destination_trip.booking_result.booking.routes.pickup
    _install_leg(service_journey, service_pickup)
    service_journey.certify_station_arrival(
        "pickup",
        "library",
        _arrived_result(),
        _vehicle_at_station("library")
    )
    service_journey.confirm_passenger_boarding()
    var service_destination: Dictionary = \
        service_destination_trip.booking_result.booking.routes.destination
    _install_leg(service_journey, service_destination)
    var at_service_arrival: Dictionary = service_journey.certify_station_arrival(
        "destination",
        "north_gate",
        _arrived_result(),
        _vehicle_at_station("north_gate")
    )
    service_journey.confirm_passenger_exit()
    var stationary_return: Dictionary = service_journey.request_next_route_installation(
        _vehicle_at_station("north_gate")
    )
    _expect(
        at_service_arrival.valid && stationary_return.valid
            && stationary_return.get("stationary_route", false)
            && service_journey.state == PassengerJourney.IDLE,
        "passenger exit at the service station completes a certified stationary return"
    )

func _test_safe_blocking_failures() -> void:
    var wrong_station_trip: Dictionary = _start_pickup_leg()
    var wrong_station = wrong_station_trip.journey.certify_station_arrival(
        "pickup",
        "sports_hall",
        _arrived_result(),
        _vehicle_at_station("sports_hall")
    )
    _expect(
        not wrong_station.valid
            && wrong_station_trip.journey.state == PassengerJourney.BLOCKED
            && wrong_station_trip.journey.blocked_reason == "wrong_station_arrival",
        "arrival at a wrong station blocks the occupied journey"
    )
    _expect(
        wrong_station_trip.dispatcher.has_active_booking(),
        "blocked wrong-station journey retains its reservation"
    )

    var uncertified_trip: Dictionary = _start_pickup_leg()
    var uncertified_result := _arrived_result()
    uncertified_result.arrival_certified = false
    var uncertified = uncertified_trip.journey.certify_station_arrival(
        "pickup",
        "library",
        uncertified_result,
        _vehicle_at_station("library")
    )
    _expect(
        not uncertified.valid
            && uncertified_trip.journey.state == PassengerJourney.BLOCKED,
        "ARRIVED without controller certification cannot advance the journey"
    )

    var localization_trip: Dictionary = _start_pickup_leg()
    var lost_localization := _vehicle_at_station("library")
    lost_localization.localized = false
    var lost = localization_trip.journey.certify_station_arrival(
        "pickup",
        "library",
        _arrived_result(),
        lost_localization
    )
    _expect(
        not lost.valid
            && localization_trip.journey.state == PassengerJourney.BLOCKED
            && localization_trip.journey.blocked_reason == "arrival_localization_invalid",
        "lost localization blocks arrival certification"
    )

    var wrong_position_trip: Dictionary = _start_pickup_leg()
    var reported_pickup_elsewhere: Dictionary = wrong_position_trip.journey.certify_station_arrival(
        "pickup",
        "library",
        _arrived_result(),
        _vehicle_at_station("sports_hall")
    )
    _expect(
        not reported_pickup_elsewhere.valid
            && wrong_position_trip.journey.blocked_reason == "vehicle_not_at_expected_station",
        "correct station ID without physical stop proximity cannot certify arrival"
    )

    var moving_trip: Dictionary = _start_pickup_leg()
    var moving_vehicle := _vehicle_at_station("library")
    moving_vehicle.speed_mps = 0.5
    var moving_arrival: Dictionary = moving_trip.journey.certify_station_arrival(
        "pickup",
        "library",
        _arrived_result(),
        moving_vehicle
    )
    _expect(
        not moving_arrival.valid
            && moving_trip.journey.blocked_reason == "vehicle_not_stopped_at_station",
        "arrival at excessive speed cannot be certified"
    )

    var invalid_controller_localization_trip: Dictionary = _start_pickup_leg()
    var invalid_controller_localization := _arrived_result()
    invalid_controller_localization.localized = false
    var invalid_controller_localization_result: Dictionary = \
        invalid_controller_localization_trip.journey.certify_station_arrival(
            "pickup",
            "library",
            invalid_controller_localization,
            _vehicle_at_station("library")
        )
    _expect(
        not invalid_controller_localization_result.valid
            && invalid_controller_localization_trip.journey.blocked_reason
                == "arrival_localization_invalid",
        "controller-reported invalid localization blocks arrival certification"
    )

    var manual_trip: Dictionary = _start_pickup_leg()
    var manual_state := _vehicle_at_station("library")
    manual_state.manual_intervention = true
    var manual_result = manual_trip.journey.certify_station_arrival(
        "pickup",
        "library",
        _arrived_result(),
        manual_state
    )
    _expect(
        not manual_result.valid
            && manual_trip.journey.state == PassengerJourney.BLOCKED,
        "manual intervention prevents service-state advancement"
    )

    var route_failure_trip: Dictionary = _new_trip()
    var failed_route: Dictionary = route_failure_trip.booking_result.booking.routes.pickup
    _expect(
        route_failure_trip.booking_result.valid
            && route_failure_trip.assigned.valid
            && route_failure_trip.journey.state == PassengerJourney.ASSIGNED,
        "route-installation failure test has a valid assigned journey"
    )
    if not route_failure_trip.booking_result.valid \
        || not route_failure_trip.assigned.valid \
        || route_failure_trip.journey.state != PassengerJourney.ASSIGNED:
        return

    var failed_route_vehicle: Dictionary = _vehicle_at_route_start(failed_route)
    var original_failed_route_vehicle: Dictionary = failed_route_vehicle.duplicate(true)
    var failed_route_request: Dictionary = route_failure_trip.journey.request_next_route_installation(
        failed_route_vehicle
    )
    _expect(
        failed_route_request.valid
            && failed_route_request.get("leg", "") == "pickup"
            && route_failure_trip.journey.state == PassengerJourney.ASSIGNED,
        "route-installation failure test obtains a valid pickup install request"
    )
    if not failed_route_request.valid:
        return

    var failed_confirmation: Dictionary = route_failure_trip.journey.confirm_route_installation(
        failed_route_request.request_id,
        {"installed": false}
    )
    _expect(
        not failed_confirmation.valid
            && failed_confirmation.failure_reason == "route_installation_failed"
            && route_failure_trip.journey.state == PassengerJourney.BLOCKED
            && route_failure_trip.journey.blocked_reason == "route_installation_failed",
        "external route installation rejection safely blocks the journey"
    )
    _expect(
        route_failure_trip.journey.state != PassengerJourney.IDLE
            && not failed_confirmation.get("arrival_certified", false)
            && not failed_confirmation.get("arrived", false),
        "rejected route installation cannot report arrival or return to IDLE"
    )
    _expect(
        failed_route_vehicle == original_failed_route_vehicle,
        "route installation rejection leaves the caller's vehicle pose unchanged"
    )
    _expect(
        route_failure_trip.dispatcher.has_active_booking(),
        "rejected route installation keeps the shuttle reservation occupied"
    )

    var return_failure_dispatcher = ShuttleDispatcher.new(network)
    var booking := return_failure_dispatcher.request_booking("library", "sports_hall")
    var return_failure_journey = PassengerJourney.new(network, return_failure_dispatcher)
    return_failure_journey.assign_booking(booking)
    _expect(
        not return_failure_journey.confirm_passenger_exit().valid
            && return_failure_journey.state == PassengerJourney.ASSIGNED,
        "passenger exit in the wrong state is rejected without completing the booking"
    )
    _expect(
        not return_failure_journey.certify_station_arrival(
            "return",
            "north_gate",
            _arrived_result(),
            _vehicle_at_station("north_gate")
        ).valid,
        "return arrival before a return leg is rejected"
    )

    var manual_signal_trip: Dictionary = _new_trip()
    var manual_signal: Dictionary = manual_signal_trip.journey.report_manual_intervention()
    _expect(
        not manual_signal.valid
            && manual_signal_trip.journey.state == PassengerJourney.BLOCKED
            && manual_signal_trip.journey.blocked_reason == "manual_intervention",
        "manual intervention explicitly pauses the active service journey"
    )

func _test_pose_handoff_and_request_validation() -> void:
    var incompatible_trip: Dictionary = _new_trip()
    var route: Dictionary = incompatible_trip.booking_result.booking.routes.pickup
    var bad_pose := _vehicle_at_route_start(route)
    bad_pose.position_xz += Vector2(5.0, 0.0)
    var incompatible = incompatible_trip.journey.request_next_route_installation(bad_pose)
    _expect(
        not incompatible.valid
            && incompatible_trip.journey.state == PassengerJourney.BLOCKED
            && incompatible_trip.journey.blocked_reason == "route_start_pose_incompatible",
        "route with incompatible actual position is rejected without pose snapping"
    )

    var bad_heading_trip: Dictionary = _new_trip()
    var bad_heading_route: Dictionary = bad_heading_trip.booking_result.booking.routes.pickup
    var bad_heading_pose: Dictionary = _vehicle_at_route_start(bad_heading_route)
    bad_heading_pose.heading_rad = wrapf(bad_heading_pose.heading_rad + PI / 2.0, -PI, PI)
    var incompatible_heading: Dictionary = \
        bad_heading_trip.journey.request_next_route_installation(bad_heading_pose)
    _expect(
        not incompatible_heading.valid
            && bad_heading_trip.journey.blocked_reason == "route_start_heading_incompatible",
        "route with incompatible actual heading is rejected without rotating the vehicle"
    )

    var handoff_trip: Dictionary = _new_trip()
    var handoff_route: Dictionary = handoff_trip.booking_result.booking.routes.pickup
    var before: Dictionary = _vehicle_at_route_start(handoff_route)
    var request: Dictionary = handoff_trip.journey.request_next_route_installation(before)
    var changed_pose: Dictionary = before.duplicate(true)
    changed_pose.position_xz += Vector2(0.01, 0.0)
    var handoff = handoff_trip.journey.confirm_route_installation(
        request.request_id,
        {"installed": true, "vehicle_state": changed_pose}
    )
    _expect(
        not handoff.valid
            && handoff_trip.journey.state == PassengerJourney.BLOCKED
            && handoff_trip.journey.blocked_reason == "route_installation_changed_vehicle_state",
        "route installation that changes actual vehicle pose is rejected"
    )

    var wrong_request_trip: Dictionary = _new_trip()
    var wrong_request_route: Dictionary = wrong_request_trip.booking_result.booking.routes.pickup
    var request_pose: Dictionary = _vehicle_at_route_start(wrong_request_route)
    wrong_request_trip.journey.request_next_route_installation(request_pose)
    _expect(
        not wrong_request_trip.journey.confirm_route_installation(
            "wrong-request",
            {"installed": true, "vehicle_state": request_pose}
        ).valid && wrong_request_trip.journey.state == PassengerJourney.BLOCKED,
        "an uncorrelated route-install result cannot advance the service state"
    )

    var no_mutation_trip: Dictionary = _new_trip()
    var no_mutation_route: Dictionary = no_mutation_trip.booking_result.booking.routes.pickup
    var input_state: Dictionary = _vehicle_at_route_start(no_mutation_route)
    var original_position: Vector2 = input_state.position_xz
    var original_heading: float = input_state.heading_rad
    var input_copy: Dictionary = input_state.duplicate(true)
    no_mutation_trip.journey.request_next_route_installation(input_state)
    _expect(
        input_state == input_copy
            && input_state.position_xz == original_position
            && is_equal_approx(input_state.heading_rad, original_heading),
        "service request leaves caller-owned vehicle state unchanged"
    )

func _test_deterministic_repeated_journeys() -> void:
    var first: Dictionary = _new_trip()
    var second: Dictionary = _new_trip()
    var first_route: Dictionary = first.booking_result.booking.routes.pickup
    var second_route: Dictionary = second.booking_result.booking.routes.pickup
    var first_request: Dictionary = first.journey.request_next_route_installation(
        _vehicle_at_route_start(first_route)
    )
    var second_request: Dictionary = second.journey.request_next_route_installation(
        _vehicle_at_route_start(second_route)
    )
    _expect(
        first_request.valid && second_request.valid
            && first_request.request_id == second_request.request_id
            && first_request.centerline_points == second_request.centerline_points,
        "repeated identical journeys produce deterministic route requests"
    )

func _start_pickup_leg() -> Dictionary:
    var trip: Dictionary = _new_trip()
    var pickup_route: Dictionary = trip.booking_result.booking.routes.pickup
    _install_leg(trip.journey, pickup_route)
    return trip
