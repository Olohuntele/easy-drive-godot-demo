extends SceneTree

const CampusNetwork = preload("res://scripts/campus_network.gd")
const CampusServiceCoordinator = preload("res://scripts/campus_service_coordinator.gd")
const PassengerJourney = preload("res://scripts/passenger_journey.gd")

const FIXED_STEPS: Array[float] = [1.0 / 60.0, 1.0 / 120.0]
const MAX_TICKS_PER_LEG: int = 12000
const MAX_CROSS_TRACK_ERROR_M: float = 0.75
const MAX_ENDPOINT_ERROR_M: float = 1.0
const MAX_FINAL_SPEED_MPS: float = 0.1
const POSE_TOLERANCE_M: float = 0.000001
const HEADING_TOLERANCE_RAD: float = 0.000001

var failures: int = 0
var assertions: int = 0
var network: RefCounted

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    network = CampusNetwork.new()
    _test_complete_journeys_at_both_timesteps()
    _test_sequential_bookings()
    _test_three_consecutive_journeys_and_stale_events()
    _test_booking_and_confirmation_validation()
    _test_stationary_and_remote_pickup()
    _test_wrong_station_arrival_and_safe_blocking()
    _test_manual_interruption_and_invalid_localization()
    _test_duplicate_physics_tick_is_ignored()
    _test_initialization_validation_and_determinism()
    if failures == 0:
        print("CampusServiceCoordinator tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d CampusServiceCoordinator assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _heading_for_start(segment_id: String) -> float:
    var segment: Dictionary = network.get_segment(segment_id)
    var points: Array = segment.centerline_points
    var tangent: Vector2 = (points[1] - points[0]).normalized()
    return atan2(tangent.x, tangent.y)

func _new_coordinator(
    service_station_id: String = "north_gate",
    heading_rad: float = -1.0
) -> RefCounted:
    var coordinator = CampusServiceCoordinator.new()
    var stop: Dictionary = network.get_station_stop(service_station_id)
    var initial_heading: float = heading_rad
    if initial_heading < -0.5:
        initial_heading = _heading_for_start("north_to_east")
    var initialized: Dictionary = coordinator.initialize_service(
        service_station_id,
        stop.position_xz,
        initial_heading
    )
    _expect(initialized.valid, "coordinator initializes at %s" % service_station_id)
    return coordinator

func _run_booking(
    coordinator: RefCounted,
    pickup_station_id: String,
    destination_station_id: String,
    delta: float,
    start_tick_id: int
) -> Dictionary:
    var booking: Dictionary = coordinator.request_booking(
        pickup_station_id,
        destination_station_id
    )
    _expect(
        booking.valid,
        "booking %s -> %s is accepted (%s)"
            % [pickup_station_id, destination_station_id, booking.get("failure_reason", "")]
    )
    if not booking.valid:
        return {"success": false, "next_tick_id": start_tick_id}

    var context := {"tick_id": start_tick_id}
    if coordinator.journey.state == PassengerJourney.TO_PICKUP:
        var pickup_reached: bool = _drive_until_state(
            coordinator,
            PassengerJourney.BOARDING,
            delta,
            context,
            "pickup"
        )
        _expect(pickup_reached, "remote pickup reaches BOARDING")
        if not pickup_reached:
            return {"success": false, "next_tick_id": context.tick_id}
    else:
        _expect(
            coordinator.journey.state == PassengerJourney.BOARDING,
            "stationary pickup reaches BOARDING without a drive leg"
        )

    var boarding: Dictionary = coordinator.confirm_boarding()
    _expect(
        boarding.valid && coordinator.journey.state == PassengerJourney.TO_DESTINATION,
        "boarding confirmation installs the passenger route"
    )
    if not boarding.valid:
        return {"success": false, "next_tick_id": context.tick_id}
    var destination_reached: bool = _drive_until_state(
        coordinator,
        PassengerJourney.DROPOFF,
        delta,
        context,
        "destination"
    )
    _expect(destination_reached, "passenger route reaches DROPOFF")
    if not destination_reached:
        return {"success": false, "next_tick_id": context.tick_id}

    var passenger_exit: Dictionary = coordinator.confirm_passenger_exit()
    _expect(
        passenger_exit.valid
            && coordinator.journey.state == PassengerJourney.RETURNING_TO_SERVICE,
        "passenger exit confirmation installs the return route"
    )
    if not passenger_exit.valid:
        return {"success": false, "next_tick_id": context.tick_id}
    var return_reached: bool = _drive_until_state(
        coordinator,
        PassengerJourney.IDLE,
        delta,
        context,
        "service return"
    )
    _expect(return_reached, "certified return completes the booking and reaches IDLE")
    return {
        "success": return_reached,
        "next_tick_id": context.tick_id,
        "snapshot": coordinator.get_service_snapshot()
    }

func _drive_until_state(
    coordinator: RefCounted,
    target_state: String,
    delta: float,
    context: Dictionary,
    leg_description: String
) -> bool:
    var start_steps: int = coordinator.dynamics_step_count
    var dynamics_step_limit_violated: bool = false
    for _tick in range(MAX_TICKS_PER_LEG):
        if coordinator.journey.state == target_state:
            return true
        if coordinator.journey.state == PassengerJourney.BLOCKED:
            break
        var previous_steps: int = coordinator.dynamics_step_count
        var result: Dictionary = coordinator.step(delta, int(context.tick_id))
        context.tick_id = int(context.tick_id) + 1
        dynamics_step_limit_violated = dynamics_step_limit_violated \
            || coordinator.dynamics_step_count - previous_steps > 1
        if not result.get("valid", false) \
            && result.get("status", "") != "ARRIVED":
            break
    var snapshot: Dictionary = coordinator.get_service_snapshot()
    _expect(
        not dynamics_step_limit_violated,
        "%s physics ticks perform no more than one dynamics step" % leg_description
    )
    print(
        "LEG RESULT %s: state=%s, ticks=%d, dynamics_steps=%d, speed=%.4f, reason=%s"
            % [
                leg_description,
                snapshot.service_state,
                int(context.tick_id),
                coordinator.dynamics_step_count - start_steps,
                snapshot.speed_mps,
                snapshot.last_failure_reason
            ]
    )
    return coordinator.journey.state == target_state

func _test_complete_journeys_at_both_timesteps() -> void:
    var journeys := [
        {"pickup": "north_gate", "destination": "library"},
        {"pickup": "sports_hall", "destination": "innovation_centre"},
        {"pickup": "library", "destination": "sports_hall"}
    ]
    for delta in FIXED_STEPS:
        for journey_case in journeys:
            var coordinator: RefCounted = _new_coordinator()
            var completed: Dictionary = _run_booking(
                coordinator,
                str(journey_case.pickup),
                str(journey_case.destination),
                delta,
                1
            )
            _expect(
                completed.success,
                "%s -> %s completes at dt=%.6f"
                    % [journey_case.pickup, journey_case.destination, delta]
            )
            if not completed.success:
                continue
            var snapshot: Dictionary = completed.snapshot
            _expect(
                snapshot.service_state == PassengerJourney.IDLE
                    && snapshot.active_booking.is_empty(),
                "successful %s journey releases its booking only at certified service return"
                    % journey_case.pickup
            )
            _expect(
                snapshot.maximum_cross_track_error_m <= MAX_CROSS_TRACK_ERROR_M,
                "%s journey max CTE %.4f m is within %.2f m"
                    % [
                        journey_case.pickup,
                        snapshot.maximum_cross_track_error_m,
                        MAX_CROSS_TRACK_ERROR_M
                    ]
            )
            _expect(
                snapshot.maximum_endpoint_error_m <= MAX_ENDPOINT_ERROR_M,
                "%s journey endpoint error %.4f m is within %.2f m"
                    % [
                        journey_case.pickup,
                        snapshot.maximum_endpoint_error_m,
                        MAX_ENDPOINT_ERROR_M
                    ]
            )
            _expect(
                snapshot.speed_mps <= MAX_FINAL_SPEED_MPS,
                "service return stops at or below %.2f m/s" % MAX_FINAL_SPEED_MPS
            )
            _expect(
                snapshot.elapsed_simulation_s > 0.0
                    && snapshot.route_leg_sequence == (
                        PackedStringArray(["destination", "return"])
                        if journey_case.pickup == "north_gate"
                        else PackedStringArray(["pickup", "destination", "return"])
                    ),
                "journey reports elapsed travel time and all three moving legs"
            )
            _expect(
                snapshot.maximum_route_handoff_position_delta_m <= POSE_TOLERANCE_M
                    && snapshot.maximum_route_handoff_heading_delta_rad
                        <= HEADING_TOLERANCE_RAD,
                "route handoffs preserve vehicle pose and heading"
            )
            print(
                "JOURNEY SUMMARY %s -> %s dt=%.6f s: travel=%.3f s max_cte=%.4f m endpoint=%.4f m final=%s legs=%s"
                    % [
                        journey_case.pickup,
                        journey_case.destination,
                        delta,
                        snapshot.elapsed_simulation_s,
                        snapshot.maximum_cross_track_error_m,
                        snapshot.maximum_endpoint_error_m,
                        snapshot.service_state,
                        ",".join(snapshot.route_leg_sequence)
                    ]
            )

func _test_sequential_bookings() -> void:
    var coordinator: RefCounted = _new_coordinator()
    var first: Dictionary = _run_booking(
        coordinator,
        "north_gate",
        "library",
        FIXED_STEPS[0],
        1
    )
    _expect(first.success, "first sequential passenger journey returns to service")
    if not first.success:
        return
    var position_before_second: Vector2 = Vector2(
        coordinator.dynamics.position_x_m,
        coordinator.dynamics.position_z_m
    )
    var heading_before_second: float = coordinator.dynamics.heading_rad
    var second: Dictionary = _run_booking(
        coordinator,
        "sports_hall",
        "innovation_centre",
        FIXED_STEPS[0],
        int(first.next_tick_id)
    )
    _expect(second.success, "second booking is served after certified first return")
    _expect(
        position_before_second.distance_to(
            network.get_station_stop("north_gate").position_xz
        ) <= MAX_ENDPOINT_ERROR_M
            && coordinator.get_service_snapshot().service_state == PassengerJourney.IDLE,
        "second booking starts at the physically reached service station"
    )
    _expect(
        is_finite(heading_before_second)
            && coordinator.get_service_snapshot().maximum_route_handoff_position_delta_m
                <= POSE_TOLERANCE_M,
        "sequential journey preserves physical handoff state"
    )

func _test_three_consecutive_journeys_and_stale_events() -> void:
    var coordinator: RefCounted = _new_coordinator()
    var dt: float = FIXED_STEPS[0]
    var tick: int = 1

    var res1 = _run_booking(coordinator, "north_gate", "library", dt, tick)
    _expect(res1.success, "three consecutive journeys: journey 1 completes successfully")
    if not res1.success:
        return
    tick = int(res1.next_tick_id)

    var stale_board = coordinator.confirm_boarding()
    var stale_exit = coordinator.confirm_passenger_exit()
    _expect(
        not stale_board.get("valid", false) && not stale_exit.get("valid", false),
        "stale confirmation events after completed journey are rejected"
    )

    var res2 = _run_booking(coordinator, "sports_hall", "innovation_centre", dt, tick)
    _expect(res2.success, "three consecutive journeys: journey 2 completes successfully")
    if not res2.success:
        return
    tick = int(res2.next_tick_id)

    var res3 = _run_booking(coordinator, "library", "sports_hall", dt, tick)
    _expect(
        res3.success && coordinator.get_service_snapshot().service_state == PassengerJourney.IDLE,
        "three consecutive journeys: journey 3 completes and returns to IDLE availability"
    )

func _test_booking_and_confirmation_validation() -> void:
    var coordinator: RefCounted = _new_coordinator()
    _expect(
        not coordinator.request_booking("missing", "library").valid
            && coordinator.journey.state == PassengerJourney.IDLE,
        "unknown pickup booking is rejected without changing IDLE"
    )
    _expect(
        not coordinator.request_booking("library", "missing").valid
            && coordinator.journey.state == PassengerJourney.IDLE,
        "unknown destination booking is rejected without changing IDLE"
    )
    _expect(
        not coordinator.request_booking("library", "library").valid
            && coordinator.journey.state == PassengerJourney.IDLE,
        "same-station booking is rejected without changing IDLE"
    )
    _expect(
        not coordinator.confirm_boarding().valid
            && not coordinator.confirm_passenger_exit().valid,
        "wrong-state passenger confirmations are rejected"
    )
    var accepted: Dictionary = coordinator.request_booking("library", "sports_hall")
    _expect(accepted.valid, "valid booking is accepted before occupancy checks")
    _expect(
        not coordinator.request_booking("sports_hall", "library").valid
            && coordinator.dispatcher.has_active_booking(),
        "overlapping booking is rejected while journey is occupied"
    )
    var before_pickup: Dictionary = coordinator.get_service_snapshot()
    _expect(
        not coordinator.confirm_boarding().valid
            && coordinator.journey.state == PassengerJourney.TO_PICKUP,
        "boarding cannot be confirmed before pickup arrival"
    )
    _expect(
        before_pickup.service_state == PassengerJourney.TO_PICKUP,
        "rejected boarding does not skip the pickup leg"
    )
    var tick_context := {"tick_id": 1}
    var pickup_reached: bool = _drive_until_state(
        coordinator,
        PassengerJourney.BOARDING,
        FIXED_STEPS[0],
        tick_context,
        "boarding gate"
    )
    var boarding: Dictionary = coordinator.confirm_boarding() if pickup_reached else {}
    var early_exit: Dictionary = coordinator.confirm_passenger_exit()
    _expect(
        pickup_reached
            && boarding.get("valid", false)
            && not early_exit.get("valid", false)
            && coordinator.journey.state == PassengerJourney.TO_DESTINATION,
        "passenger exit cannot be confirmed before certified drop-off"
    )

func _test_stationary_and_remote_pickup() -> void:
    var stationary: RefCounted = _new_coordinator()
    var stationary_booking: Dictionary = stationary.request_booking(
        "north_gate",
        "library"
    )
    _expect(
        stationary_booking.valid
            && stationary.journey.state == PassengerJourney.BOARDING
            && stationary.current_leg.is_empty()
            && stationary.dynamics_step_count == 0,
        "pickup at current station is certified stationary without a motion step"
    )
    _expect(
        stationary.confirm_boarding().valid
            && stationary.journey.state == PassengerJourney.TO_DESTINATION,
        "stationary pickup still requires explicit boarding"
    )

    for pickup_id in ["library", "sports_hall"]:
        var remote: RefCounted = _new_coordinator()
        var request: Dictionary = remote.request_booking(pickup_id, "innovation_centre")
        _expect(
            request.valid && remote.journey.state == PassengerJourney.TO_PICKUP
                && remote.current_leg == "pickup",
            "%s remote pickup installs a heading-compatible moving route" % pickup_id
        )

func _test_wrong_station_arrival_and_safe_blocking() -> void:
    var coordinator: RefCounted = _new_coordinator()
    var booking: Dictionary = coordinator.request_booking("library", "sports_hall")
    _expect(booking.valid, "wrong-station scenario obtains a valid booking")
    if not booking.valid:
        return
    var wrong_stop: Dictionary = network.get_station_stop("sports_hall")
    var fake_arrival := {
        "status": "ARRIVED",
        "arrival_certified": true,
        "localized": true,
        "reason": "route_complete"
    }
    var rejected: Dictionary = coordinator.journey.certify_station_arrival(
        "pickup",
        "sports_hall",
        fake_arrival,
        {
            "position_xz": wrong_stop.position_xz,
            "heading_rad": coordinator.dynamics.heading_rad,
            "speed_mps": 0.0,
            "localized": true,
            "manual_intervention": false
        }
    )
    _expect(
        not rejected.valid
            && coordinator.journey.state == PassengerJourney.BLOCKED
            && coordinator.dispatcher.has_active_booking(),
        "wrong-station arrival telemetry blocks without releasing the booking"
    )
    var blocked_step: Dictionary = coordinator.step(FIXED_STEPS[0], 1)
    _expect(
        not blocked_step.valid
            && coordinator.get_service_snapshot().service_state == PassengerJourney.BLOCKED
            && coordinator.get_service_snapshot().arrival_certified == false,
        "blocked wrong-station journey cannot falsely report arrival or IDLE"
    )

    var invalid_telemetry: RefCounted = _new_coordinator()
    var invalid_booking: Dictionary = invalid_telemetry.request_booking(
        "library",
        "sports_hall"
    )
    var uncertified_arrival: Dictionary = invalid_telemetry.journey.certify_station_arrival(
        "pickup",
        "library",
        {
            "status": "ARRIVED",
            "arrival_certified": false,
            "localized": true,
            "reason": "uncertified"
        },
        {
            "position_xz": Vector2(
                invalid_telemetry.dynamics.position_x_m,
                invalid_telemetry.dynamics.position_z_m
            ),
            "heading_rad": invalid_telemetry.dynamics.heading_rad,
            "speed_mps": invalid_telemetry.dynamics.speed_mps,
            "localized": true,
            "manual_intervention": false
        }
    )
    _expect(
        invalid_booking.valid
            && not uncertified_arrival.get("valid", false)
            && invalid_telemetry.journey.state == PassengerJourney.BLOCKED
            && invalid_telemetry.dispatcher.has_active_booking(),
        "uncertified arrival telemetry blocks without releasing the booking"
    )

func _test_manual_interruption_and_invalid_localization() -> void:
    var manual: RefCounted = _new_coordinator()
    _expect(
        manual.request_booking("library", "sports_hall").valid,
        "manual interruption scenario starts a remote pickup"
    )
    var first_tick: Dictionary = manual.step(FIXED_STEPS[0], 1)
    _expect(first_tick.valid, "manual scenario advances one autonomous physics tick")
    var steps_before_interrupt: int = manual.dynamics_step_count
    var interruption: Dictionary = manual.interrupt_service()
    _expect(
        not interruption.valid && manual.journey.state == PassengerJourney.BLOCKED
            && manual.dispatcher.has_active_booking(),
        "manual interruption blocks journey and retains reservation"
    )
    var brake_step: Dictionary = manual.step(FIXED_STEPS[0], 2)
    _expect(
        not brake_step.valid && manual.dynamics_step_count == steps_before_interrupt + 1
            && manual.dynamics.speed_mps <= 0.1,
        "blocked manual journey applies one safe braking step without becoming IDLE"
    )

    var localization: RefCounted = _new_coordinator()
    _expect(
        localization.request_booking("library", "sports_hall").valid,
        "localization scenario starts with valid route and booking"
    )
    localization.controller.max_localization_error_m = 0.000001
    var localization_failed: bool = false
    for tick in range(1, 200):
        var result: Dictionary = localization.step(FIXED_STEPS[0], tick)
        if localization.journey.state == PassengerJourney.BLOCKED:
            localization_failed = true
            _expect(
                not result.get("valid", false)
                    && localization.journey.blocked_reason in [
                        "route_localization_lost",
                        "invalid_controller_configuration"
                    ],
                "lost route localization enters explicit blocked state"
            )
            break
    _expect(localization_failed, "invalid localization cannot continue route execution")
    _expect(
        localization.journey.state != PassengerJourney.IDLE
            && localization.dispatcher.has_active_booking(),
        "localization failure neither completes journey nor releases shuttle"
    )

func _test_duplicate_physics_tick_is_ignored() -> void:
    var coordinator: RefCounted = _new_coordinator()
    _expect(
        coordinator.request_booking("library", "sports_hall").valid,
        "duplicate-tick scenario starts an active route"
    )
    var ticks_before_first: int = coordinator.coordinator_tick_count
    var steps_before_first: int = coordinator.dynamics_step_count
    var first: Dictionary = coordinator.step(FIXED_STEPS[0], 77)
    var steps_after_first: int = coordinator.dynamics_step_count
    var ticks_after_first: int = coordinator.coordinator_tick_count
    var pose_after_first := Vector2(
        coordinator.dynamics.position_x_m,
        coordinator.dynamics.position_z_m
    )
    var duplicate: Dictionary = coordinator.step(FIXED_STEPS[0], 77)
    _expect(first.valid && duplicate.get("duplicate_tick", false),
        "repeated callback with the same physics tick ID is rejected")
    _expect(
        coordinator.coordinator_tick_count == ticks_before_first + 1
            && coordinator.dynamics_step_count == steps_before_first + 1,
        "one accepted physics tick increments the counter and steps dynamics once"
    )
    _expect(
        coordinator.dynamics_step_count == steps_after_first
            && coordinator.coordinator_tick_count == ticks_after_first
            && Vector2(
                coordinator.dynamics.position_x_m,
                coordinator.dynamics.position_z_m
            ) == pose_after_first,
        "duplicate physics tick performs no controller/dynamics advancement or pose change"
    )
    var next_tick_steps: int = coordinator.dynamics_step_count
    var next_tick: Dictionary = coordinator.step(FIXED_STEPS[0], 78)
    _expect(
        next_tick.valid
            && coordinator.coordinator_tick_count == ticks_after_first + 1
            && coordinator.dynamics_step_count == next_tick_steps + 1,
        "a new valid physics tick increments the counter and steps dynamics once"
    )
    var ticks_before_invalid: int = coordinator.coordinator_tick_count
    var steps_before_invalid: int = coordinator.dynamics_step_count
    var invalid_tick: Dictionary = coordinator.step(0.0, 79)
    _expect(
        not invalid_tick.get("valid", false)
            && coordinator.coordinator_tick_count == ticks_before_invalid
            && coordinator.dynamics_step_count == steps_before_invalid,
        "invalid timestep is rejected without incrementing or stepping dynamics"
    )
    var after_invalid: Dictionary = coordinator.step(FIXED_STEPS[0], 79)
    _expect(
        after_invalid.valid
            && coordinator.coordinator_tick_count == ticks_before_invalid + 1
            && coordinator.dynamics_step_count == steps_before_invalid + 1,
        "rejected tick does not consume its ID or prevent the next accepted tick"
    )

    var automatic: RefCounted = _new_coordinator()
    _expect(
        automatic.request_booking("library", "sports_hall").valid,
        "automatic-tick scenario starts an active route"
    )
    var automatic_first: Dictionary = automatic.step(FIXED_STEPS[0])
    var automatic_steps: int = automatic.dynamics_step_count
    var automatic_duplicate: Dictionary = automatic.step(FIXED_STEPS[0])
    _expect(
        automatic_first.valid
            && automatic_duplicate.get("duplicate_tick", false)
            && automatic.dynamics_step_count == automatic_steps,
        "step(delta) uses the engine physics frame to prevent duplicate stepping"
    )

func _test_initialization_validation_and_determinism() -> void:
    var invalid = CampusServiceCoordinator.new()
    _expect(
        not invalid.initialize_service("unknown", Vector2.ZERO, 0.0).valid
            && not invalid.initialized,
        "unknown service station cannot initialize coordinator"
    )
    var wrong_pose = CampusServiceCoordinator.new()
    _expect(
        not wrong_pose.initialize_service(
            "north_gate",
            Vector2(5.0, 5.0),
            0.0
        ).valid
            && not wrong_pose.initialized,
        "initial pose away from service station is rejected"
    )
    var repeat_one: RefCounted = _new_coordinator()
    var repeat_two: RefCounted = _new_coordinator()
    var one: Dictionary = _run_booking(
        repeat_one,
        "sports_hall",
        "innovation_centre",
        FIXED_STEPS[0],
        1
    )
    var two: Dictionary = _run_booking(
        repeat_two,
        "sports_hall",
        "innovation_centre",
        FIXED_STEPS[0],
        1
    )
    _expect(
        one.success && two.success
            && one.snapshot.route_leg_sequence == two.snapshot.route_leg_sequence
            && is_equal_approx(
                one.snapshot.elapsed_simulation_s,
                two.snapshot.elapsed_simulation_s
            )
            && is_equal_approx(
                one.snapshot.maximum_cross_track_error_m,
                two.snapshot.maximum_cross_track_error_m
            ),
        "identical coordinator journeys produce deterministic route and tracking results"
    )
