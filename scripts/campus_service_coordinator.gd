extends RefCounted

## Headless service executor. Call step once per physics tick; optional tick IDs reject duplicates.
const CampusNetwork = preload("res://scripts/campus_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const PassengerJourney = preload("res://scripts/passenger_journey.gd")
const RouteFollowingController = preload("res://scripts/route_following_controller.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const ShuttleDispatcher = preload("res://scripts/shuttle_dispatcher.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")
const RouteObstacle = preload("res://scripts/route_obstacle.gd")
const OffsetPathBuilder = preload("res://scripts/offset_path_builder.gd")

const POSITION_TOLERANCE_M: float = 1.0
const ROUTE_SAMPLES_PER_SEGMENT: int = 20
const ROUTE_SWAP_POSE_TOLERANCE_M: float = 0.0001
const ROUTE_SWAP_HEADING_TOLERANCE_RAD: float = deg_to_rad(12.0)

var network: RefCounted
var planner: RefCounted
var dispatcher: RefCounted
var journey: RefCounted
var navigator: RefCounted
var controller: RefCounted
var dynamics: RefCounted
var service_station_id: String = ""
var initialized: bool = false
var initialization_error: String = ""
var blocked_reason: String = ""
var last_failure_reason: String = ""
var current_leg: String = ""
var current_destination_station_id: String = ""
var arrival_status: String = ""
var last_control_result: Dictionary = {}
var last_step_result: Dictionary = {}
var route_leg_sequence: PackedStringArray = PackedStringArray()
var elapsed_simulation_s: float = 0.0
var maximum_cross_track_error_m: float = 0.0
var maximum_endpoint_error_m: float = 0.0
var maximum_route_handoff_position_delta_m: float = 0.0
var maximum_route_handoff_heading_delta_rad: float = 0.0
var dynamics_step_count: int = 0
var coordinator_tick_count: int = 0
var defer_destination_routing: bool = false
var defer_return_routing: bool = false
var active_obstacle: RefCounted = null
var lane_change_active: bool = false
var lane_change_wait_elapsed_s: float = 0.0
var no_way_hold_active: bool = false
var offset_path_builder: RefCounted = null
var road_width_m: float = 4.0

var _last_physics_tick_id: int = -1

func _init() -> void:
    network = CampusNetwork.new()
    planner = CampusRoutePlanner.new(network)
    dynamics = VehicleDynamics.new()
    navigator = RouteNavigator.new()
    controller = RouteFollowingController.new()
    offset_path_builder = OffsetPathBuilder.new()
    dispatcher = null
    journey = null

## Initializes the one-time physical start pose and the service station.
## This is the only method that resets VehicleDynamics; it cannot be called twice.
func initialize_service(
    initial_service_station_id: String,
    initial_position_xz: Vector2,
    initial_heading_rad: float
) -> Dictionary:
    if initialized:
        return _failure("service_already_initialized")
    if not initial_position_xz.is_finite() || not is_finite(initial_heading_rad):
        initialization_error = "invalid_initial_vehicle_pose"
        return _failure(initialization_error)
    var stop: Dictionary = network.get_station_stop(initial_service_station_id)
    if not stop.get("valid", false):
        initialization_error = "unknown_service_station"
        return _failure(initialization_error)
    var position_error_m: float = initial_position_xz.distance_to(stop.position_xz)
    if position_error_m > POSITION_TOLERANCE_M:
        initialization_error = "initial_pose_not_at_service_station"
        return _failure(initialization_error)

    service_station_id = initial_service_station_id
    dynamics.reset(
        initial_position_xz.y,
        0.0,
        initial_position_xz.x,
        initial_heading_rad
    )
    dispatcher = ShuttleDispatcher.new(network, planner, service_station_id)
    journey = PassengerJourney.new(network, dispatcher)
    initialized = true
    initialization_error = ""
    return {"valid": true, "failure_reason": "", "state": PassengerJourney.IDLE}

## Requests one booking and immediately installs/validates its pickup leg if needed.
func request_booking(
    pickup_station_id: String,
    destination_station_id: String
) -> Dictionary:
    if not initialized:
        return _failure("service_not_initialized")
    if journey.state != PassengerJourney.IDLE:
        return _failure("shuttle_occupied")
    var booking_result: Dictionary = dispatcher.request_booking(
        pickup_station_id,
        destination_station_id
    )
    if not booking_result.get("valid", false):
        last_failure_reason = str(booking_result.get("failure_reason", "booking_rejected"))
        return booking_result
    var assignment: Dictionary = journey.assign_booking(booking_result)
    if not assignment.get("valid", false):
        last_failure_reason = str(assignment.get("failure_reason", "booking_assignment_failed"))
        return assignment
    last_failure_reason = ""
    var route_result: Dictionary = _install_active_leg()
    if not route_result.get("valid", false):
        return route_result
    return {
        "valid": true,
        "failure_reason": "",
        "booking": booking_result.booking.duplicate(true),
        "state": journey.state,
        "route_installation": route_result
    }

## Advances service logic and at most one dynamics step for this physics tick.
## Pass physics_tick_id to make duplicate callbacks in the same engine tick idempotent.
func step(delta: float, physics_tick_id: int = -1) -> Dictionary:
    if not initialized:
        return _failure("service_not_initialized")
    if not is_finite(delta) || delta <= 0.0:
        return _failure("invalid_physics_timestep")
    var effective_tick_id: int = physics_tick_id
    if effective_tick_id < 0:
        effective_tick_id = Engine.get_physics_frames()
    if effective_tick_id == _last_physics_tick_id:
        return {
            "valid": false,
            "failure_reason": "duplicate_physics_tick",
            "duplicate_tick": true,
            "state": journey.state
        }
    coordinator_tick_count += 1
    _last_physics_tick_id = effective_tick_id
    elapsed_simulation_s += delta

    if journey.state == PassengerJourney.BLOCKED:
        if blocked_reason.is_empty():
            _set_blocked(journey.blocked_reason)
        _apply_blocked_safe_brake(delta)
        return _record_step_result({
            "valid": false,
            "failure_reason": blocked_reason,
            "state": journey.state,
            "status": "BLOCKED",
            "arrival_certified": false
        })
    if not _is_moving_state(journey.state):
        return _record_step_result({
            "valid": true,
            "failure_reason": "",
            "state": journey.state,
            "status": "WAITING_FOR_CONFIRMATION" if journey.state != PassengerJourney.IDLE
                else "IDLE",
            "arrival_certified": false
        })

    var control_obstacle: RefCounted = active_obstacle if not lane_change_active else null
    var control: Dictionary = controller.update(
        dynamics,
        navigator,
        delta,
        false,
        control_obstacle
    )
    if not lane_change_active:
        if bool(control.get("obstacle_hold", false)) \
            && dynamics.speed_mps <= OffsetPathBuilder.ARRIVAL_SPEED_TOLERANCE_MPS:
            lane_change_wait_elapsed_s += delta
            if lane_change_wait_elapsed_s >= OffsetPathBuilder.HOLD_BEFORE_BYPASS_S:
                var bypass: Dictionary = offset_path_builder.build(
                    navigator,
                    network,
                    Vector2(dynamics.position_x_m, dynamics.position_z_m),
                    dynamics.heading_rad,
                    dynamics.speed_mps,
                    active_obstacle,
                    road_width_m,
                    dynamics.wheelbase_m,
                    dynamics.steering_limit_rad
                )
                if bypass.get("valid", false):
                    var swap := replace_active_route(
                        bypass.centerline_points,
                        ROUTE_SAMPLES_PER_SEGMENT,
                        "lane-bypass",
                        PackedStringArray(["lane-bypass"])
                    )
                    if swap.get("valid", false):
                        lane_change_active = true
                        lane_change_wait_elapsed_s = 0.0
                        no_way_hold_active = false
                        control = controller.update(dynamics, navigator, delta, false, null)
                else:
                    no_way_hold_active = true
        elif active_obstacle == null || not active_obstacle.active:
            lane_change_wait_elapsed_s = 0.0
            no_way_hold_active = false
    last_control_result = control.duplicate(true)
    _update_tracking_metrics(control)
    var manual_intervention: bool = bool(control.get("manual_intervention", false))
    if manual_intervention:
        _block_active_journey("manual_intervention")
        _apply_dynamics_command(
            float(control.get("requested_acceleration_mps2", -dynamics.max_braking)),
            0.0,
            delta
        )
        return _record_step_result({
            "valid": false,
            "failure_reason": blocked_reason,
            "state": journey.state,
            "status": "BLOCKED",
            "arrival_certified": false
        })

    if control.get("status", "") == "ARRIVED" \
        && control.get("arrival_certified", false):
        return _certify_current_arrival(control)
    if control.get("status", "") == "SAFE STOP / INVALID NAVIGATION" \
        || control.get("status", "") == "MISSED DESTINATION" \
        || not control.get("localized", false):
        var reason: String = str(control.get("reason", control.get("status", "invalid_navigation")))
        _block_active_journey(reason)
        _apply_dynamics_command(
            float(control.get("requested_acceleration_mps2", -dynamics.max_braking)),
            0.0,
            delta
        )
        return _record_step_result({
            "valid": false,
            "failure_reason": blocked_reason,
            "state": journey.state,
            "status": "BLOCKED",
            "arrival_certified": false,
            "controller_result": control.duplicate(true)
        })

    _apply_dynamics_command(
        float(control.get("requested_acceleration_mps2", 0.0)),
        float(control.get("requested_steering_rad", 0.0)),
        delta
    )
    return _record_step_result({
        "valid": true,
        "failure_reason": "",
        "state": journey.state,
        "status": str(control.get("status", "EN ROUTE")),
        "arrival_certified": false,
        "controller_result": control.duplicate(true)
    })

## Signals manual intervention and enters BLOCKED; the next step applies safe braking.
func interrupt_service(reason: String = "manual_intervention") -> Dictionary:
    if not initialized:
        return _failure("service_not_initialized")
    if journey.state == PassengerJourney.IDLE:
        return _failure("no_active_journey")
    var blocked: Dictionary = journey.block_journey(reason)
    if not blocked.get("valid", false):
        return blocked
    _set_blocked(str(blocked.get("failure_reason", reason)))
    return {"valid": false, "failure_reason": blocked_reason, "state": journey.state}

## Accepts the explicit simulated passenger boarding action.
func confirm_boarding() -> Dictionary:
    if not initialized:
        return _failure("service_not_initialized")
    var result: Dictionary = journey.confirm_passenger_boarding()
    if not result.get("valid", false):
        last_failure_reason = str(result.get("failure_reason", "boarding_confirmation_failed"))
        return result
    last_failure_reason = ""
    if defer_destination_routing:
        navigator.clear_route()
        current_leg = "boarding_confirmed"
        return {"valid": true, "failure_reason": "", "state": journey.state, "boarding_confirmed": true}
    var installation: Dictionary = _install_active_leg()
    if not installation.get("valid", false):
        return installation
    return {"valid": true, "failure_reason": "", "state": journey.state}

## Accepts the explicit simulated passenger-exit action.
func confirm_passenger_exit() -> Dictionary:
    if not initialized:
        return _failure("service_not_initialized")
    var result: Dictionary = journey.confirm_passenger_exit()
    if not result.get("valid", false):
        last_failure_reason = str(result.get("failure_reason", "passenger_exit_failed"))
        return result
    last_failure_reason = ""
    if defer_return_routing:
        navigator.clear_route()
        current_leg = "dropoff_confirmed"
        return {"valid": true, "failure_reason": "", "state": journey.state, "exit_confirmed": true}
    var installation: Dictionary = _install_active_leg()
    if not installation.get("valid", false):
        return installation
    return {"valid": true, "failure_reason": "", "state": journey.state}

func get_service_snapshot() -> Dictionary:
    var pose := Vector2(dynamics.position_x_m, dynamics.position_z_m)
    var active_booking: Dictionary = {}
    if dispatcher != null:
        active_booking = dispatcher.get_active_booking()
    return {
        "initialized": initialized,
        "service_state": journey.state if journey != null else "UNINITIALIZED",
        "active_booking": active_booking,
        "current_leg": current_leg,
        "destination_station_id": current_destination_station_id,
        "position_xz": pose,
        "heading_rad": dynamics.heading_rad,
        "speed_mps": dynamics.speed_mps,
        "arrival_status": arrival_status,
        "arrival_certified": arrival_status == "ARRIVED",
        "boarding_confirmed": journey != null && journey.has_method("is_boarding_confirmed") && journey.is_boarding_confirmed(),
        "exit_confirmed": journey != null && journey.has_method("is_passenger_exit_confirmed") && journey.is_passenger_exit_confirmed(),
        "blocked": journey != null && journey.state == PassengerJourney.BLOCKED,
        "blocked_reason": blocked_reason if not blocked_reason.is_empty()
            else (journey.blocked_reason if journey != null else ""),
        "last_failure_reason": last_failure_reason,
        "elapsed_simulation_s": elapsed_simulation_s,
        "maximum_cross_track_error_m": maximum_cross_track_error_m,
        "maximum_endpoint_error_m": maximum_endpoint_error_m,
        "maximum_route_handoff_position_delta_m": maximum_route_handoff_position_delta_m,
        "maximum_route_handoff_heading_delta_rad": maximum_route_handoff_heading_delta_rad,
        "route_leg_sequence": route_leg_sequence.duplicate(),
        "dynamics_step_count": dynamics_step_count,
        "coordinator_tick_count": coordinator_tick_count,
        "last_control_result": last_control_result.duplicate(true),
        "last_step_result": last_step_result.duplicate(true),
        "obstacle_hold": bool(last_control_result.get("obstacle_hold", false)),
        "obstacle_distance_m": last_control_result.get("obstacle_distance_m", INF),
        "obstacle_stop_distance_m": last_control_result.get("obstacle_stop_distance_m", INF),
        "lane_change_active": lane_change_active,
        "lane_change_wait_elapsed_s": lane_change_wait_elapsed_s,
        "lane_change_status": "CHANGING LANE" if lane_change_active else "",
        "no_way_hold_active": no_way_hold_active
    }

func place_obstacle(
    distance_along_route_m: float,
    length_m: float = 4.0,
    route_id: String = "",
    segment_id: String = ""
) -> Dictionary:
    if not is_finite(distance_along_route_m) || not is_finite(length_m) || length_m < 0.0:
        return _failure("invalid_obstacle_data")
    active_obstacle = RouteObstacle.new(
        distance_along_route_m,
        length_m,
        route_id,
        segment_id,
        navigator.get_point_at_distance(distance_along_route_m),
        1.8
    )
    return {"valid": true, "failure_reason": ""}

func clear_obstacle() -> Dictionary:
    if active_obstacle != null:
        active_obstacle.active = false
    active_obstacle = null
    if not lane_change_active:
        lane_change_wait_elapsed_s = 0.0
        no_way_hold_active = false
    return {"valid": true, "failure_reason": ""}

## Replaces the active route without changing the coordinator's physical state.
## The replacement route must begin at the current pose and match its heading.
func replace_active_route(
    centerline_points: Array[Vector2],
    samples_per_segment: int = ROUTE_SAMPLES_PER_SEGMENT,
    route_id: String = "",
    segment_ids: PackedStringArray = PackedStringArray()
) -> Dictionary:
    if not initialized:
        return _failure("service_not_initialized")
    if not _is_moving_state(journey.state):
        return _failure("route_swap_not_allowed_in_current_state")
    if centerline_points.size() < 2:
        return _failure("route_swap_centerline_invalid")
    if samples_per_segment < 1:
        return _failure("route_swap_samples_invalid")

    var current_position := Vector2(dynamics.position_x_m, dynamics.position_z_m)
    var current_heading: float = dynamics.heading_rad
    var current_speed: float = dynamics.speed_mps
    if not current_position.is_finite() || not is_finite(current_heading) \
        || not is_finite(current_speed) || current_speed < 0.0:
        return _failure("route_swap_invalid_vehicle_state")
    if centerline_points[0].distance_to(current_position) > ROUTE_SWAP_POSE_TOLERANCE_M:
        return _failure("route_swap_start_pose_incompatible")
    var first_edge: Vector2 = centerline_points[1] - centerline_points[0]
    if first_edge.length_squared() <= 0.000001:
        return _failure("route_swap_start_tangent_invalid")
    var expected_heading: float = atan2(first_edge.x, first_edge.y)
    var heading_error: float = absf(wrapf(
        current_heading - expected_heading,
        -PI,
        PI
    ))
    if heading_error > ROUTE_SWAP_HEADING_TOLERANCE_RAD:
        return _failure("route_swap_start_heading_incompatible")

    var candidate_navigator := RouteNavigator.new()
    if not candidate_navigator.set_route(centerline_points, samples_per_segment):
        return _failure("route_swap_route_invalid")
    var projected_obstacle_distance: float = -1.0
    if active_obstacle != null && active_obstacle.active:
        var obstacle_projection: Dictionary = candidate_navigator.find_nearest_point(
            active_obstacle.world_position_xz,
            true
        )
        if not obstacle_projection.get("valid", false):
            return _failure("route_swap_obstacle_projection_failed")
        projected_obstacle_distance = float(obstacle_projection.progress_m)

    var pose_before := current_position
    var heading_before: float = current_heading
    var speed_before: float = current_speed
    if not navigator.set_route(centerline_points, samples_per_segment):
        return _failure("route_swap_installation_failed")
    controller.reset()
    var pose_after := Vector2(dynamics.position_x_m, dynamics.position_z_m)
    var heading_after: float = dynamics.heading_rad
    var speed_after: float = dynamics.speed_mps
    if pose_before.distance_to(pose_after) > ROUTE_SWAP_POSE_TOLERANCE_M \
        || absf(wrapf(heading_before - heading_after, -PI, PI)) > 0.000001 \
        || absf(speed_before - speed_after) > 0.0001:
        return _failure("route_swap_changed_vehicle_state")
    if active_obstacle != null && active_obstacle.active:
        active_obstacle.distance_along_route_m = projected_obstacle_distance
        active_obstacle.route_id = route_id
        active_obstacle.segment_id = str(segment_ids)
    return {
        "valid": true,
        "failure_reason": "",
        "state": journey.state,
        "route_id": route_id,
        "segment_ids": segment_ids.duplicate(),
        "obstacle_reprojected": projected_obstacle_distance >= 0.0,
        "obstacle_distance_along_route_m": projected_obstacle_distance
    }

func _install_active_leg() -> Dictionary:
    var booking: Dictionary = journey.get_active_booking()
    var leg: String
    var origin_station_id: String
    var destination_station_id: String
    match journey.state:
        PassengerJourney.ASSIGNED:
            leg = "pickup"
            origin_station_id = str(booking.get("service_station_id", ""))
            destination_station_id = str(booking.get("origin_station_id", ""))
        PassengerJourney.BOARDING:
            leg = "destination"
            origin_station_id = str(booking.get("origin_station_id", ""))
            destination_station_id = str(booking.get("destination_station_id", ""))
        PassengerJourney.DROPOFF:
            leg = "return"
            origin_station_id = str(booking.get("destination_station_id", ""))
            destination_station_id = str(booking.get("service_station_id", ""))
        _:
            return _failure("no_installable_route_for_current_state")

    var vehicle_state: Dictionary = _vehicle_snapshot(true)
    var planned_route: Dictionary
    if leg == "pickup" && _is_at_station(vehicle_state.position_xz, destination_station_id):
        planned_route = _stationary_route(origin_station_id, destination_station_id)
    elif origin_station_id == destination_station_id:
        planned_route = _stationary_route(origin_station_id, destination_station_id)
    else:
        planned_route = planner.plan_route_from_pose(
            origin_station_id,
            destination_station_id,
            vehicle_state.position_xz,
            vehicle_state.heading_rad
        )
        if not planned_route.get("valid", false):
            var planning_reason: String = str(
                planned_route.get("failure_reason", "route_planning_failed")
            )
            var rejected: Dictionary = journey.request_next_route_installation(
                vehicle_state,
                {"valid": false}
            )
            _set_blocked(planning_reason)
            return {
                "valid": false,
                "failure_reason": planning_reason,
                "journey_failure": rejected
            }

    var installation_request: Dictionary = journey.request_next_route_installation(
        vehicle_state,
        planned_route
    )
    if not installation_request.get("valid", false):
        _set_blocked(str(installation_request.get("failure_reason", "route_handoff_rejected")))
        return installation_request
    if installation_request.get("stationary_route", false):
        current_leg = ""
        current_destination_station_id = ""
        arrival_status = "ARRIVED"
        lane_change_active = false
        lane_change_wait_elapsed_s = 0.0
        no_way_hold_active = false
        last_failure_reason = ""
        return installation_request

    var centerline_value: Variant = installation_request.get("centerline_points", null)
    if not centerline_value is Array:
        var missing_centerline: Dictionary = journey.confirm_route_installation(
            str(installation_request.get("request_id", "")),
            {"installed": false}
        )
        _set_blocked(str(missing_centerline.get("failure_reason", "route_centerline_missing")))
        return missing_centerline
    var centerline: Array = centerline_value
    var typed_centerline: Array[Vector2] = []
    for point in centerline:
        if not point is Vector2:
            var invalid_centerline: Dictionary = journey.confirm_route_installation(
                str(installation_request.get("request_id", "")),
                {"installed": false}
            )
            _set_blocked(str(invalid_centerline.get("failure_reason", "route_centerline_invalid")))
            return invalid_centerline
        typed_centerline.append(point)
    var pose_before_install := Vector2(dynamics.position_x_m, dynamics.position_z_m)
    var heading_before_install: float = dynamics.heading_rad
    var route_installed: bool = navigator.set_route(
        typed_centerline,
        ROUTE_SAMPLES_PER_SEGMENT
    )
    if route_installed:
        controller.reset()
        var pose_after_install := Vector2(dynamics.position_x_m, dynamics.position_z_m)
        maximum_route_handoff_position_delta_m = maxf(
            maximum_route_handoff_position_delta_m,
            pose_before_install.distance_to(pose_after_install)
        )
        maximum_route_handoff_heading_delta_rad = maxf(
            maximum_route_handoff_heading_delta_rad,
            absf(wrapf(dynamics.heading_rad - heading_before_install, -PI, PI))
        )
        var actual_after_install: Dictionary = _vehicle_snapshot(true)
        var pose_verification: Dictionary = journey.confirm_route_installation(
            str(installation_request.get("request_id", "")),
            {"installed": true, "vehicle_state": actual_after_install}
        )
        if not pose_verification.get("valid", false):
            _set_blocked(str(pose_verification.get("failure_reason", "route_handoff_failed")))
            return pose_verification
    else:
        var installation_failure: Dictionary = journey.confirm_route_installation(
            str(installation_request.get("request_id", "")),
            {"installed": false}
        )
        _set_blocked(str(installation_failure.get("failure_reason", "route_installation_failed")))
        return installation_failure

    current_leg = leg
    current_destination_station_id = destination_station_id
    arrival_status = "EN_ROUTE"
    last_failure_reason = ""
    if route_leg_sequence.is_empty() || route_leg_sequence[route_leg_sequence.size() - 1] != leg:
        route_leg_sequence.append(leg)
    return {
        "valid": true,
        "failure_reason": "",
        "leg": leg,
        "destination_station_id": destination_station_id,
        "segment_ids": planned_route.get("segment_ids", PackedStringArray()),
        "position_error_m": installation_request.get("position_error_m", 0.0),
        "heading_error_rad": installation_request.get("heading_error_rad", 0.0)
    }

func _certify_current_arrival(control: Dictionary) -> Dictionary:
    var destination_id: String = current_destination_station_id
    var leg: String = current_leg
    var actual_vehicle_state: Dictionary = _vehicle_snapshot(
        bool(control.get("localized", false))
    )
    var endpoint_error_m: float = float(control.get("endpoint_error_m", INF))
    maximum_endpoint_error_m = maxf(maximum_endpoint_error_m, endpoint_error_m)
    var arrival: Dictionary = journey.certify_station_arrival(
        leg,
        destination_id,
        control,
        actual_vehicle_state
    )
    if not arrival.get("valid", false):
        _set_blocked(str(arrival.get("failure_reason", "arrival_certification_failed")))
        return _record_step_result({
            "valid": false,
            "failure_reason": blocked_reason,
            "status": "BLOCKED",
            "state": journey.state,
            "arrival_certified": false,
            "controller_result": control.duplicate(true)
        })
    current_leg = ""
    current_destination_station_id = ""
    arrival_status = "ARRIVED"
    last_failure_reason = ""
    return _record_step_result({
        "valid": true,
        "failure_reason": "",
        "status": "ARRIVED",
        "state": journey.state,
        "arrival_certified": true,
        "station_id": destination_id,
        "leg": leg,
        "controller_result": control.duplicate(true)
    })

func _vehicle_snapshot(localized: bool) -> Dictionary:
    return {
        "position_xz": Vector2(dynamics.position_x_m, dynamics.position_z_m),
        "heading_rad": dynamics.heading_rad,
        "speed_mps": dynamics.speed_mps,
        "localized": localized,
        "manual_intervention": false
    }

func _stationary_route(origin_station_id: String, destination_station_id: String) -> Dictionary:
    return {
        "valid": true,
        "stationary": true,
        "origin_station_id": origin_station_id,
        "destination_station_id": destination_station_id,
        "segment_ids": PackedStringArray(),
        "centerline_points": []
    }

func _is_at_station(position_xz: Vector2, station_id: String) -> bool:
    var stop: Dictionary = network.get_station_stop(station_id)
    return stop.get("valid", false) \
        && position_xz.distance_to(stop.position_xz) <= POSITION_TOLERANCE_M

func _is_moving_state(service_state: String) -> bool:
    return service_state in [
        PassengerJourney.TO_PICKUP,
        PassengerJourney.TO_DESTINATION,
        PassengerJourney.RETURNING_TO_SERVICE
    ]

func _apply_dynamics_command(
    acceleration_mps2: float,
    steering_rad: float,
    delta: float
) -> void:
    dynamics.step(acceleration_mps2, delta, steering_rad)
    dynamics_step_count += 1

func _apply_blocked_safe_brake(delta: float) -> void:
    if dynamics.speed_mps <= 0.0:
        return
    _apply_dynamics_command(-dynamics.max_braking, 0.0, delta)

func _block_active_journey(reason: String) -> void:
    var blocked: Dictionary = journey.block_journey(reason)
    _set_blocked(
        str(blocked.get("failure_reason", reason))
        if blocked.get("valid", false)
        else reason
    )

func _set_blocked(reason: String) -> void:
    blocked_reason = reason if not reason.is_empty() else "service_blocked"
    last_failure_reason = blocked_reason
    arrival_status = "BLOCKED"
    current_leg = ""
    current_destination_station_id = ""

func _update_tracking_metrics(control: Dictionary) -> void:
    if control.has("cross_track_error_m") && is_finite(float(control.cross_track_error_m)):
        maximum_cross_track_error_m = maxf(
            maximum_cross_track_error_m,
            absf(float(control.cross_track_error_m))
        )

func _record_step_result(result: Dictionary) -> Dictionary:
    last_step_result = result.duplicate(true)
    return last_step_result.duplicate(true)

func _failure(reason: String) -> Dictionary:
    last_failure_reason = reason
    return {
        "valid": false,
        "failure_reason": reason,
        "state": journey.state if journey != null else "UNINITIALIZED"
    }
