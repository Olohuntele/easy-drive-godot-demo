extends SceneTree

const CampusNetwork = preload("res://scripts/campus_network.gd")
const CampusRoutePlanner = preload("res://scripts/campus_route_planner.gd")
const RouteFollowingController = preload("res://scripts/route_following_controller.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")

const FIXED_STEPS: Array[float] = [1.0 / 60.0, 1.0 / 120.0]
const CROSS_TRACK_TOLERANCE_M: float = 0.75
const ENDPOINT_TOLERANCE_M: float = 1.0
const ARRIVAL_SPEED_TOLERANCE_MPS: float = 0.1
const NUMERICAL_TOLERANCE: float = 0.0001
const MAX_SIMULATION_TICKS: int = 12000
const HEADING_COMPATIBILITY_TOLERANCE_RAD: float = deg_to_rad(12.0)

var failures: int = 0
var assertions: int = 0
var network: RefCounted
var planner: RefCounted
var station_ids: PackedStringArray
var pair_results: Dictionary = {}

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    network = CampusNetwork.new()
    planner = CampusRoutePlanner.new(network)
    station_ids = network.get_station_ids()
    _test_all_pairs_at_both_timesteps()
    _test_consecutive_journeys()

    if failures == 0:
        print("CampusRouteDriving tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d CampusRouteDriving assertions failed." % [failures, assertions])
        quit(1)

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _test_all_pairs_at_both_timesteps() -> void:
    for delta in FIXED_STEPS:
        var timestep_label: String = _timestep_label(delta)
        var successful_pairs: int = 0
        var maximum_cross_track_error_m: float = 0.0
        var maximum_endpoint_error_m: float = 0.0
        for origin_id in station_ids:
            for destination_id in station_ids:
                if origin_id == destination_id:
                    continue
                var plan: Dictionary = planner.plan_route(origin_id, destination_id)
                _expect(
                    plan.get("valid", false),
                    "%s -> %s plans at %s (%s)"
                        % [origin_id, destination_id, timestep_label, plan.get("failure_reason", "")]
                )
                if not plan.get("valid", false):
                    continue
                var route_result: Dictionary = _drive_fresh_route(plan, delta)
                var key: String = _pair_key(origin_id, destination_id, delta)
                pair_results[key] = route_result
                successful_pairs += 1
                maximum_cross_track_error_m = maxf(
                    maximum_cross_track_error_m,
                    route_result.maximum_cross_track_error_m
                )
                maximum_endpoint_error_m = maxf(
                    maximum_endpoint_error_m,
                    route_result.endpoint_error_m
                )
                _report_route_result(origin_id, destination_id, timestep_label, route_result)
                _expect(
                    route_result.arrived,
                    "%s -> %s reaches certified ARRIVED at %s"
                        % [origin_id, destination_id, timestep_label]
                )
                _expect(
                    route_result.failure_reason.is_empty(),
                    "%s -> %s has no controller or navigation failure at %s"
                        % [origin_id, destination_id, timestep_label]
                )
                _expect(
                    route_result.invalid_localization_count == 0,
                    "%s -> %s has no invalid localization at %s"
                        % [origin_id, destination_id, timestep_label]
                )
                _expect(
                    route_result.endpoint_error_m <= ENDPOINT_TOLERANCE_M,
                    "%s -> %s stops within %.2f m of its station at %s"
                        % [origin_id, destination_id, ENDPOINT_TOLERANCE_M, timestep_label]
                )
                _expect(
                    route_result.final_speed_mps <= ARRIVAL_SPEED_TOLERANCE_MPS + NUMERICAL_TOLERANCE,
                    "%s -> %s arrives at no more than %.2f m/s at %s"
                        % [
                            origin_id,
                            destination_id,
                            ARRIVAL_SPEED_TOLERANCE_MPS,
                            timestep_label
                        ]
                )
                _expect(
                    route_result.maximum_speed_mps
                        <= route_result.configured_max_speed_mps + NUMERICAL_TOLERANCE,
                    "%s -> %s stays within configured dynamics speed at %s"
                        % [origin_id, destination_id, timestep_label]
                )
                _expect(
                    route_result.maximum_steering_rad
                        <= route_result.steering_limit_rad + NUMERICAL_TOLERANCE,
                    "%s -> %s stays within steering limit at %s"
                        % [origin_id, destination_id, timestep_label]
                )
                _expect(
                    route_result.maximum_cross_track_error_m <= CROSS_TRACK_TOLERANCE_M,
                    "%s -> %s maximum cross-track error %.3f m is within %.2f m at %s"
                        % [
                            origin_id,
                            destination_id,
                            route_result.maximum_cross_track_error_m,
                            CROSS_TRACK_TOLERANCE_M,
                            timestep_label
                        ]
                )

        _expect(
            successful_pairs == 12,
            "all 12 ordered station pairs completed simulation setup at %s" % timestep_label
        )
        print(
            "TIMESTEP SUMMARY %s: routes=%d, maximum CTE=%.4f m, maximum endpoint error=%.4f m"
                % [
                    timestep_label,
                    successful_pairs,
                    maximum_cross_track_error_m,
                    maximum_endpoint_error_m
                ]
        )

func _drive_fresh_route(plan: Dictionary, delta: float) -> Dictionary:
    var dynamics = VehicleDynamics.new()
    var navigator = RouteNavigator.new()
    var controller = RouteFollowingController.new()
    var route_points: Array[Vector2] = plan.centerline_points
    var route_set: bool = navigator.set_route(route_points)
    if not route_set:
        return _empty_drive_result("route_navigator_rejected_centerline", plan)
    var samples: PackedVector2Array = navigator.get_centerline_samples()
    if samples.size() < 2:
        return _empty_drive_result("route_navigator_produced_no_tangent", plan)
    var first_tangent: Vector2 = (samples[1] - samples[0]).normalized()
    var initial_heading_rad: float = atan2(first_tangent.x, first_tangent.y)
    dynamics.reset(samples[0].y, 0.0, samples[0].x, initial_heading_rad)

    var result := _empty_drive_result("", plan)
    var terminal_status: String = ""
    for tick in range(MAX_SIMULATION_TICKS):
        var control: Dictionary = controller.update(dynamics, navigator, delta)
        result.tick_count = tick + 1
        result.maximum_cross_track_error_m = maxf(
            result.maximum_cross_track_error_m,
            absf(float(control.get("cross_track_error_m", 0.0)))
        )
        result.maximum_steering_rad = maxf(
            result.maximum_steering_rad,
            absf(dynamics.steering_rad)
        )
        var requested_steering_rad: float = float(control.get("requested_steering_rad", 0.0))
        result.maximum_steering_rad = maxf(
            result.maximum_steering_rad,
            absf(requested_steering_rad)
        )
        result.maximum_speed_mps = maxf(result.maximum_speed_mps, dynamics.speed_mps)
        result.maximum_route_progress_m = maxf(
            result.maximum_route_progress_m,
            float(control.get("route_progress_m", 0.0))
        )
        if not control.get("localized", false):
            result.invalid_localization_count += 1
        if control.status == "SAFE STOP / INVALID NAVIGATION" \
            || control.status == "MISSED DESTINATION":
            result.failure_reason = str(control.get("reason", control.status))
            terminal_status = control.status
            break
        if control.status == "ARRIVED" && control.get("arrival_certified", false):
            result.arrived = true
            terminal_status = control.status
            break
        dynamics.step(
            float(control.get("requested_acceleration_mps2", 0.0)),
            delta,
            requested_steering_rad
        )

    var destination_stop: Dictionary = network.get_station_stop(plan.destination_station_id)
    var final_position := Vector2(dynamics.position_x_m, dynamics.position_z_m)
    result.final_position_xz = final_position
    result.final_heading_rad = dynamics.heading_rad
    result.final_speed_mps = dynamics.speed_mps
    result.endpoint_error_m = final_position.distance_to(destination_stop.position_xz)
    result.terminal_status = terminal_status if not terminal_status.is_empty() else "TIMEOUT"
    result.configured_max_speed_mps = dynamics.max_speed
    result.steering_limit_rad = dynamics.steering_limit_rad
    return result

func _test_consecutive_journeys() -> void:
    _run_consecutive_journey(
        ["north_gate", "library", "sports_hall"],
        "north_gate -> library -> sports_hall"
    )
    _run_consecutive_journey(
        ["library", "north_gate", "innovation_centre"],
        "library -> north_gate -> innovation_centre"
    )
    _run_consecutive_journey(
        ["sports_hall", "library", "north_gate"],
        "sports_hall -> library -> north_gate"
    )

func _run_consecutive_journey(stations: Array[String], journey_name: String) -> void:
    var dynamics = VehicleDynamics.new()
    var navigator = RouteNavigator.new()
    var controller = RouteFollowingController.new()
    var journey_results: Array[Dictionary] = []
    var journey_failed: bool = false

    for leg_index in range(stations.size() - 1):
        var origin_id: String = stations[leg_index]
        var destination_id: String = stations[leg_index + 1]
        var plan: Dictionary = planner.plan_route(origin_id, destination_id)
        _expect(
            plan.get("valid", false),
            "%s leg %d planner succeeds (%s)"
                % [journey_name, leg_index + 1, plan.get("failure_reason", "")]
        )
        if not plan.get("valid", false):
            journey_failed = true
            break

        var route_points: Array[Vector2] = plan.centerline_points
        var next_samples_navigator = RouteNavigator.new()
        var next_route_set: bool = next_samples_navigator.set_route(route_points)
        _expect(
            next_route_set,
            "%s leg %d route installs into RouteNavigator" % [journey_name, leg_index + 1]
        )
        if not next_route_set:
            journey_failed = true
            break
        var next_samples: PackedVector2Array = next_samples_navigator.get_centerline_samples()
        var start_tangent: Vector2 = (next_samples[1] - next_samples[0]).normalized()
        var expected_start_heading_rad: float = atan2(start_tangent.x, start_tangent.y)

        if leg_index == 0:
            navigator.set_route(route_points)
            var first_samples: PackedVector2Array = navigator.get_centerline_samples()
            var initial_tangent: Vector2 = (first_samples[1] - first_samples[0]).normalized()
            dynamics.reset(
                first_samples[0].y,
                0.0,
                first_samples[0].x,
                atan2(initial_tangent.x, initial_tangent.y)
            )
        else:
            var actual_pose := Vector2(dynamics.position_x_m, dynamics.position_z_m)
            var next_start: Vector2 = next_samples[0]
            var position_error_m: float = actual_pose.distance_to(next_start)
            var heading_error_rad: float = absf(wrapf(
                dynamics.heading_rad - expected_start_heading_rad,
                -PI,
                PI
            ))
            var speed_compatible: bool = dynamics.speed_mps \
                <= ARRIVAL_SPEED_TOLERANCE_MPS + NUMERICAL_TOLERANCE
            var position_compatible: bool = position_error_m <= ENDPOINT_TOLERANCE_M
            var heading_compatible: bool = heading_error_rad \
                <= HEADING_COMPATIBILITY_TOLERANCE_RAD
            print(
                "LEG TRANSITION %s: pose error=%.4f m, heading error=%.3f deg, speed=%.4f m/s"
                    % [
                        journey_name,
                        position_error_m,
                        rad_to_deg(heading_error_rad),
                        dynamics.speed_mps
                    ]
            )
            _expect(
                position_compatible,
                "%s leg %d start is within %.2f m of actual previous-leg pose"
                    % [journey_name, leg_index + 1, ENDPOINT_TOLERANCE_M]
            )
            _expect(
                heading_compatible,
                "%s leg %d start tangent is within %.1f degrees of actual heading"
                    % [
                        journey_name,
                        leg_index + 1,
                        rad_to_deg(HEADING_COMPATIBILITY_TOLERANCE_RAD)
                    ]
            )
            _expect(
                speed_compatible,
                "%s leg %d begins only from near-zero station speed"
                    % [journey_name, leg_index + 1]
            )
            if not position_compatible || not heading_compatible || not speed_compatible:
                journey_failed = true
                break

            var pose_before_route_install := Vector2(
                dynamics.position_x_m,
                dynamics.position_z_m
            )
            var heading_before_route_install: float = dynamics.heading_rad
            navigator.set_route(route_points)
            var pose_after_route_install := Vector2(
                dynamics.position_x_m,
                dynamics.position_z_m
            )
            _expect(
                pose_after_route_install.distance_to(pose_before_route_install) <= 0.000001,
                "%s next-leg installation preserves actual vehicle position"
                    % journey_name
            )
            _expect(
                is_equal_approx(dynamics.heading_rad, heading_before_route_install),
                "%s next-leg installation preserves actual vehicle heading"
                    % journey_name
            )

        var leg_result: Dictionary = _drive_existing_vehicle(
            dynamics,
            navigator,
            controller,
            plan,
            FIXED_STEPS[0]
        )
        journey_results.append(leg_result)
        _report_route_result(
            origin_id,
            destination_id,
            "continuous 1/60 s",
            leg_result
        )
        _expect(
            leg_result.arrived,
            "%s leg %d arrives without resetting the vehicle"
                % [journey_name, leg_index + 1]
        )
        _expect(
            leg_result.failure_reason.is_empty(),
            "%s leg %d has no navigation failure" % [journey_name, leg_index + 1]
        )
        _expect(
            leg_result.endpoint_error_m <= ENDPOINT_TOLERANCE_M,
            "%s leg %d ends at its requested station" % [journey_name, leg_index + 1]
        )
        _expect(
            leg_result.final_speed_mps
                <= ARRIVAL_SPEED_TOLERANCE_MPS + NUMERICAL_TOLERANCE,
            "%s leg %d stops at the station" % [journey_name, leg_index + 1]
        )
        if not leg_result.arrived:
            journey_failed = true
            break

    print(
        "CONTINUOUS JOURNEY %s: legs=%d, success=%s"
            % [journey_name, journey_results.size(), not journey_failed]
    )
    _expect(
        journey_results.size() == stations.size() - 1 && not journey_failed,
        "%s completes both legs with continuous physical state" % journey_name
    )

func _drive_existing_vehicle(
    dynamics: RefCounted,
    navigator: RefCounted,
    controller: RefCounted,
    plan: Dictionary,
    delta: float
) -> Dictionary:
    var result := _empty_drive_result("", plan)
    var terminal_status: String = ""
    for tick in range(MAX_SIMULATION_TICKS):
        var control: Dictionary = controller.update(dynamics, navigator, delta)
        result.tick_count = tick + 1
        result.maximum_cross_track_error_m = maxf(
            result.maximum_cross_track_error_m,
            absf(float(control.get("cross_track_error_m", 0.0)))
        )
        result.maximum_steering_rad = maxf(
            result.maximum_steering_rad,
            absf(float(control.get("requested_steering_rad", 0.0)))
        )
        result.maximum_speed_mps = maxf(result.maximum_speed_mps, dynamics.speed_mps)
        result.maximum_route_progress_m = maxf(
            result.maximum_route_progress_m,
            float(control.get("route_progress_m", 0.0))
        )
        if not control.get("localized", false):
            result.invalid_localization_count += 1
        if control.status == "SAFE STOP / INVALID NAVIGATION" \
            || control.status == "MISSED DESTINATION":
            result.failure_reason = str(control.get("reason", control.status))
            terminal_status = control.status
            break
        if control.status == "ARRIVED" && control.get("arrival_certified", false):
            result.arrived = true
            terminal_status = control.status
            break
        dynamics.step(
            float(control.get("requested_acceleration_mps2", 0.0)),
            delta,
            float(control.get("requested_steering_rad", 0.0))
        )

    var final_position := Vector2(dynamics.position_x_m, dynamics.position_z_m)
    var destination_stop: Dictionary = network.get_station_stop(plan.destination_station_id)
    result.final_position_xz = final_position
    result.final_heading_rad = dynamics.heading_rad
    result.final_speed_mps = dynamics.speed_mps
    result.endpoint_error_m = final_position.distance_to(destination_stop.position_xz)
    result.terminal_status = terminal_status if not terminal_status.is_empty() else "TIMEOUT"
    result.configured_max_speed_mps = dynamics.max_speed
    result.steering_limit_rad = dynamics.steering_limit_rad
    return result

func _empty_drive_result(failure_reason: String, plan: Dictionary) -> Dictionary:
    return {
        "origin_station_id": plan.get("origin_station_id", ""),
        "destination_station_id": plan.get("destination_station_id", ""),
        "arrived": false,
        "terminal_status": "",
        "failure_reason": failure_reason,
        "tick_count": 0,
        "final_position_xz": Vector2.ZERO,
        "final_heading_rad": 0.0,
        "final_speed_mps": 0.0,
        "endpoint_error_m": INF,
        "maximum_cross_track_error_m": 0.0,
        "maximum_steering_rad": 0.0,
        "steering_limit_rad": 0.0,
        "maximum_speed_mps": 0.0,
        "configured_max_speed_mps": 0.0,
        "invalid_localization_count": 0,
        "maximum_route_progress_m": 0.0
    }

func _report_route_result(
    origin_id: String,
    destination_id: String,
    timestep_label: String,
    result: Dictionary
) -> void:
    print(
        "DRIVE RESULT %s -> %s [%s]: status=%s, ticks=%d, final=(%.3f, %.3f), heading=%.2f deg, speed=%.4f m/s, max_cte=%.4f m, endpoint=%.4f m, max_speed=%.4f m/s, max_steer=%.2f deg, invalid_localizations=%d, failure=%s"
            % [
                origin_id,
                destination_id,
                timestep_label,
                result.terminal_status,
                result.tick_count,
                result.final_position_xz.x,
                result.final_position_xz.y,
                rad_to_deg(result.final_heading_rad),
                result.final_speed_mps,
                result.maximum_cross_track_error_m,
                result.endpoint_error_m,
                result.maximum_speed_mps,
                rad_to_deg(result.maximum_steering_rad),
                result.invalid_localization_count,
                result.failure_reason
            ]
    )

func _pair_key(origin_id: String, destination_id: String, delta: float) -> String:
    return "%s>%s@%s" % [origin_id, destination_id, _timestep_label(delta)]

func _timestep_label(delta: float) -> String:
    if is_equal_approx(delta, 1.0 / 60.0):
        return "1/60 s"
    return "1/120 s"
