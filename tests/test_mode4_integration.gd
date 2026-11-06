extends SceneTree

const RouteFollowingController = preload("res://scripts/route_following_controller.gd")
const RouteNavigator = preload("res://scripts/route_navigator.gd")
const VehicleDynamics = preload("res://scripts/vehicle_dynamics.gd")

const FIXED_STEP: float = 1.0 / 60.0

var failures: int = 0
var assertions: int = 0
var simulation: Node

func _initialize() -> void:
    call_deferred("_run_tests")

func _run_tests() -> void:
    await _create_simulation()
    _test_mode_selection_and_key4_edge()
    _test_route_initialization_and_reset()
    _test_one_dynamics_step_per_tick()
    _test_vehicle_visual_tracks_authoritative_heading()
    _test_left_right_and_s_routes()
    _test_existing_mode_selectors()
    _test_manual_certification_latch()

    simulation.queue_free()
    await process_frame
    if failures == 0:
        print("Mode 4 integration tests passed: %d assertions." % assertions)
        quit(0)
    else:
        push_error("%d of %d Mode 4 integration assertions failed." % [failures, assertions])
        quit(1)

func _create_simulation() -> void:
    var packed_scene: PackedScene = load("res://scenes/main.tscn")
    simulation = packed_scene.instantiate()
    root.add_child(simulation)
    simulation.set_physics_process(false)
    await process_frame

func _expect(condition: bool, message: String) -> void:
    assertions += 1
    if condition:
        print("PASS: " + message)
    else:
        push_error("FAIL: " + message)
        failures += 1

func _test_mode_selection_and_key4_edge() -> void:
    _expect(simulation.key4_press_edge(true), "first key-4 press is detected")
    _expect(not simulation.key4_press_edge(true), "holding key 4 does not retrigger selection")
    _expect(not simulation.key4_press_edge(false), "key release does not select a mode")
    _expect(simulation.key4_press_edge(true), "a new key-4 press after release is detected")
    simulation.key4_press_edge(false)

    simulation.select_route_following_mode()
    _expect(simulation.autonomous_route_mode, "Mode 4 selector activates autonomous route following")
    _expect(not simulation.steering_test_mode, "Mode 4 exits manual steering-test mode")
    _expect(not simulation.scenario_blocked_road, "Mode 4 disables the legacy blocked-road scenario")
    _expect(simulation.route_navigation_ready, "Mode 4 initializes the broad-bend route")
    _expect(simulation.driving_controller.auto_drive_enabled, "Mode 4 restores autonomous control")
    _expect(not simulation.route_following_controller.manual_intervention_seen, "Mode 4 clears the manual-intervention latch")

func _test_route_initialization_and_reset() -> void:
    var samples: PackedVector2Array = simulation.route_navigator.get_centerline_samples()
    _expect(simulation.vehicle_dynamics.position_x_m == samples[0].x, "Mode 4 starts at route world X")
    _expect(simulation.vehicle_dynamics.position_z_m == samples[0].y, "Mode 4 starts at route world Z")
    var initial_tangent: Vector2 = samples[1] - samples[0]
    var expected_heading: float = atan2(initial_tangent.x, initial_tangent.y)
    _expect(
        is_equal_approx(simulation.vehicle_dynamics.heading_rad, expected_heading),
        "Mode 4 initial heading follows the route's initial tangent"
    )

    simulation._physics_process(FIXED_STEP)
    simulation._physics_process(FIXED_STEP)
    _expect(simulation.route_navigator.progress_m > 0.0, "route progress advances after a physics tick")
    simulation.vehicle_dynamics.reset(40.0, 3.0, 8.0, 0.5)
    simulation.route_following_controller.manual_intervention_seen = true
    simulation.reset_simulation()
    _expect(
        simulation.vehicle_dynamics.position_x_m == samples[0].x
            && simulation.vehicle_dynamics.position_z_m == samples[0].y,
        "R-equivalent reset restores route starting position"
    )
    _expect(is_equal_approx(simulation.route_navigator.progress_m, 0.0), "route reset clears navigation progress")
    _expect(not simulation.route_navigator.is_localized, "route reset requires fresh localization")
    _expect(not simulation.route_following_controller.manual_intervention_seen, "route reset clears manual certification latch")
    _expect(simulation.driving_controller.auto_drive_enabled, "route reset restores autonomous mode")

func _test_one_dynamics_step_per_tick() -> void:
    simulation.select_route_following_mode()
    var reference_dynamics = VehicleDynamics.new()
    var reference_navigator = RouteNavigator.new()
    reference_navigator.set_route(RouteNavigator.create_broad_bend_waypoints())
    var samples: PackedVector2Array = reference_navigator.get_centerline_samples()
    var tangent: Vector2 = samples[1] - samples[0]
    reference_dynamics.reset(samples[0].y, 0.0, samples[0].x, atan2(tangent.x, tangent.y))
    var reference_controller = RouteFollowingController.new()
    var expected: Dictionary = reference_controller.update(
        reference_dynamics,
        reference_navigator,
        FIXED_STEP
    )
    reference_dynamics.step(
        expected.requested_acceleration_mps2,
        FIXED_STEP,
        expected.requested_steering_rad
    )

    simulation._physics_process(FIXED_STEP)
    _expect(
        is_equal_approx(simulation.vehicle_dynamics.position_x_m, reference_dynamics.position_x_m)
            && is_equal_approx(simulation.vehicle_dynamics.position_z_m, reference_dynamics.position_z_m)
            && is_equal_approx(simulation.vehicle_dynamics.heading_rad, reference_dynamics.heading_rad)
            && is_equal_approx(simulation.vehicle_dynamics.speed_mps, reference_dynamics.speed_mps),
        "one Mode 4 physics tick matches exactly one controller update and dynamics step"
    )
    _expect(
        simulation.vehicle_dynamics.speed_mps <= simulation.ego_max_accel * FIXED_STEP + 0.00001,
        "first physics tick cannot contain a duplicate acceleration step"
    )

func _test_vehicle_visual_tracks_authoritative_heading() -> void:
    simulation.select_route_following_mode()
    var saw_curved_steering: bool = false
    var yaw_changed: bool = false
    var visual_heading_matches_dynamics: bool = true
    var visual_forward_matches_dynamics: bool = true
    var camera_yaw_stays_world_aligned: bool = true
    var maximum_heading_rad: float = 0.0
    var final_camera_forward: Vector3 = Vector3.ZERO
    for i in range(3000):
        simulation._physics_process(FIXED_STEP)
        var dynamics = simulation.vehicle_dynamics
        var ego_car: Node3D = simulation.ego_car
        if absf(dynamics.steering_rad) > deg_to_rad(0.5):
            saw_curved_steering = true
        if absf(dynamics.heading_rad) > deg_to_rad(2.0):
            yaw_changed = true
        maximum_heading_rad = maxf(maximum_heading_rad, absf(dynamics.heading_rad))
        visual_heading_matches_dynamics = visual_heading_matches_dynamics \
            && is_equal_approx(ego_car.rotation.y, dynamics.heading_rad)
        var expected_forward := Vector3(
            sin(dynamics.heading_rad),
            0.0,
            cos(dynamics.heading_rad)
        )
        var visual_forward: Vector3 = ego_car.global_basis * Vector3(0.0, 0.0, 1.0)
        visual_forward_matches_dynamics = visual_forward_matches_dynamics \
            && visual_forward.normalized().dot(expected_forward.normalized()) > 0.999
        var camera_forward: Vector3 = simulation.follow_camera.global_basis * Vector3.FORWARD
        final_camera_forward = camera_forward.normalized()
        camera_yaw_stays_world_aligned = camera_yaw_stays_world_aligned \
            && absf(final_camera_forward.x) < 0.001 && final_camera_forward.z > 0.99
        if dynamics.speed_mps == 0.0 && dynamics.position_z_m > 90.0:
            break
    print("MODE 4 MAX DYNAMICS HEADING: %.2f deg" % rad_to_deg(maximum_heading_rad))
    print("MODE 4 CAMERA FORWARD: %s" % final_camera_forward)
    _expect(saw_curved_steering, "Mode 4 issues nonzero steering along the bend")
    _expect(yaw_changed, "Mode 4 dynamics heading changes during curved-route tracking")
    _expect(visual_heading_matches_dynamics, "vehicle visual yaw stays equal to dynamics heading")
    _expect(visual_forward_matches_dynamics, "vehicle model's local +Z forward follows dynamics heading")
    _expect(camera_yaw_stays_world_aligned, "Mode 4 camera remains world-aligned to reveal vehicle yaw")
    _expect(
        simulation.ego_car.rotation.y > deg_to_rad(2.0),
        "right-bending route rotates the shuttle toward world +X"
    )

func _test_existing_mode_selectors() -> void:
    simulation.select_blocked_road_mode()
    _expect(simulation.scenario_blocked_road && not simulation.autonomous_route_mode, "Mode 1 selector preserves blocked-road mode")
    _expect(not simulation.steering_test_mode, "Mode 1 exits steering-test mode")

    simulation.select_clear_road_mode()
    _expect(not simulation.scenario_blocked_road && not simulation.autonomous_route_mode, "Mode 2 selector preserves clear-road mode")

    simulation.select_route_following_mode()
    simulation.toggle_steering_test_mode()
    _expect(simulation.steering_test_mode, "Mode 3 selector still activates manual steering demo")
    _expect(not simulation.autonomous_route_mode, "Mode 3 remains separate from autonomous route mode")
    simulation.toggle_steering_test_mode()
    _expect(not simulation.steering_test_mode, "Mode 3 toggle still exits steering demo")
    _expect(not simulation.route_visual_root.visible, "Modes 1-3 hide generated Mode 4 road geometry")
    _expect(simulation.straight_road.visible, "Modes 1-3 preserve the existing straight road")
    _expect(simulation.straight_centerline.visible, "Modes 1-3 preserve the existing straight-road centerline")
    _expect(simulation.straight_roadside_scenery.visible, "Modes 1-3 preserve straight-road roadside scenery")

func _test_left_right_and_s_routes() -> void:
    var route_names: Array[String] = [
        simulation.ROUTE_LEFT,
        simulation.ROUTE_RIGHT,
        simulation.ROUTE_S_CURVE
    ]
    for route_name in route_names:
        simulation.select_route_following_mode()
        simulation.select_route_variant(route_name)
        _expect(simulation.route_navigation_ready, "%s route initializes" % route_name)
        _expect(simulation.route_visual_root.visible, "%s route displays generated road" % route_name)
        _expect(not simulation.straight_road.visible, "%s route hides the straight road" % route_name)
        _expect(
            not simulation.straight_roadside_scenery.visible,
            "%s route hides straight-road roadside scenery" % route_name
        )
        _expect(
            simulation.route_visual_centerline_xz == simulation.route_navigator.get_centerline_samples(),
            "%s road visualization uses the navigator's sampled centerline" % route_name
        )
        _expect(_route_surface_matches_centerline(), "%s road surface is centered on navigation samples" % route_name)
        _expect(
            simulation.route_visual_root.get_node("RouteLeftBoundary").mesh != null
                && simulation.route_visual_root.get_node("RouteRightBoundary").mesh != null
                && simulation.route_visual_root.get_node("RouteCenterline").mesh != null,
            "%s road has two boundaries and a centerline marking" % route_name
        )

        simulation._physics_process(FIXED_STEP)
        var first_visual_forward: Vector3 = simulation.ego_car.global_basis * Vector3(0.0, 0.0, 1.0)
        var first_expected_forward := Vector3(
            sin(simulation.vehicle_dynamics.heading_rad),
            0.0,
            cos(simulation.vehicle_dynamics.heading_rad)
        )
        var visual_heading_matches_dynamics: bool = is_equal_approx(
            simulation.ego_car.rotation.y,
            simulation.vehicle_dynamics.heading_rad
        ) && first_visual_forward.normalized().dot(first_expected_forward.normalized()) > 0.999
        var previous_progress: float = simulation.route_navigator.progress_m
        var progress_never_reversed: bool = true
        var maximum_cross_track_error_m: float = 0.0
        var maximum_speed_mps: float = 0.0
        var saw_positive_steering: bool = false
        var saw_negative_steering: bool = false
        var saw_endpoint_braking: bool = false
        var saw_heading_change: bool = false
        var camera_stayed_world_aligned: bool = true
        var arrived: bool = false
        for _tick in range(6000):
            var dynamics = simulation.vehicle_dynamics
            var decision: Dictionary = simulation.route_following_controller.update(
                dynamics,
                simulation.route_navigator,
                FIXED_STEP
            )
            dynamics.step(
                decision.requested_acceleration_mps2,
                FIXED_STEP,
                decision.requested_steering_rad
            )
            var progress_m: float = simulation.route_navigator.progress_m
            progress_never_reversed = progress_never_reversed && progress_m + 0.0001 >= previous_progress
            previous_progress = progress_m
            maximum_speed_mps = maxf(maximum_speed_mps, dynamics.speed_mps)
            if progress_m >= 8.0:
                var location: Dictionary = simulation.route_navigator.find_nearest_point(
                    Vector2(dynamics.position_x_m, dynamics.position_z_m)
                )
                maximum_cross_track_error_m = maxf(maximum_cross_track_error_m, location.distance_m)
            saw_positive_steering = saw_positive_steering || dynamics.steering_rad > deg_to_rad(0.5)
            saw_negative_steering = saw_negative_steering || dynamics.steering_rad < -deg_to_rad(0.5)
            saw_heading_change = saw_heading_change || absf(dynamics.heading_rad) > deg_to_rad(2.0)
            saw_endpoint_braking = saw_endpoint_braking || (
                dynamics.speed_mps > 0.1
                && dynamics.actual_acceleration_mps2 <= -dynamics.max_braking + 0.001
                && simulation.route_navigator.total_length_m - progress_m < 4.0
            )
            var camera_forward: Vector3 = simulation.follow_camera.global_basis * Vector3.FORWARD
            camera_stayed_world_aligned = camera_stayed_world_aligned \
                && absf(camera_forward.normalized().x) < 0.001 \
                && camera_forward.normalized().z > 0.99
            var endpoint: Vector2 = simulation.route_navigator.get_point_at_distance(
                simulation.route_navigator.total_length_m
            )
            if progress_m >= simulation.route_navigator.total_length_m - 1.0 \
                && Vector2(dynamics.position_x_m, dynamics.position_z_m).distance_to(endpoint) <= 1.0 \
                && dynamics.speed_mps <= 0.1:
                arrived = true
                break

        var final_decision: Dictionary = simulation.route_following_controller.update(
            simulation.vehicle_dynamics,
            simulation.route_navigator,
            FIXED_STEP
        )
        print(
            "MODE 4 %s: progress %.2f/%.2f m, max CTE %.3f m, max speed %.3f m/s, final %s"
            % [
                route_name,
                simulation.route_navigator.progress_m,
                simulation.route_navigator.total_length_m,
                maximum_cross_track_error_m,
                maximum_speed_mps,
                final_decision.status
            ]
        )
        if route_name == simulation.ROUTE_LEFT:
            _expect(saw_negative_steering && not saw_positive_steering, "left route requests negative steering")
        elif route_name == simulation.ROUTE_RIGHT:
            _expect(saw_positive_steering && not saw_negative_steering, "right route requests positive steering")
        else:
            _expect(saw_positive_steering && saw_negative_steering, "S route requests both steering directions")
        _expect(saw_heading_change, "%s route changes heading through VehicleDynamics" % route_name)
        _expect(
            visual_heading_matches_dynamics,
            "%s scene update keeps the vehicle visual aligned to dynamics heading" % route_name
        )
        _expect(progress_never_reversed, "%s route progress never jumps backward" % route_name)
        _expect(maximum_cross_track_error_m <= 0.75, "%s route stays within 0.75 m cross-track tolerance" % route_name)
        _expect(maximum_speed_mps <= 4.05, "%s route stays within configured navigation speed" % route_name)
        _expect(saw_endpoint_braking, "%s route brakes before the endpoint" % route_name)
        _expect(arrived && final_decision.status == "ARRIVED", "%s route reaches a valid stopped arrival" % route_name)
        _expect(camera_stayed_world_aligned, "%s route preserves the Mode 4 world-aligned camera" % route_name)

        simulation.route_following_controller.update(
            simulation.vehicle_dynamics,
            simulation.route_navigator,
            FIXED_STEP,
            true
        )
        var manual_arrival: Dictionary = simulation.route_following_controller.update(
            simulation.vehicle_dynamics,
            simulation.route_navigator,
            FIXED_STEP
        )
        _expect(
            manual_arrival.status != "ARRIVED" && not manual_arrival.arrival_certified,
            "%s manual intervention prevents arrival certification" % route_name
        )
        simulation.reset_simulation()
        _expect(
            simulation.route_following_controller.manual_intervention_seen == false
                && simulation.vehicle_dynamics.speed_mps == 0.0
                && is_equal_approx(simulation.route_navigator.progress_m, 0.0),
            "%s reset clears route progress, speed, and manual-intervention latch" % route_name
        )

func _route_surface_matches_centerline() -> bool:
    var arrays: Array = simulation.route_surface_mesh.surface_get_arrays(0)
    var vertices: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
    var samples: PackedVector2Array = simulation.route_visual_centerline_xz
    if vertices.size() != samples.size() * 2:
        return false
    for index in range(samples.size()):
        var midpoint := (vertices[index * 2] + vertices[index * 2 + 1]) * 0.5
        if midpoint.distance_to(Vector3(samples[index].x, 0.055, samples[index].y)) > 0.0001:
            return false
    return true

func _test_manual_certification_latch() -> void:
    simulation.select_route_following_mode()
    var endpoint: Vector2 = simulation.route_navigator.get_point_at_distance(
        simulation.route_navigator.total_length_m
    )
    simulation.vehicle_dynamics.reset(endpoint.y, 0.0, endpoint.x)
    simulation.route_navigator.reset_progress()
    var manual_update: Dictionary = simulation.route_following_controller.update(
        simulation.vehicle_dynamics,
        simulation.route_navigator,
        FIXED_STEP,
        true
    )
    _expect(manual_update.status == "EN ROUTE", "manual intervention prevents Mode 4 ARRIVED status")
    _expect(not manual_update.arrival_certified, "manual intervention prevents Mode 4 arrival certification")
    simulation.reset_simulation()
    _expect(not simulation.route_following_controller.manual_intervention_seen, "R-equivalent reset allows a fresh autonomous run")
