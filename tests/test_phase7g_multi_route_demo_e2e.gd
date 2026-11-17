extends SceneTree

const FIXED_STEP: float = 1.0 / 60.0
const MAX_TICKS: int = 12000
const PassengerJourney = preload("res://scripts/passenger_journey.gd")
const VehicleDimensions = preload("res://scripts/offset_path_builder.gd")

var failures: int = 0
var assertions: int = 0

func _initialize() -> void:
	call_deferred("_run_tests")

func _run_tests() -> void:
	await _test_visual_box_never_moves_and_general_clearance()
	await _test_rejected_bypass_holds_each_leg()
	await _test_accepted_bypass_completes_each_leg()
	await _test_rejected_hold_clears_and_completes_each_leg()
	if failures == 0:
		print("Phase 7G multi-route demo end-to-end tests passed: %d assertions." % assertions)
		quit(0)
	else:
		push_error("%d of %d Phase 7G assertions failed." % [failures, assertions])
		quit(1)

func _expect(condition: bool, message: String) -> void:
	assertions += 1
	if condition:
		print("PASS: " + message)
	else:
		push_error("FAIL: " + message)
		failures += 1

func _new_demo() -> Node:
	var packed: PackedScene = load("res://scenes/multi_route_demo.tscn")
	var demo: Node = packed.instantiate()
	root.add_child(demo)
	demo.set_physics_process(false)
	return demo

func _step(demo: Node, tick: int) -> Dictionary:
	var coordinator = demo.service_coordinator
	coordinator.step(FIXED_STEP, tick)
	var snapshot: Dictionary = coordinator.get_service_snapshot()
	demo.ego_car.position = Vector3(snapshot.position_xz.x, 0.2, snapshot.position_xz.y)
	demo.ego_car.rotation.y = snapshot.heading_rad
	demo._update_obstacle_visual(snapshot)
	demo._update_hud()
	return snapshot

func _advance_to_state(demo: Node, target_state: String, tick_start: int) -> int:
	var tick: int = tick_start
	for index in range(MAX_TICKS):
		var snapshot: Dictionary = _step(demo, tick)
		var state: String = snapshot.service_state
		if state == PassengerJourney.BOARDING:
			demo.service_coordinator.confirm_boarding()
		if state == PassengerJourney.DROPOFF:
			demo.service_coordinator.confirm_passenger_exit()
		if state == target_state:
			return tick
		if state == PassengerJourney.BLOCKED:
			return -1
		tick += 1
	return -1

func _prepare_leg(demo: Node, leg: String, tick: int) -> int:
	demo.service_coordinator.request_booking("sports_hall", "innovation_centre")
	var target_state: String = PassengerJourney.TO_PICKUP
	if leg == "TO_DESTINATION":
		target_state = PassengerJourney.TO_DESTINATION
	elif leg == "RETURNING_TO_SERVICE":
		target_state = PassengerJourney.RETURNING_TO_SERVICE
	return _advance_to_state(demo, target_state, tick)

func _place_mid(demo: Node) -> void:
	var coordinator = demo.service_coordinator
	var snapshot: Dictionary = coordinator.get_service_snapshot()
	var navigation = coordinator.navigator.find_nearest_point(snapshot.position_xz, true)
	var progress: float = navigation.progress_m
	var distance: float = coordinator.navigator.total_length_m - 8.0
	if snapshot.service_state == PassengerJourney.TO_PICKUP:
		distance = lerpf(progress + 1.0, coordinator.navigator.total_length_m - 12.0, 0.5)
	if distance <= progress:
		distance = progress + 5.0
	coordinator.place_obstacle(distance, 4.0, "", "", 5.0)
	demo._update_obstacle_visual(coordinator.get_service_snapshot())

func _vehicle_rectangle(demo: Node) -> Array[Vector2]:
	var pos: Vector2 = Vector2(demo.ego_car.global_position.x, demo.ego_car.global_position.z)
	var heading: float = demo.ego_car.rotation.y
	var forward: Vector2 = Vector2(sin(heading), cos(heading))
	var right: Vector2 = Vector2(forward.y, -forward.x)
	return _rectangle(pos, forward, right, VehicleDimensions.VEHICLE_LENGTH_M, VehicleDimensions.VEHICLE_WIDTH_M)

func _obstacle_rectangle(demo: Node) -> Array[Vector2]:
	var obstacle = demo.service_coordinator.active_obstacle
	if obstacle == null:
		return []
	var pos: Vector2 = Vector2(demo.obstacle_visual.global_position.x, demo.obstacle_visual.global_position.z)
	var forward: Vector2 = Vector2(sin(demo.obstacle_visual.rotation.y), cos(demo.obstacle_visual.rotation.y))
	var right: Vector2 = Vector2(forward.y, -forward.x)
	return _rectangle(pos, forward, right, obstacle.length_m, obstacle.width_m)

func _rectangle(center: Vector2, forward: Vector2, right: Vector2, length: float, width: float) -> Array[Vector2]:
	return [
		center + forward*length*0.5 + right*width*0.5,
		center + forward*length*0.5 - right*width*0.5,
		center - forward*length*0.5 - right*width*0.5,
		center - forward*length*0.5 + right*width*0.5
	]

func _oriented_clearance(a: Array[Vector2], b: Array[Vector2]) -> float:
	if a.is_empty() or b.is_empty():
		return INF
	var axes: Array[Vector2] = [(a[1]-a[0]).normalized(), (a[0]-a[3]).normalized(), (b[1]-b[0]).normalized(), (b[0]-b[3]).normalized()]
	var maximum_gap: float = 0.0
	var minimum_overlap: float = INF
	for axis in axes:
		var ap: Vector2 = _project(a, axis)
		var bp: Vector2 = _project(b, axis)
		var gap: float = maxf(bp.x-ap.y, ap.x-bp.y)
		if gap > 0.0:
			maximum_gap = maxf(maximum_gap, gap)
		else:
			minimum_overlap = minf(minimum_overlap, minf(ap.y,bp.y)-maxf(ap.x,bp.x))
	return maximum_gap if maximum_gap > 0.0 else -minimum_overlap

func _project(points: Array[Vector2], axis: Vector2) -> Vector2:
	var mn: float = INF
	var mx: float = -INF
	for point in points:
		var value: float = point.dot(axis)
		mn = minf(mn, value)
		mx = maxf(mx, value)
	return Vector2(mn,mx)

func _test_visual_box_never_moves_and_general_clearance() -> void:
	for leg in ["TO_PICKUP", "TO_DESTINATION", "RETURNING_TO_SERVICE"]:
		var demo: Node = _new_demo()
		var tick: int = _prepare_leg(demo, leg, 1)
		if tick >= 0:
			demo._place_obstacle_pressed()
			var before: Vector3 = demo.obstacle_visual.global_position
			var moved: bool = false
			var overlap: bool = false
			for index in range(900):
				_step(demo, tick + index)
				if demo.obstacle_visual.global_position.distance_to(before) > 0.001:
					moved = true
				if _oriented_clearance(_vehicle_rectangle(demo), _obstacle_rectangle(demo)) < -0.001:
					overlap = true
				if demo.service_coordinator.lane_change_active:
					break
			_expect(not moved, leg + " visual box remains at its fixed world position")
			_expect(not overlap, leg + " EgoCar footprint never overlaps active visual box")
		demo.queue_free()
		await process_frame

func _test_rejected_bypass_holds_each_leg() -> void:
	for leg in ["TO_PICKUP", "TO_DESTINATION", "RETURNING_TO_SERVICE"]:
		var demo: Node = _new_demo()
		var tick: int = _prepare_leg(demo, leg, 1)
		if tick >= 0:
			for warmup in range(90):
				_step(demo, tick + warmup)
			tick += 90
			_place_mid(demo)
			var persist: bool = false
			for index in range(1200):
				var snapshot: Dictionary = _step(demo, tick + index)
				if snapshot.no_way_hold_active:
					persist = true
				if snapshot.no_way_hold_active and "NO CLEAR PATH - WAITING" in demo.hud_label.text:
					pass
			_expect(persist, leg + " rejected bypass reaches NO CLEAR PATH - WAITING hold")
			if persist:
				var snapshot: Dictionary = demo.service_coordinator.get_service_snapshot()
				_expect("NO CLEAR PATH - WAITING" in demo.hud_label.text, leg + " HUD displays waiting while hold persists")
			_demo_clear_and_finish(demo, tick + 300)
		demo.queue_free()
		await process_frame

func _test_accepted_bypass_completes_each_leg() -> void:
	for leg in ["TO_PICKUP", "TO_DESTINATION", "RETURNING_TO_SERVICE"]:
		var demo: Node = _new_demo()
		var tick: int = _prepare_leg(demo, leg, 1)
		if tick >= 0:
			demo._place_obstacle_pressed()
			var accepted: bool = false
			var completed: bool = false
			var overlap: bool = false
			for index in range(MAX_TICKS):
				var snapshot: Dictionary = _step(demo, tick + index)
				if snapshot.lane_change_active:
					accepted = true
				if _oriented_clearance(_vehicle_rectangle(demo), _obstacle_rectangle(demo)) < -0.001:
					overlap = true
				if leg == "TO_PICKUP" and snapshot.service_state == PassengerJourney.BOARDING:
					completed = true
					break
				if leg == "TO_DESTINATION" and snapshot.service_state == PassengerJourney.DROPOFF:
					completed = true
					break
				if leg == "RETURNING_TO_SERVICE" and snapshot.service_state == PassengerJourney.IDLE:
					completed = true
					break
				if snapshot.blocked:
					break
			_expect(accepted, leg + " accepted bypass starts lane change")
			_expect(not overlap, leg + " accepted bypass has no footprint overlap")
			_expect(completed, leg + " accepted-bypass journey completes")
		demo.queue_free()
		await process_frame

func _test_rejected_hold_clears_and_completes_each_leg() -> void:
	for leg in ["TO_PICKUP", "TO_DESTINATION", "RETURNING_TO_SERVICE"]:
		var demo: Node = _new_demo()
		var tick: int = _prepare_leg(demo, leg, 1)
		if tick >= 0:
			for warmup in range(90):
				_step(demo, tick + warmup)
			tick += 90
			_place_mid(demo)
			for index in range(180):
				_step(demo, tick + index)
			demo.clear_obstacle_button.emit_signal("pressed")
			var completed: bool = _demo_clear_and_finish(demo, tick + 180)
			_expect(completed, leg + " clearing rejected hold resumes and completes journey")
		demo.queue_free()
		await process_frame

func _demo_clear_and_finish(demo: Node, tick: int) -> bool:
	demo.service_coordinator.clear_obstacle()
	demo._update_obstacle_visual(demo.service_coordinator.get_service_snapshot())
	var leg: String = demo.service_coordinator.journey.state
	for index in range(MAX_TICKS):
		var snapshot: Dictionary = _step(demo, tick + index)
		if leg == PassengerJourney.TO_PICKUP and snapshot.service_state == PassengerJourney.BOARDING:
			return true
		if leg == PassengerJourney.TO_DESTINATION and snapshot.service_state == PassengerJourney.DROPOFF:
			return true
		if leg == PassengerJourney.RETURNING_TO_SERVICE and snapshot.service_state == PassengerJourney.IDLE:
			return true
		if snapshot.blocked:
			return false
	return false
