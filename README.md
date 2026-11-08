# Easy Drive Godot Demo

An independent Godot 4.7 Compatibility project demonstrating a campus shuttle
and a coordinator-driven passenger-service journey. The project runs entirely in
GDScript, with its scene resources contained in this repository.

## Run

Open this folder in Godot 4.7 and run the project. Its entry scene is
`scenes/main.tscn`.

## Demo

The scene includes a generated road, a four-wheeled vehicle, a follow camera,
and a telemetry HUD. The existing GDScript simulation supports a blocked-road
scenario, a clear-road destination scenario, manual driving, autonomous curve
navigation, and a coordinator-driven passenger-service demonstration.

- `1`: blocked-road scenario
- `2`: clear-road scenario
- `3`: manual steering demonstration
- `4`: autonomous route following
- `5` / `6` / `7`: LEFT / RIGHT / S-CURVE route variants in Mode 4
- `8`: start an independent Mode 5 passenger service shuttle at North Gate
- `Space` or `Enter`: toggle autonomous/manual control
- `W` / Up: accelerate in manual mode
- `S` / Down: brake in manual mode
- `A` / `D`: steer in manual mode
- `R`: reset the simulation

Existing driving modes and vehicle dynamics are preserved. Mode 5 adds a
separate coordinator-driven passenger-service demonstration without changing
the vehicle physics model.

### Mode 5 — passenger service demo

Mode 5 uses `CampusServiceCoordinator` as the sole motion authority while
active. Its `VehicleDynamics` state drives the visible shuttle and the existing
smoothed follow camera; the legacy mode dynamics are not stepped in parallel.
The demonstration booking is fixed to Sports Hall pickup, Innovation Centre
destination, and North Gate service return. There is no booking form.

- `B`: request the demonstration booking
- `E`: confirm boarding after certified arrival at Sports Hall
- `X`: confirm passenger exit after certified arrival at Innovation Centre
- `R`: reinitialize only when the service shuttle is available and stopped at North Gate
- `1`–`4`: return to an existing mode only when no booking is active

On Mode 5 entry, a fresh service coordinator initializes its own stopped
shuttle at the actual North Gate position and aligns it once to the legal
outgoing campus-road tangent. The legacy vehicle state is left unchanged. If a
legacy demonstration was moving, it is suspended while Mode 5 is active; only
the coordinator advances. Later route handoffs preserve service-shuttle
position, heading, and speed; no reset or alignment occurs between journey
legs.

The Mode 5 HUD shows booking, journey, route leg, target, speed, arrival/
confirmation status, and service availability. An occupied or blocked booking
cannot be reset or abandoned by switching modes. Pressing `8` during an active
Mode 5 journey does not restart it. Its lightweight 3D roads are generated from
`CampusNetwork` segments, the active highlight uses the coordinator's installed
`RouteNavigator` centerline, and a top-right plan map labels the four stations.
The visualization does not certify road-width containment or physical lane
keeping.

The Mode 5 booking panel lists pickup and destination choices directly from
`CampusNetwork`. It submits selected station IDs through
`CampusServiceCoordinator.request_booking`; the dispatcher and coordinator
remain authoritative for reservation and route acceptance. Identical stops
are rejected with inline feedback. After acceptance, the same panel shows the
selected trip, coordinator-reported journey stage, and progress through pickup,
boarding, destination, exit, and return. Board Shuttle and Exit Shuttle appear
only when the coordinator snapshot reports the matching certified arrival;
they call the existing coordinator confirmation APIs. The booking form returns
after the certified service return makes the shuttle idle and available.
The same buttons supplement, rather than replace, the existing `E` boarding
and `X` exit shortcuts.

The `B` shortcut remains available for the original Sports Hall-to-Innovation
Centre demonstration booking. When a booking control has keyboard focus,
simulation shortcuts are paused; press `Esc` to release focus and resume `B`,
`E`, `X`, `R`, and mode shortcuts.

### Manual visual check

1. Run the project in the Godot GUI and press `8` from any legacy mode, including while its shuttle is away from North Gate.
2. Choose different stops and request a shuttle; confirm the panel switches to journey tracking and shows the selected trip.
3. Check that progress/status update as the shuttle travels to pickup, waits for boarding, travels to destination, waits for exit, and returns.
4. Confirm Board Shuttle appears only at certified pickup arrival; click it and verify destination travel begins.
5. Confirm Exit Shuttle appears only at certified destination arrival; click it and verify the shuttle returns to service.
6. Verify the booking form returns only after certified North Gate return and availability.
7. Try both Sports Hall → Innovation Centre and Library → North Gate; check camera tracking, map/station alignment, and route handoff continuity.
8. Check `B`, `E`, `X`, and `R`, then try `1`–`4`; verify the existing shortcuts and Modes 1–4 remain functional.

## Passenger service foundation

The standalone `ShuttleDispatcher` and `PassengerJourney` scripts validate
one booking at a time and model pickup, boarding, destination, drop-off, and
return-to-service transitions. Mode 5 connects them to the running scene
through `CampusServiceCoordinator`.
The dispatcher requires valid pickup, passenger, and return routes; the
journey returns route-install requests for an external `RouteNavigator` caller
and requires that caller to confirm installation without changing vehicle
pose. Station arrival is accepted only with controller certification (or a
validated stationary pickup), stop proximity, low speed, valid localization,
and no manual intervention.

For route handoffs after the shuttle has moved, use
`CampusRoutePlanner.plan_route_from_pose()` with the actual station position
and heading, then pass that route to
`PassengerJourney.request_next_route_installation()`. The dispatcher reserves
station-to-station route availability but cannot predict later arrival
headings; each leg should be selected from the then-current vehicle pose.

## Passenger journey coordinator

`CampusServiceCoordinator` connects the dispatcher and journey state machine
to pose-aware route planning, `RouteNavigator`, `RouteFollowingController`, and
`VehicleDynamics`. It remains testable headlessly and powers the visual Mode 5
demonstration in `scenes/main.tscn`.

Create a coordinator and call `initialize_service(station_id, position_xz,
heading_rad)` once with the shuttle's initial pose at its service station.
Then use `request_booking(pickup_station_id, destination_station_id)`,
`step(delta)`, `confirm_boarding()`,
`confirm_passenger_exit()`, and `get_service_snapshot()`. Provide a unique
non-negative physics tick ID as an optional second `step()` argument for
custom/headless loops; otherwise the coordinator uses Godot's current physics
frame to reject duplicate calls within that tick. Arrival and return-to-service
are certified from controller telemetry and the actual dynamics snapshot;
route handoffs do not reset or modify vehicle pose. Failed route execution,
manual interruption, or lost localization leaves the booking reserved in a
blocked state.
