# Easy Drive Godot Demo

An independent Godot 4.7 Compatibility project demonstrating a campus shuttle
on a straight road. The project runs entirely in GDScript, with its scene
resources contained in this repository.

## Run

Open this folder in Godot 4.7 and run the project. Its entry scene is
`scenes/main.tscn`.

## Demo

The scene includes a generated road, a four-wheeled vehicle, a follow camera,
and a telemetry HUD. The existing GDScript simulation supports a blocked-road
scenario and a clear-road destination scenario.

- `1`: blocked-road scenario
- `2`: clear-road scenario
- `Space` or `Enter`: toggle autonomous/manual control
- `W` / Up: accelerate in manual mode
- `S` / Down: brake in manual mode
- `R`: reset the simulation

The simulation and scene are intentionally preserved from the committed
Godot project baseline; this repository separation does not add driving
features or redesign the vehicle.

## Passenger service foundation

The standalone `ShuttleDispatcher` and `PassengerJourney` scripts validate
one booking at a time and model pickup, boarding, destination, drop-off, and
return-to-service transitions. They are not integrated into the running scene.
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

## Headless passenger journey coordinator

`CampusServiceCoordinator` connects the dispatcher and journey state machine
to pose-aware route planning, `RouteNavigator`, `RouteFollowingController`, and
`VehicleDynamics`. It is a standalone headless API and is not wired into
`scenes/main.tscn`.

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
