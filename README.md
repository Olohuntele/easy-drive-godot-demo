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
