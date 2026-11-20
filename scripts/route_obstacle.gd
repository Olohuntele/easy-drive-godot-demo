extends RefCounted

## Route-relative, non-physics obstacle used by the service route controller.
var route_id: String = ""
var segment_id: String = ""
var distance_along_route_m: float = 0.0
var length_m: float = 0.0
var world_position_xz: Vector2 = Vector2.ZERO
var world_heading_rad: float = 0.0
var width_m: float = 1.8
var active: bool = true

func _init(
    obstacle_distance_m: float = 0.0,
    obstacle_length_m: float = 4.0,
    obstacle_route_id: String = "",
    obstacle_segment_id: String = "",
    obstacle_world_position_xz: Vector2 = Vector2.ZERO,
    obstacle_width_m: float = 1.8,
    obstacle_world_heading_rad: float = 0.0
) -> void:
    distance_along_route_m = obstacle_distance_m
    length_m = obstacle_length_m
    route_id = obstacle_route_id
    segment_id = obstacle_segment_id
    world_position_xz = obstacle_world_position_xz
    world_heading_rad = obstacle_world_heading_rad
    width_m = obstacle_width_m

func duplicate_obstacle() -> RefCounted:
    var copy: RefCounted = get_script().new(
        distance_along_route_m,
        length_m,
        route_id,
        segment_id,
        world_position_xz,
        width_m,
        world_heading_rad
    )
    copy.active = active
    return copy
