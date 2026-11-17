extends RefCounted

## Route-relative, non-physics obstacle used by the service route controller.
var route_id: String = ""
var segment_id: String = ""
var distance_along_route_m: float = 0.0
var length_m: float = 0.0
var active: bool = true

func _init(
    obstacle_distance_m: float = 0.0,
    obstacle_length_m: float = 4.0,
    obstacle_route_id: String = "",
    obstacle_segment_id: String = ""
) -> void:
    distance_along_route_m = obstacle_distance_m
    length_m = obstacle_length_m
    route_id = obstacle_route_id
    segment_id = obstacle_segment_id

func duplicate_obstacle() -> RefCounted:
    var copy: RefCounted = get_script().new(
        distance_along_route_m,
        length_m,
        route_id,
        segment_id
    )
    copy.active = active
    return copy
