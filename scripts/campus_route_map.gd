extends Control

const MAP_MARGIN: float = 22.0
const WORLD_MIN_X: float = -52.0
const WORLD_MAX_X: float = 52.0
const WORLD_MIN_Z: float = -12.0
const WORLD_MAX_Z: float = 92.0

var _network: RefCounted
var _active_route: PackedVector2Array = PackedVector2Array()

func configure(network: RefCounted) -> void:
    _network = network
    custom_minimum_size = Vector2(280.0, 280.0)
    size = custom_minimum_size
    anchor_left = 1.0
    anchor_right = 1.0
    offset_left = -300.0
    offset_right = -20.0
    offset_top = 18.0
    offset_bottom = 298.0
    mouse_filter = Control.MOUSE_FILTER_IGNORE
    queue_redraw()

func set_active_route(points: PackedVector2Array) -> void:
    _active_route = points.duplicate()
    queue_redraw()

func _draw() -> void:
    if _network == null:
        return
    var panel := Rect2(Vector2.ZERO, size)
    draw_rect(panel, Color(0.035, 0.055, 0.075, 0.88), true)
    draw_rect(panel, Color(0.55, 0.68, 0.76, 0.95), false, 1.5)

    for segment in _network.get_segments():
        var points := PackedVector2Array()
        for world_point in segment.centerline_points:
            points.append(_world_to_map(world_point))
        draw_polyline(points, Color(0.19, 0.23, 0.26, 1.0), 11.0, true)
        draw_polyline(points, Color(0.68, 0.72, 0.73, 1.0), 6.0, true)

    if _active_route.size() >= 2:
        var route_points := PackedVector2Array()
        for point in _active_route:
            route_points.append(_world_to_map(point))
        draw_polyline(route_points, Color(1.0, 0.63, 0.12, 1.0), 3.5, true)

    for station in _network.get_stations():
        var station_position: Vector2 = _world_to_map(station.stop_position_xz)
        draw_circle(station_position, 6.0, Color(0.12, 0.72, 0.9, 1.0))
        draw_arc(
            station_position,
            7.0,
            0.0,
            TAU,
            20,
            Color(0.92, 0.97, 1.0, 1.0),
            1.2,
            true
        )
        var label: String = str(station.display_name).replace(" Stop", "")
        var offset := Vector2(8.0, -8.0)
        match station.id:
            "north_gate":
                offset = Vector2(-4.0, 17.0)
            "sports_hall":
                offset = Vector2(8.0, -9.0)
            "library":
                offset = Vector2(-108.0, -9.0)
            "innovation_centre":
                offset = Vector2(8.0, -9.0)
        draw_string(
            ThemeDB.fallback_font,
            station_position + offset,
            label,
            HORIZONTAL_ALIGNMENT_LEFT,
            120.0,
            12,
            Color(0.96, 0.98, 1.0, 1.0)
        )

    draw_string(
        ThemeDB.fallback_font,
        Vector2(10.0, 17.0),
        "CAMPUS ROUTE MAP",
        HORIZONTAL_ALIGNMENT_LEFT,
        180.0,
        11,
        Color(0.82, 0.9, 0.95, 1.0)
    )

func _world_to_map(point: Vector2) -> Vector2:
    var map_extent: float = minf(size.x, size.y) - MAP_MARGIN * 2.0
    var scale: float = map_extent / maxf(
        WORLD_MAX_X - WORLD_MIN_X,
        WORLD_MAX_Z - WORLD_MIN_Z
    )
    return Vector2(
        MAP_MARGIN + (point.x - WORLD_MIN_X) * scale,
        MAP_MARGIN + (WORLD_MAX_Z - point.y) * scale
    )
