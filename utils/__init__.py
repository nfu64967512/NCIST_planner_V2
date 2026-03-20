"""
工具模組
提供數學計算、文件讀寫、日誌管理等基礎工具
"""

from .math_utils import (
    deg_to_rad,
    rad_to_deg,
    normalize_angle,
    haversine_distance,
    bearing_between_points,
    point_at_distance_bearing,
    rotate_point,
    line_intersection
)

from .file_io import (
    read_json,
    write_json,
    read_yaml,
    write_yaml,
    read_waypoints,
    write_waypoints
)

from .logger import (
    setup_logger,
    get_logger
)

__all__ = [
    # Math utilities
    'deg_to_rad',
    'rad_to_deg',
    'normalize_angle',
    'haversine_distance',
    'bearing_between_points',
    'point_at_distance_bearing',
    'rotate_point',
    'line_intersection',
    
    # File I/O
    'read_json',
    'write_json',
    'read_yaml',
    'write_yaml',
    'read_waypoints',
    'write_waypoints',
    
    # Logger
    'setup_logger',
    'get_logger'
]
