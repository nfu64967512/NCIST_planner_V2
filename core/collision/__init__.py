"""
碰撞檢測與避障模組
提供碰撞檢測、避障策略、障礙物管理等功能
"""

from .collision_checker import (
    CollisionChecker,
    CircleObstacle,
    PolygonObstacle,
    check_point_collision,
    check_path_collision
)

from .avoidance import (
    AvoidanceStrategy,
    TangentAvoidance,
    APFAvoidance,
    calculate_safe_detour
)

from .obstacle_manager import (
    ObstacleManager,
    ObstacleBase,
    CircularObstacle,
    PolygonalObstacle
)

__all__ = [
    # Collision Checker
    'CollisionChecker',
    'CircleObstacle',
    'PolygonObstacle',
    'check_point_collision',
    'check_path_collision',
    
    # Avoidance
    'AvoidanceStrategy',
    'TangentAvoidance',
    'APFAvoidance',
    'calculate_safe_detour',
    
    # Obstacle Manager
    'ObstacleManager',
    'ObstacleBase',
    'CircularObstacle',
    'PolygonalObstacle'
]
