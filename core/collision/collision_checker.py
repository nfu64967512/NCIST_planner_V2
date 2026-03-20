"""
碰撞檢測器模組
提供點、線段、路徑的碰撞檢測功能
"""

import math
from typing import List, Tuple, Optional, Union
from dataclasses import dataclass
from abc import ABC, abstractmethod


# ==========================================
# 障礙物基類
# ==========================================
class Obstacle(ABC):
    """障礙物抽象基類"""
    
    def __init__(self, obstacle_id: str = ""):
        self.obstacle_id = obstacle_id
    
    @abstractmethod
    def contains_point(self, point: Tuple[float, float]) -> bool:
        """判斷點是否在障礙物內"""
        pass
    
    @abstractmethod
    def intersects_segment(self, p1: Tuple[float, float], 
                          p2: Tuple[float, float]) -> bool:
        """判斷線段是否與障礙物相交"""
        pass
    
    @abstractmethod
    def distance_to_point(self, point: Tuple[float, float]) -> float:
        """計算點到障礙物的最短距離"""
        pass


# ==========================================
# 圓形障礙物
# ==========================================
@dataclass
class CircleObstacle(Obstacle):
    """圓形障礙物"""
    
    center: Tuple[float, float]
    radius: float
    safety_margin: float = 0.0
    
    def __post_init__(self):
        super().__init__(f"Circle_{id(self)}")
    
    @property
    def effective_radius(self) -> float:
        """有效半徑（包含安全邊距）"""
        return self.radius + self.safety_margin
    
    def contains_point(self, point: Tuple[float, float]) -> bool:
        """判斷點是否在圓內"""
        distance = self.distance_to_point(point)
        return distance < self.effective_radius
    
    def intersects_segment(self, p1: Tuple[float, float], 
                          p2: Tuple[float, float]) -> bool:
        """判斷線段是否與圓相交"""
        # 計算線段到圓心的最短距離
        distance = self._point_to_segment_distance(self.center, p1, p2)
        return distance < self.effective_radius
    
    def distance_to_point(self, point: Tuple[float, float]) -> float:
        """計算點到圓邊界的距離"""
        px, py = point
        cx, cy = self.center
        
        # 計算點到圓心的距離
        distance_to_center = math.sqrt((px - cx)**2 + (py - cy)**2)
        
        # 減去半徑得到到邊界的距離
        return abs(distance_to_center - self.radius)
    
    def _point_to_segment_distance(self, point: Tuple[float, float],
                                   p1: Tuple[float, float],
                                   p2: Tuple[float, float]) -> float:
        """計算點到線段的最短距離"""
        px, py = point
        x1, y1 = p1
        x2, y2 = p2
        
        dx = x2 - x1
        dy = y2 - y1
        
        if abs(dx) < 1e-10 and abs(dy) < 1e-10:
            # 線段退化為點
            return math.sqrt((px - x1)**2 + (py - y1)**2)
        
        # 計算投影參數
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
        
        # 計算最近點
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        return math.sqrt((px - closest_x)**2 + (py - closest_y)**2)


# ==========================================
# 多邊形障礙物
# ==========================================
@dataclass
class PolygonObstacle(Obstacle):
    """多邊形障礙物"""
    
    vertices: List[Tuple[float, float]]
    safety_margin: float = 0.0
    
    def __post_init__(self):
        super().__init__(f"Polygon_{id(self)}")
        
        if len(self.vertices) < 3:
            raise ValueError("多邊形至少需要3個頂點")
    
    def contains_point(self, point: Tuple[float, float]) -> bool:
        """射線法判斷點是否在多邊形內"""
        px, py = point
        n = len(self.vertices)
        inside = False
        
        p1x, p1y = self.vertices[0]
        for i in range(1, n + 1):
            p2x, p2y = self.vertices[i % n]
            
            if py > min(p1y, p2y):
                if py <= max(p1y, p2y):
                    if px <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (py - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or px <= xinters:
                            inside = not inside
            
            p1x, p1y = p2x, p2y
        
        # 如果在內部，檢查是否在安全邊距內
        if inside and self.safety_margin > 0:
            # 計算到邊界的距離
            min_distance = self.distance_to_point(point)
            if min_distance < self.safety_margin:
                return True
        
        return inside
    
    def intersects_segment(self, p1: Tuple[float, float], 
                          p2: Tuple[float, float]) -> bool:
        """判斷線段是否與多邊形相交"""
        # 檢查線段端點是否在多邊形內
        if self.contains_point(p1) or self.contains_point(p2):
            return True
        
        # 檢查線段是否與多邊形任一邊相交
        n = len(self.vertices)
        for i in range(n):
            v1 = self.vertices[i]
            v2 = self.vertices[(i + 1) % n]
            
            if self._segments_intersect(p1, p2, v1, v2):
                return True
        
        return False
    
    def distance_to_point(self, point: Tuple[float, float]) -> float:
        """計算點到多邊形邊界的最短距離"""
        min_distance = float('inf')
        n = len(self.vertices)
        
        for i in range(n):
            v1 = self.vertices[i]
            v2 = self.vertices[(i + 1) % n]
            
            distance = self._point_to_segment_distance(point, v1, v2)
            min_distance = min(min_distance, distance)
        
        return min_distance
    
    def _segments_intersect(self, p1: Tuple[float, float], p2: Tuple[float, float],
                           p3: Tuple[float, float], p4: Tuple[float, float]) -> bool:
        """判斷兩條線段是否相交"""
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4
        
        dx1 = x2 - x1
        dy1 = y2 - y1
        dx2 = x4 - x3
        dy2 = y4 - y3
        
        det = dx1 * dy2 - dy1 * dx2
        
        if abs(det) < 1e-10:
            return False
        
        t = ((x3 - x1) * dy2 - (y3 - y1) * dx2) / det
        u = ((x3 - x1) * dy1 - (y3 - y1) * dx1) / det
        
        return 0 <= t <= 1 and 0 <= u <= 1
    
    def _point_to_segment_distance(self, point: Tuple[float, float],
                                   p1: Tuple[float, float],
                                   p2: Tuple[float, float]) -> float:
        """計算點到線段的最短距離"""
        px, py = point
        x1, y1 = p1
        x2, y2 = p2
        
        dx = x2 - x1
        dy = y2 - y1
        
        if abs(dx) < 1e-10 and abs(dy) < 1e-10:
            return math.sqrt((px - x1)**2 + (py - y1)**2)
        
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
        
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        return math.sqrt((px - closest_x)**2 + (py - closest_y)**2)


# ==========================================
# 碰撞檢測器
# ==========================================
class CollisionChecker:
    """碰撞檢測器"""
    
    def __init__(self):
        """初始化碰撞檢測器"""
        self.obstacles: List[Obstacle] = []
    
    def add_obstacle(self, obstacle: Obstacle):
        """添加障礙物"""
        self.obstacles.append(obstacle)
    
    def remove_obstacle(self, obstacle: Obstacle):
        """移除障礙物"""
        if obstacle in self.obstacles:
            self.obstacles.remove(obstacle)
    
    def clear_obstacles(self):
        """清除所有障礙物"""
        self.obstacles.clear()
    
    def check_point_collision(self, point: Tuple[float, float]) -> bool:
        """
        檢查點是否與任何障礙物碰撞
        
        參數:
            point: 點座標
        
        返回:
            是否碰撞
        """
        for obstacle in self.obstacles:
            if obstacle.contains_point(point):
                return True
        return False
    
    def check_segment_collision(self, p1: Tuple[float, float], 
                               p2: Tuple[float, float]) -> bool:
        """
        檢查線段是否與任何障礙物碰撞
        
        參數:
            p1, p2: 線段端點
        
        返回:
            是否碰撞
        """
        for obstacle in self.obstacles:
            if obstacle.intersects_segment(p1, p2):
                return True
        return False
    
    def check_path_collision(self, path: List[Tuple[float, float]]) -> bool:
        """
        檢查路徑是否與任何障礙物碰撞
        
        參數:
            path: 路徑點列表
        
        返回:
            是否碰撞
        """
        if len(path) < 2:
            return self.check_point_collision(path[0]) if path else False
        
        # 檢查每條線段
        for i in range(len(path) - 1):
            if self.check_segment_collision(path[i], path[i + 1]):
                return True
        
        return False
    
    def get_colliding_obstacles(self, point: Tuple[float, float]) -> List[Obstacle]:
        """
        獲取與點碰撞的所有障礙物
        
        參數:
            point: 點座標
        
        返回:
            碰撞的障礙物列表
        """
        return [obs for obs in self.obstacles if obs.contains_point(point)]
    
    def get_nearest_obstacle(self, point: Tuple[float, float]) -> Optional[Tuple[Obstacle, float]]:
        """
        獲取最近的障礙物及距離
        
        參數:
            point: 點座標
        
        返回:
            (最近障礙物, 距離) 或 None
        """
        if not self.obstacles:
            return None
        
        min_distance = float('inf')
        nearest_obstacle = None
        
        for obstacle in self.obstacles:
            distance = obstacle.distance_to_point(point)
            if distance < min_distance:
                min_distance = distance
                nearest_obstacle = obstacle
        
        return (nearest_obstacle, min_distance)
    
    def is_path_clear(self, path: List[Tuple[float, float]], 
                     min_clearance: float = 0.0) -> bool:
        """
        檢查路徑是否安全（有足夠間隙）
        
        參數:
            path: 路徑點列表
            min_clearance: 最小安全間隙（公尺）
        
        返回:
            是否安全
        """
        for point in path:
            for obstacle in self.obstacles:
                distance = obstacle.distance_to_point(point)
                if distance < min_clearance:
                    return False
        
        return True


# ==========================================
# 便捷函數
# ==========================================
def check_point_collision(point: Tuple[float, float], 
                         obstacles: List[Obstacle]) -> bool:
    """
    檢查點是否與障礙物碰撞
    
    參數:
        point: 點座標
        obstacles: 障礙物列表
    
    返回:
        是否碰撞
    """
    return any(obs.contains_point(point) for obs in obstacles)


def check_path_collision(path: List[Tuple[float, float]], 
                        obstacles: List[Obstacle]) -> bool:
    """
    檢查路徑是否與障礙物碰撞
    
    參數:
        path: 路徑點列表
        obstacles: 障礙物列表
    
    返回:
        是否碰撞
    """
    if len(path) < 2:
        return check_point_collision(path[0], obstacles) if path else False
    
    for i in range(len(path) - 1):
        for obstacle in obstacles:
            if obstacle.intersects_segment(path[i], path[i + 1]):
                return True
    
    return False
