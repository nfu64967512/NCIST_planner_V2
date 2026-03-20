"""
避障策略模組
提供多種避障策略：切線法、人工勢場法等
"""

import math
from typing import List, Tuple, Optional
from abc import ABC, abstractmethod

from .collision_checker import Obstacle, CircleObstacle


# ==========================================
# 避障策略基類
# ==========================================
class AvoidanceStrategy(ABC):
    """避障策略抽象基類"""
    
    @abstractmethod
    def calculate_detour(self, start: Tuple[float, float],
                        goal: Tuple[float, float],
                        obstacle: Obstacle) -> List[Tuple[float, float]]:
        """
        計算繞行路徑
        
        參數:
            start: 起點
            goal: 終點
            obstacle: 障礙物
        
        返回:
            繞行航點列表
        """
        pass


# ==========================================
# 切線避障法
# ==========================================
class TangentAvoidance(AvoidanceStrategy):
    """切線避障法（適用於圓形障礙物）"""
    
    def __init__(self, safety_margin: float = 1.0):
        """
        初始化切線避障
        
        參數:
            safety_margin: 額外安全邊距（公尺）
        """
        self.safety_margin = safety_margin
    
    def calculate_detour(self, start: Tuple[float, float],
                        goal: Tuple[float, float],
                        obstacle: Obstacle) -> List[Tuple[float, float]]:
        """計算切線繞行路徑"""
        if not isinstance(obstacle, CircleObstacle):
            # 僅支援圓形障礙物
            return [start, goal]
        
        # 計算有效半徑
        radius = obstacle.effective_radius + self.safety_margin
        center = obstacle.center
        
        # 計算切線點
        tangent_points = self._calculate_tangent_points(start, center, radius, goal)
        
        if not tangent_points:
            # 無法計算切線，返回直線
            return [start, goal]
        
        # 選擇最短路徑
        return self._select_shortest_path(start, goal, tangent_points)
    
    def _calculate_tangent_points(self, start: Tuple[float, float],
                                  center: Tuple[float, float],
                                  radius: float,
                                  goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """計算從起點到圓的切線點"""
        sx, sy = start
        cx, cy = center
        gx, gy = goal
        
        # 計算起點到圓心的距離
        dx = cx - sx
        dy = cy - sy
        d_start = math.sqrt(dx * dx + dy * dy)
        
        if d_start <= radius:
            # 起點在圓內，無法計算切線
            return []
        
        # 計算切線點
        # 使用幾何方法：切線點到圓心連線垂直於切線
        a = radius * radius / d_start
        h = math.sqrt(radius * radius - a * a)
        
        # 計算中間點
        px = sx + a * dx / d_start
        py = sy + a * dy / d_start
        
        # 計算兩個切線點
        t1x = px + h * dy / d_start
        t1y = py - h * dx / d_start
        
        t2x = px - h * dy / d_start
        t2y = py + h * dx / d_start
        
        # 計算從切線點到終點的距離，選擇更接近終點方向的切線點
        dist1 = (t1x - gx) ** 2 + (t1y - gy) ** 2
        dist2 = (t2x - gx) ** 2 + (t2y - gy) ** 2
        
        if dist1 < dist2:
            return [(t1x, t1y)]
        else:
            return [(t2x, t2y)]
    
    def _select_shortest_path(self, start: Tuple[float, float],
                             goal: Tuple[float, float],
                             tangent_points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """選擇最短繞行路徑"""
        if not tangent_points:
            return [start, goal]
        
        # 簡單策略：使用第一個切線點
        return [start, tangent_points[0], goal]


# ==========================================
# 人工勢場避障法
# ==========================================
class APFAvoidance(AvoidanceStrategy):
    """人工勢場法避障"""
    
    def __init__(self, repulsive_gain: float = 10.0,
                attractive_gain: float = 1.0,
                influence_distance: float = 10.0):
        """
        初始化人工勢場避障
        
        參數:
            repulsive_gain: 斥力增益
            attractive_gain: 引力增益
            influence_distance: 影響距離（公尺）
        """
        self.repulsive_gain = repulsive_gain
        self.attractive_gain = attractive_gain
        self.influence_distance = influence_distance
    
    def calculate_detour(self, start: Tuple[float, float],
                        goal: Tuple[float, float],
                        obstacle: Obstacle) -> List[Tuple[float, float]]:
        """使用人工勢場計算繞行路徑"""
        # 簡化實現：生成若干中間點，受勢場影響調整
        num_waypoints = 5
        path = []
        
        # 在起點和終點間插入中間點
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            x = start[0] + t * (goal[0] - start[0])
            y = start[1] + t * (goal[1] - start[1])
            
            # 計算勢場力
            force = self._calculate_force((x, y), goal, obstacle)
            
            # 根據勢場力調整位置
            adjusted_x = x + force[0] * 0.5
            adjusted_y = y + force[1] * 0.5
            
            path.append((adjusted_x, adjusted_y))
        
        return path
    
    def _calculate_force(self, point: Tuple[float, float],
                        goal: Tuple[float, float],
                        obstacle: Obstacle) -> Tuple[float, float]:
        """計算點受到的合力"""
        # 引力（指向目標）
        attractive_force = self._calculate_attractive_force(point, goal)
        
        # 斥力（遠離障礙物）
        repulsive_force = self._calculate_repulsive_force(point, obstacle)
        
        # 合力
        total_force = (
            attractive_force[0] + repulsive_force[0],
            attractive_force[1] + repulsive_force[1]
        )
        
        return total_force
    
    def _calculate_attractive_force(self, point: Tuple[float, float],
                                    goal: Tuple[float, float]) -> Tuple[float, float]:
        """計算引力"""
        dx = goal[0] - point[0]
        dy = goal[1] - point[1]
        distance = math.sqrt(dx * dx + dy * dy)
        
        if distance < 1e-6:
            return (0.0, 0.0)
        
        # 引力正比於距離
        fx = self.attractive_gain * dx / distance
        fy = self.attractive_gain * dy / distance
        
        return (fx, fy)
    
    def _calculate_repulsive_force(self, point: Tuple[float, float],
                                   obstacle: Obstacle) -> Tuple[float, float]:
        """計算斥力"""
        # 計算點到障礙物的距離
        if isinstance(obstacle, CircleObstacle):
            cx, cy = obstacle.center
            dx = point[0] - cx
            dy = point[1] - cy
            distance = math.sqrt(dx * dx + dy * dy) - obstacle.effective_radius
        else:
            distance = obstacle.distance_to_point(point)
        
        if distance > self.influence_distance or distance < 1e-6:
            return (0.0, 0.0)
        
        # 斥力反比於距離平方
        magnitude = self.repulsive_gain * (1.0 / distance - 1.0 / self.influence_distance) / (distance * distance)
        
        # 方向：遠離障礙物
        if isinstance(obstacle, CircleObstacle):
            cx, cy = obstacle.center
            dx = point[0] - cx
            dy = point[1] - cy
        else:
            # 簡化：使用障礙物中心（多邊形情況需要更複雜的計算）
            dx = point[0] - obstacle.vertices[0][0]
            dy = point[1] - obstacle.vertices[0][1]
        
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < 1e-6:
            return (0.0, 0.0)
        
        fx = magnitude * dx / dist
        fy = magnitude * dy / dist
        
        return (fx, fy)


# ==========================================
# 便捷函數
# ==========================================
def calculate_safe_detour(start: Tuple[float, float],
                         goal: Tuple[float, float],
                         obstacles: List[Obstacle],
                         strategy: Optional[AvoidanceStrategy] = None) -> List[Tuple[float, float]]:
    """
    計算安全繞行路徑
    
    參數:
        start: 起點
        goal: 終點
        obstacles: 障礙物列表
        strategy: 避障策略（如未指定，使用切線法）
    
    返回:
        繞行路徑
    """
    if strategy is None:
        strategy = TangentAvoidance()
    
    # 檢查直線路徑是否安全
    path = [start, goal]
    
    for obstacle in obstacles:
        if obstacle.intersects_segment(start, goal):
            # 需要繞行
            detour = strategy.calculate_detour(start, goal, obstacle)
            if len(detour) > 2:
                path = detour
                break
    
    return path


def smooth_detour_path(path: List[Tuple[float, float]], 
                      smoothing_factor: float = 0.3) -> List[Tuple[float, float]]:
    """
    平滑繞行路徑
    
    參數:
        path: 原始路徑
        smoothing_factor: 平滑係數（0-1）
    
    返回:
        平滑後的路徑
    """
    if len(path) < 3:
        return path
    
    smoothed = [path[0]]  # 保持起點
    
    for i in range(1, len(path) - 1):
        prev = path[i - 1]
        curr = path[i]
        next_pt = path[i + 1]
        
        # 計算平滑點
        smooth_x = curr[0] + smoothing_factor * ((prev[0] + next_pt[0]) / 2 - curr[0])
        smooth_y = curr[1] + smoothing_factor * ((prev[1] + next_pt[1]) / 2 - curr[1])
        
        smoothed.append((smooth_x, smooth_y))
    
    smoothed.append(path[-1])  # 保持終點
    
    return smoothed
