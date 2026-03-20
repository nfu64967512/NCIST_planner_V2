"""
障礙物管理器 - 核心版本
提供障礙物的統一管理和查詢接口
與 collision_checker.py 配合使用
"""

import math
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass, field

from ..geometry import CoordinateTransform


@dataclass
class ObstacleBase:
    """障礙物基類"""
    id: str
    position: Tuple[float, float]  # 中心位置（經緯度）
    obstacle_type: str = "unknown"
    active: bool = True
    metadata: dict = field(default_factory=dict)
    
    def get_bounds(self) -> Tuple[float, float, float, float]:
        """
        獲取邊界框
        
        返回:
            (min_lat, min_lon, max_lat, max_lon)
        """
        raise NotImplementedError


@dataclass
class CircularObstacle(ObstacleBase):
    """圓形障礙物"""
    radius: float = 5.0  # 半徑（公尺）
    safety_margin: float = 1.0  # 安全邊距（公尺）
    
    def __post_init__(self):
        self.obstacle_type = "circular"
    
    @property
    def effective_radius(self) -> float:
        """有效半徑（包含安全邊距）"""
        return self.radius + self.safety_margin
    
    def get_bounds(self) -> Tuple[float, float, float, float]:
        """獲取邊界框（經緯度）"""
        lat, lon = self.position
        
        # 簡化計算：1度約111km
        lat_offset = self.effective_radius / 111111.0
        lon_offset = self.effective_radius / (111111.0 * math.cos(math.radians(lat)))
        
        return (
            lat - lat_offset,  # min_lat
            lon - lon_offset,  # min_lon
            lat + lat_offset,  # max_lat
            lon + lon_offset   # max_lon
        )
    
    def contains_point(self, point: Tuple[float, float]) -> bool:
        """
        檢查點是否在障礙物內
        
        參數:
            point: 點座標（經緯度）
        
        返回:
            是否在障礙物內
        """
        distance = self._calculate_distance(point, self.position)
        return distance <= self.effective_radius
    
    def intersects_segment(self, 
                          p1: Tuple[float, float],
                          p2: Tuple[float, float]) -> bool:
        """
        檢查線段是否與障礙物相交
        
        參數:
            p1: 線段起點（經緯度）
            p2: 線段終點（經緯度）
        
        返回:
            是否相交
        """
        # 計算點到線段的最短距離
        distance = self._point_to_segment_distance(self.position, p1, p2)
        return distance <= self.effective_radius
    
    def _calculate_distance(self, 
                          p1: Tuple[float, float],
                          p2: Tuple[float, float]) -> float:
        """計算兩點間距離（公尺）"""
        lat1, lon1 = p1
        lat2, lon2 = p2
        
        avg_lat = (lat1 + lat2) / 2
        dlat = (lat2 - lat1) * 111111.0
        dlon = (lon2 - lon1) * 111111.0 * math.cos(math.radians(avg_lat))
        
        return math.sqrt(dlat**2 + dlon**2)
    
    def _point_to_segment_distance(self,
                                  point: Tuple[float, float],
                                  seg_start: Tuple[float, float],
                                  seg_end: Tuple[float, float]) -> float:
        """計算點到線段的最短距離（公尺）"""
        # 轉換到平面座標
        center_lat = (point[0] + seg_start[0] + seg_end[0]) / 3
        center_lon = (point[1] + seg_start[1] + seg_end[1]) / 3
        transform = CoordinateTransform(center_lat, center_lon)
        
        px, py = transform.latlon_to_xy(point)
        sx, sy = transform.latlon_to_xy(seg_start)
        ex, ey = transform.latlon_to_xy(seg_end)
        
        # 線段向量
        dx = ex - sx
        dy = ey - sy
        
        if dx == 0 and dy == 0:
            # 退化為點
            return math.sqrt((px - sx)**2 + (py - sy)**2)
        
        # 參數 t 表示投影點在線段上的位置
        t = max(0, min(1, ((px - sx) * dx + (py - sy) * dy) / (dx**2 + dy**2)))
        
        # 投影點
        proj_x = sx + t * dx
        proj_y = sy + t * dy
        
        # 距離
        return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)


@dataclass
class PolygonalObstacle(ObstacleBase):
    """多邊形障礙物"""
    vertices: List[Tuple[float, float]] = field(default_factory=list)  # 頂點（經緯度）
    safety_margin: float = 1.0
    
    def __post_init__(self):
        self.obstacle_type = "polygonal"
        if not self.vertices and self.position:
            # 如果沒有頂點，創建一個默認的正方形
            self.vertices = self._create_default_polygon()
    
    def _create_default_polygon(self, size: float = 10.0) -> List[Tuple[float, float]]:
        """創建默認多邊形（正方形）"""
        lat, lon = self.position
        half_size = size / 2
        
        lat_offset = half_size / 111111.0
        lon_offset = half_size / (111111.0 * math.cos(math.radians(lat)))
        
        return [
            (lat - lat_offset, lon - lon_offset),  # 左下
            (lat - lat_offset, lon + lon_offset),  # 右下
            (lat + lat_offset, lon + lon_offset),  # 右上
            (lat + lat_offset, lon - lon_offset),  # 左上
        ]
    
    def get_bounds(self) -> Tuple[float, float, float, float]:
        """獲取邊界框"""
        if not self.vertices:
            return (0, 0, 0, 0)
        
        lats = [v[0] for v in self.vertices]
        lons = [v[1] for v in self.vertices]
        
        margin_lat = self.safety_margin / 111111.0
        margin_lon = self.safety_margin / (111111.0 * math.cos(math.radians(sum(lats)/len(lats))))
        
        return (
            min(lats) - margin_lat,
            min(lons) - margin_lon,
            max(lats) + margin_lat,
            max(lons) + margin_lon
        )
    
    def contains_point(self, point: Tuple[float, float]) -> bool:
        """檢查點是否在多邊形內（射線法）"""
        if not self.vertices:
            return False
        
        x, y = point
        n = len(self.vertices)
        inside = False
        
        p1x, p1y = self.vertices[0]
        for i in range(1, n + 1):
            p2x, p2y = self.vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
    
    def intersects_segment(self,
                          p1: Tuple[float, float],
                          p2: Tuple[float, float]) -> bool:
        """檢查線段是否與多邊形相交"""
        if not self.vertices:
            return False
        
        # 檢查線段端點是否在多邊形內
        if self.contains_point(p1) or self.contains_point(p2):
            return True
        
        # 檢查線段是否與多邊形的任何邊相交
        n = len(self.vertices)
        for i in range(n):
            v1 = self.vertices[i]
            v2 = self.vertices[(i + 1) % n]
            if self._segments_intersect(p1, p2, v1, v2):
                return True
        
        return False
    
    def _segments_intersect(self,
                          p1: Tuple[float, float],
                          p2: Tuple[float, float],
                          q1: Tuple[float, float],
                          q2: Tuple[float, float]) -> bool:
        """檢查兩線段是否相交"""
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
        
        return ccw(p1, q1, q2) != ccw(p2, q1, q2) and ccw(p1, p2, q1) != ccw(p1, p2, q2)


class ObstacleManager:
    """
    障礙物管理器
    
    功能:
    - 障礙物的添加、刪除、查詢
    - 空間索引（基於網格）
    - 批量碰撞檢測
    """
    
    def __init__(self, grid_size: float = 100.0):
        """
        初始化障礙物管理器
        
        參數:
            grid_size: 空間網格大小（公尺）
        """
        self.obstacles: List[ObstacleBase] = []
        self.obstacle_dict: dict = {}  # id -> obstacle
        self.grid_size = grid_size
        self.spatial_grid: dict = {}  # 空間索引
        self._next_id = 0
    
    def add_obstacle(self, obstacle: ObstacleBase) -> str:
        """
        添加障礙物
        
        參數:
            obstacle: 障礙物對象
        
        返回:
            障礙物 ID
        """
        # 分配 ID（如果沒有）
        if not obstacle.id:
            obstacle.id = f"obs_{self._next_id}"
            self._next_id += 1
        
        # 添加到列表和字典
        self.obstacles.append(obstacle)
        self.obstacle_dict[obstacle.id] = obstacle
        
        # 更新空間索引
        self._add_to_spatial_grid(obstacle)
        
        return obstacle.id
    
    def add_circular_obstacle(self,
                            position: Tuple[float, float],
                            radius: float,
                            safety_margin: float = 1.0,
                            metadata: dict = None) -> str:
        """
        添加圓形障礙物（便捷方法）
        
        參數:
            position: 中心位置（經緯度）
            radius: 半徑（公尺）
            safety_margin: 安全邊距（公尺）
            metadata: 元數據
        
        返回:
            障礙物 ID
        """
        obstacle = CircularObstacle(
            id="",
            position=position,
            radius=radius,
            safety_margin=safety_margin,
            metadata=metadata or {}
        )
        return self.add_obstacle(obstacle)
    
    def add_polygonal_obstacle(self,
                              vertices: List[Tuple[float, float]],
                              safety_margin: float = 1.0,
                              metadata: dict = None) -> str:
        """
        添加多邊形障礙物（便捷方法）
        
        參數:
            vertices: 頂點列表（經緯度）
            safety_margin: 安全邊距（公尺）
            metadata: 元數據
        
        返回:
            障礙物 ID
        """
        # 計算中心
        center_lat = sum(v[0] for v in vertices) / len(vertices)
        center_lon = sum(v[1] for v in vertices) / len(vertices)
        
        obstacle = PolygonalObstacle(
            id="",
            position=(center_lat, center_lon),
            vertices=vertices,
            safety_margin=safety_margin,
            metadata=metadata or {}
        )
        return self.add_obstacle(obstacle)
    
    def remove_obstacle(self, obstacle_id: str) -> bool:
        """
        移除障礙物
        
        參數:
            obstacle_id: 障礙物 ID
        
        返回:
            是否成功移除
        """
        if obstacle_id not in self.obstacle_dict:
            return False
        
        obstacle = self.obstacle_dict[obstacle_id]
        
        # 從列表移除
        self.obstacles.remove(obstacle)
        
        # 從字典移除
        del self.obstacle_dict[obstacle_id]
        
        # 從空間索引移除
        self._remove_from_spatial_grid(obstacle)
        
        return True
    
    def clear_all(self):
        """清除所有障礙物"""
        self.obstacles.clear()
        self.obstacle_dict.clear()
        self.spatial_grid.clear()
        self._next_id = 0
    
    def get_obstacle(self, obstacle_id: str) -> Optional[ObstacleBase]:
        """獲取障礙物"""
        return self.obstacle_dict.get(obstacle_id)
    
    def get_all_obstacles(self) -> List[ObstacleBase]:
        """獲取所有活躍障礙物"""
        return [obs for obs in self.obstacles if obs.active]
    
    def check_point_collision(self, point: Tuple[float, float]) -> bool:
        """
        檢查點是否與任何障礙物碰撞
        
        參數:
            point: 點座標（經緯度）
        
        返回:
            是否碰撞
        """
        # 使用空間索引加速查詢
        nearby_obstacles = self._get_nearby_obstacles(point)
        
        for obstacle in nearby_obstacles:
            if obstacle.active and obstacle.contains_point(point):
                return True
        
        return False
    
    def check_segment_collision(self,
                               p1: Tuple[float, float],
                               p2: Tuple[float, float]) -> bool:
        """
        檢查線段是否與任何障礙物碰撞
        
        參數:
            p1: 線段起點（經緯度）
            p2: 線段終點（經緯度）
        
        返回:
            是否碰撞
        """
        # 獲取線段附近的障礙物
        nearby_obstacles = self._get_nearby_obstacles_for_segment(p1, p2)
        
        for obstacle in nearby_obstacles:
            if obstacle.active and obstacle.intersects_segment(p1, p2):
                return True
        
        return False
    
    def find_obstacles_in_region(self,
                                bounds: Tuple[float, float, float, float]) -> List[ObstacleBase]:
        """
        查找區域內的障礙物
        
        參數:
            bounds: 邊界框 (min_lat, min_lon, max_lat, max_lon)
        
        返回:
            障礙物列表
        """
        min_lat, min_lon, max_lat, max_lon = bounds
        found_obstacles = []
        
        for obstacle in self.obstacles:
            if not obstacle.active:
                continue
            
            obs_bounds = obstacle.get_bounds()
            
            # 檢查邊界框是否重疊
            if (obs_bounds[2] >= min_lat and obs_bounds[0] <= max_lat and
                obs_bounds[3] >= min_lon and obs_bounds[1] <= max_lon):
                found_obstacles.append(obstacle)
        
        return found_obstacles
    
    def get_nearest_obstacle(self,
                           point: Tuple[float, float],
                           max_distance: float = float('inf')) -> Optional[ObstacleBase]:
        """
        獲取最近的障礙物
        
        參數:
            point: 查詢點（經緯度）
            max_distance: 最大搜索距離（公尺）
        
        返回:
            最近的障礙物（如果有）
        """
        min_distance = max_distance
        nearest_obstacle = None
        
        for obstacle in self.obstacles:
            if not obstacle.active:
                continue
            
            # 計算到障礙物中心的距離
            distance = self._calculate_distance(point, obstacle.position)
            
            if distance < min_distance:
                min_distance = distance
                nearest_obstacle = obstacle
        
        return nearest_obstacle
    
    def _add_to_spatial_grid(self, obstacle: ObstacleBase):
        """添加到空間網格"""
        bounds = obstacle.get_bounds()
        grid_cells = self._get_grid_cells_for_bounds(bounds)
        
        for cell in grid_cells:
            if cell not in self.spatial_grid:
                self.spatial_grid[cell] = []
            self.spatial_grid[cell].append(obstacle.id)
    
    def _remove_from_spatial_grid(self, obstacle: ObstacleBase):
        """從空間網格移除"""
        bounds = obstacle.get_bounds()
        grid_cells = self._get_grid_cells_for_bounds(bounds)
        
        for cell in grid_cells:
            if cell in self.spatial_grid:
                if obstacle.id in self.spatial_grid[cell]:
                    self.spatial_grid[cell].remove(obstacle.id)
    
    def _get_grid_cells_for_bounds(self,
                                   bounds: Tuple[float, float, float, float]) -> Set[Tuple[int, int]]:
        """獲取邊界框覆蓋的網格單元"""
        min_lat, min_lon, max_lat, max_lon = bounds
        
        # 轉換為網格座標
        min_i = int(min_lat * 111111.0 / self.grid_size)
        min_j = int(min_lon * 111111.0 / self.grid_size)
        max_i = int(max_lat * 111111.0 / self.grid_size)
        max_j = int(max_lon * 111111.0 / self.grid_size)
        
        cells = set()
        for i in range(min_i, max_i + 1):
            for j in range(min_j, max_j + 1):
                cells.add((i, j))
        
        return cells
    
    def _get_nearby_obstacles(self, point: Tuple[float, float]) -> List[ObstacleBase]:
        """獲取附近的障礙物（使用空間索引）"""
        lat, lon = point
        
        # 計算網格單元
        i = int(lat * 111111.0 / self.grid_size)
        j = int(lon * 111111.0 / self.grid_size)
        
        # 搜索當前單元和相鄰單元
        nearby_ids = set()
        for di in [-1, 0, 1]:
            for dj in [-1, 0, 1]:
                cell = (i + di, j + dj)
                if cell in self.spatial_grid:
                    nearby_ids.update(self.spatial_grid[cell])
        
        # 轉換為障礙物對象
        return [self.obstacle_dict[obs_id] for obs_id in nearby_ids 
                if obs_id in self.obstacle_dict]
    
    def _get_nearby_obstacles_for_segment(self,
                                         p1: Tuple[float, float],
                                         p2: Tuple[float, float]) -> List[ObstacleBase]:
        """獲取線段附近的障礙物"""
        # 簡化：使用端點的並集
        obstacles_set = set()
        
        for point in [p1, p2]:
            nearby = self._get_nearby_obstacles(point)
            obstacles_set.update(nearby)
        
        return list(obstacles_set)
    
    def _calculate_distance(self,
                          p1: Tuple[float, float],
                          p2: Tuple[float, float]) -> float:
        """計算兩點間距離（公尺）"""
        lat1, lon1 = p1
        lat2, lon2 = p2
        
        avg_lat = (lat1 + lat2) / 2
        dlat = (lat2 - lat1) * 111111.0
        dlon = (lon2 - lon1) * 111111.0 * math.cos(math.radians(avg_lat))
        
        return math.sqrt(dlat**2 + dlon**2)
    
    def get_statistics(self) -> dict:
        """
        獲取統計信息
        
        返回:
            統計字典
        """
        active_obstacles = [obs for obs in self.obstacles if obs.active]
        
        circular_count = sum(1 for obs in active_obstacles 
                           if isinstance(obs, CircularObstacle))
        polygonal_count = sum(1 for obs in active_obstacles 
                            if isinstance(obs, PolygonalObstacle))
        
        return {
            'total_obstacles': len(self.obstacles),
            'active_obstacles': len(active_obstacles),
            'circular_obstacles': circular_count,
            'polygonal_obstacles': polygonal_count,
            'grid_cells_used': len(self.spatial_grid)
        }
