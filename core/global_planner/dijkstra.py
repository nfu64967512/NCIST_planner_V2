"""
Dijkstra 和 A* 路徑規劃器
基於圖搜索的最短路徑演算法
"""

import math
import heapq
from typing import List, Tuple, Optional, Dict, Set, Callable
from dataclasses import dataclass, field

from ..collision import CollisionChecker


@dataclass(order=True)
class PriorityNode:
    """優先隊列節點"""
    priority: float
    position: Tuple[float, float] = field(compare=False)
    parent: Optional[Tuple[float, float]] = field(default=None, compare=False)
    g_cost: float = field(default=0.0, compare=False)


class GridMap:
    """柵格地圖"""
    
    def __init__(self, 
                 bounds: Tuple[float, float, float, float],
                 resolution: float,
                 collision_checker: CollisionChecker):
        """
        初始化柵格地圖
        
        參數:
            bounds: 地圖邊界 (min_x, min_y, max_x, max_y)
            resolution: 柵格分辨率（公尺）
            collision_checker: 碰撞檢測器
        """
        self.min_x, self.min_y, self.max_x, self.max_y = bounds
        self.resolution = resolution
        self.collision_checker = collision_checker
        
        # 計算柵格大小
        self.width = int((self.max_x - self.min_x) / resolution) + 1
        self.height = int((self.max_y - self.min_y) / resolution) + 1
        
        # 建立柵格地圖
        self.grid = [[False] * self.width for _ in range(self.height)]
        self._build_grid()
    
    def _build_grid(self):
        """建立柵格地圖（標記障礙物）"""
        for i in range(self.height):
            for j in range(self.width):
                x = self.min_x + j * self.resolution
                y = self.min_y + i * self.resolution
                
                # 檢查是否與障礙物碰撞
                if self.collision_checker.check_point_collision((x, y)):
                    self.grid[i][j] = True  # True 表示有障礙物
    
    def is_valid(self, x: float, y: float) -> bool:
        """
        檢查座標是否有效（在地圖內且無障礙物）
        
        參數:
            x, y: 座標
        
        返回:
            是否有效
        """
        # 檢查是否在邊界內
        if not (self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y):
            return False
        
        # 轉換為柵格索引
        i = int((y - self.min_y) / self.resolution)
        j = int((x - self.min_x) / self.resolution)
        
        # 檢查邊界
        if i < 0 or i >= self.height or j < 0 or j >= self.width:
            return False
        
        # 檢查是否有障礙物
        return not self.grid[i][j]
    
    def get_neighbors(self, position: Tuple[float, float], 
                     use_diagonal: bool = True) -> List[Tuple[float, float]]:
        """
        獲取鄰居節點
        
        參數:
            position: 當前位置
            use_diagonal: 是否使用對角線移動
        
        返回:
            鄰居位置列表
        """
        x, y = position
        neighbors = []
        
        # 4方向移動
        directions = [
            (self.resolution, 0),      # 右
            (-self.resolution, 0),     # 左
            (0, self.resolution),      # 上
            (0, -self.resolution)      # 下
        ]
        
        # 8方向移動（包括對角線）
        if use_diagonal:
            directions.extend([
                (self.resolution, self.resolution),      # 右上
                (self.resolution, -self.resolution),     # 右下
                (-self.resolution, self.resolution),     # 左上
                (-self.resolution, -self.resolution)     # 左下
            ])
        
        for dx, dy in directions:
            new_x = x + dx
            new_y = y + dy
            
            if self.is_valid(new_x, new_y):
                neighbors.append((new_x, new_y))
        
        return neighbors
    
    def snap_to_grid(self, position: Tuple[float, float]) -> Tuple[float, float]:
        """
        將位置對齊到柵格
        
        參數:
            position: 原始位置
        
        返回:
            對齊後的位置
        """
        x, y = position
        grid_x = round((x - self.min_x) / self.resolution) * self.resolution + self.min_x
        grid_y = round((y - self.min_y) / self.resolution) * self.resolution + self.min_y
        return (grid_x, grid_y)


class DijkstraPlanner:
    """Dijkstra 最短路徑規劃器"""
    
    def __init__(self, 
                 grid_map: GridMap,
                 use_diagonal: bool = True):
        """
        初始化Dijkstra規劃器
        
        參數:
            grid_map: 柵格地圖
            use_diagonal: 是否使用對角線移動
        """
        self.grid_map = grid_map
        self.use_diagonal = use_diagonal
    
    def plan(self, 
             start: Tuple[float, float],
             goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        規劃從起點到終點的最短路徑
        
        參數:
            start: 起點座標
            goal: 終點座標
        
        返回:
            路徑點列表，如果找不到路徑則返回None
        """
        # 對齊到柵格
        start = self.grid_map.snap_to_grid(start)
        goal = self.grid_map.snap_to_grid(goal)
        
        # 檢查起點和終點是否有效
        if not self.grid_map.is_valid(start[0], start[1]):
            return None
        if not self.grid_map.is_valid(goal[0], goal[1]):
            return None
        
        # 初始化
        open_set = []
        heapq.heappush(open_set, PriorityNode(0.0, start, None, 0.0))
        
        closed_set: Set[Tuple[float, float]] = set()
        g_costs: Dict[Tuple[float, float], float] = {start: 0.0}
        came_from: Dict[Tuple[float, float], Tuple[float, float]] = {}
        
        # 主循環
        while open_set:
            current_node = heapq.heappop(open_set)
            current_pos = current_node.position
            
            # 到達目標
            if current_pos == goal:
                return self._reconstruct_path(came_from, start, goal)
            
            # 已訪問過
            if current_pos in closed_set:
                continue
            
            closed_set.add(current_pos)
            
            # 擴展鄰居
            for neighbor in self.grid_map.get_neighbors(current_pos, self.use_diagonal):
                if neighbor in closed_set:
                    continue
                
                # 計算新的g值
                edge_cost = self._calculate_distance(current_pos, neighbor)
                tentative_g = g_costs[current_pos] + edge_cost
                
                # 如果找到更好的路徑
                if neighbor not in g_costs or tentative_g < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g
                    came_from[neighbor] = current_pos
                    
                    # 加入優先隊列
                    heapq.heappush(open_set, PriorityNode(tentative_g, neighbor, current_pos, tentative_g))
        
        # 未找到路徑
        return None
    
    def _calculate_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """計算兩點間的歐幾里得距離"""
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        return math.sqrt(dx**2 + dy**2)
    
    def _reconstruct_path(self, came_from: Dict[Tuple[float, float], Tuple[float, float]],
                         start: Tuple[float, float],
                         goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        """重建路徑"""
        path = [goal]
        current = goal
        
        while current != start:
            current = came_from[current]
            path.append(current)
        
        path.reverse()
        return path


class AStarPlanner(DijkstraPlanner):
    """A* 路徑規劃器（啟發式搜索）"""
    
    def __init__(self, 
                 grid_map: GridMap,
                 use_diagonal: bool = True,
                 heuristic_weight: float = 1.0):
        """
        初始化A*規劃器
        
        參數:
            grid_map: 柵格地圖
            use_diagonal: 是否使用對角線移動
            heuristic_weight: 啟發式權重（>1 更快但不一定最優，<1 更保守）
        """
        super().__init__(grid_map, use_diagonal)
        self.heuristic_weight = heuristic_weight
    
    def plan(self, 
             start: Tuple[float, float],
             goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        使用A*演算法規劃路徑
        
        參數:
            start: 起點座標
            goal: 終點座標
        
        返回:
            路徑點列表，如果找不到路徑則返回None
        """
        # 對齊到柵格
        start = self.grid_map.snap_to_grid(start)
        goal = self.grid_map.snap_to_grid(goal)
        
        # 檢查起點和終點是否有效
        if not self.grid_map.is_valid(start[0], start[1]):
            return None
        if not self.grid_map.is_valid(goal[0], goal[1]):
            return None
        
        # 初始化
        start_h = self._heuristic(start, goal)
        open_set = []
        heapq.heappush(open_set, PriorityNode(start_h, start, None, 0.0))
        
        closed_set: Set[Tuple[float, float]] = set()
        g_costs: Dict[Tuple[float, float], float] = {start: 0.0}
        came_from: Dict[Tuple[float, float], Tuple[float, float]] = {}
        
        # 主循環
        while open_set:
            current_node = heapq.heappop(open_set)
            current_pos = current_node.position
            
            # 到達目標
            if current_pos == goal:
                return self._reconstruct_path(came_from, start, goal)
            
            # 已訪問過
            if current_pos in closed_set:
                continue
            
            closed_set.add(current_pos)
            
            # 擴展鄰居
            for neighbor in self.grid_map.get_neighbors(current_pos, self.use_diagonal):
                if neighbor in closed_set:
                    continue
                
                # 計算新的g值
                edge_cost = self._calculate_distance(current_pos, neighbor)
                tentative_g = g_costs[current_pos] + edge_cost
                
                # 如果找到更好的路徑
                if neighbor not in g_costs or tentative_g < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g
                    came_from[neighbor] = current_pos
                    
                    # 計算f值 = g + h
                    h = self._heuristic(neighbor, goal)
                    f = tentative_g + self.heuristic_weight * h
                    
                    # 加入優先隊列
                    heapq.heappush(open_set, PriorityNode(f, neighbor, current_pos, tentative_g))
        
        # 未找到路徑
        return None
    
    def _heuristic(self, pos: Tuple[float, float], goal: Tuple[float, float]) -> float:
        """
        啟發式函數（估計到目標的距離）
        
        使用歐幾里得距離作為啟發式
        
        參數:
            pos: 當前位置
            goal: 目標位置
        
        返回:
            估計距離
        """
        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]
        return math.sqrt(dx**2 + dy**2)


def create_grid_from_polygon(polygon: List[Tuple[float, float]],
                             resolution: float,
                             collision_checker: CollisionChecker,
                             margin: float = 10.0) -> GridMap:
    """
    從多邊形創建柵格地圖
    
    參數:
        polygon: 多邊形頂點列表
        resolution: 柵格分辨率
        collision_checker: 碰撞檢測器
        margin: 邊界擴展距離
    
    返回:
        柵格地圖
    """
    # 計算邊界
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    
    min_x = min(xs) - margin
    max_x = max(xs) + margin
    min_y = min(ys) - margin
    max_y = max(ys) + margin
    
    bounds = (min_x, min_y, max_x, max_y)
    return GridMap(bounds, resolution, collision_checker)
