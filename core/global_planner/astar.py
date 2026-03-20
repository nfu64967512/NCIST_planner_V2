"""
A* 路徑規劃算法
結合實際代價和啟發式估計，提供高效的最短路徑搜索
"""

import math
import heapq
from typing import List, Tuple, Optional, Set, Dict, Callable
from dataclasses import dataclass, field

from ..geometry import CoordinateTransform
from ..collision import CollisionChecker


@dataclass(order=True)
class AStarNode:
    """A* 節點"""
    f_cost: float  # f = g + h
    g_cost: float = field(compare=False)  # 實際代價
    h_cost: float = field(compare=False)  # 啟發式代價
    position: Tuple[float, float] = field(compare=False)
    parent: Optional['AStarNode'] = field(default=None, compare=False)
    
    def __hash__(self):
        return hash(self.position)


class HeuristicType:
    """啟發式函數類型"""
    
    @staticmethod
    def euclidean(pos1: Tuple[float, float], 
                  pos2: Tuple[float, float]) -> float:
        """歐幾里得距離"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    @staticmethod
    def manhattan(pos1: Tuple[float, float], 
                  pos2: Tuple[float, float]) -> float:
        """曼哈頓距離"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    @staticmethod
    def chebyshev(pos1: Tuple[float, float], 
                  pos2: Tuple[float, float]) -> float:
        """切比雪夫距離"""
        return max(abs(pos1[0] - pos2[0]), abs(pos1[1] - pos2[1]))
    
    @staticmethod
    def diagonal(pos1: Tuple[float, float], 
                pos2: Tuple[float, float]) -> float:
        """對角線距離（八方向移動）"""
        dx = abs(pos1[0] - pos2[0])
        dy = abs(pos1[1] - pos2[1])
        # sqrt(2) * min(dx, dy) + max(dx, dy) - min(dx, dy)
        return 1.414 * min(dx, dy) + abs(dx - dy)


class AStarPlanner:
    """
    A* 路徑規劃器
    
    特點:
    - 最優路徑保證（在可接受的啟發式下）
    - 支持多種啟發式函數
    - 可配置搜索範圍和步長
    - 支持動態障礙物
    """
    
    def __init__(self,
                 collision_checker: Optional[CollisionChecker] = None,
                 step_size: float = 5.0,
                 search_radius: float = 20.0,
                 heuristic: str = "euclidean",
                 heuristic_weight: float = 1.0):
        """
        初始化 A* 規劃器
        
        參數:
            collision_checker: 碰撞檢測器
            step_size: 搜索步長（公尺）
            search_radius: 鄰居搜索半徑（公尺）
            heuristic: 啟發式函數類型 ("euclidean", "manhattan", "chebyshev", "diagonal")
            heuristic_weight: 啟發式權重（>1 加速搜索但可能不是最優，<1 更謹慎）
        """
        self.collision_checker = collision_checker
        self.step_size = step_size
        self.search_radius = search_radius
        self.heuristic_weight = heuristic_weight
        
        # 設置啟發式函數
        heuristic_map = {
            "euclidean": HeuristicType.euclidean,
            "manhattan": HeuristicType.manhattan,
            "chebyshev": HeuristicType.chebyshev,
            "diagonal": HeuristicType.diagonal
        }
        self.heuristic_func = heuristic_map.get(heuristic, HeuristicType.euclidean)
        
        # 搜索方向（8方向）
        self.directions = [
            (1, 0), (-1, 0), (0, 1), (0, -1),  # 四方向
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # 對角線
        ]
    
    def plan(self,
             start: Tuple[float, float],
             goal: Tuple[float, float],
             boundary: Optional[List[Tuple[float, float]]] = None) -> List[Tuple[float, float]]:
        """
        執行 A* 路徑規劃
        
        參數:
            start: 起點（經緯度）
            goal: 終點（經緯度）
            boundary: 邊界多邊形（可選）
        
        返回:
            路徑點列表（經緯度）
        """
        # 座標轉換（經緯度 → 平面座標）
        center_lat = (start[0] + goal[0]) / 2
        center_lon = (start[1] + goal[1]) / 2
        coord_transform = CoordinateTransform(center_lat, center_lon)
        
        start_xy = coord_transform.latlon_to_xy(*start)
        goal_xy = coord_transform.latlon_to_xy(*goal)
        
        # 轉換邊界
        boundary_xy = None
        if boundary:
            boundary_xy = coord_transform.batch_latlon_to_xy(boundary)
        
        # 執行 A* 搜索
        path_xy = self._astar_search(start_xy, goal_xy, boundary_xy)
        
        if not path_xy:
            return []
        
        # 轉換回經緯度
        path = coord_transform.batch_xy_to_latlon(path_xy)
        return path
    
    def _astar_search(self,
                     start: Tuple[float, float],
                     goal: Tuple[float, float],
                     boundary: Optional[List[Tuple[float, float]]]) -> List[Tuple[float, float]]:
        """
        A* 搜索核心算法
        
        參數:
            start: 起點（平面座標）
            goal: 終點（平面座標）
            boundary: 邊界多邊形（平面座標）
        
        返回:
            路徑點列表（平面座標）
        """
        # 初始化
        open_set = []  # 優先隊列
        closed_set: Set[Tuple[float, float]] = set()
        g_scores: Dict[Tuple[float, float], float] = {start: 0.0}
        
        # 創建起始節點
        h_start = self.heuristic_func(start, goal) * self.heuristic_weight
        start_node = AStarNode(
            f_cost=h_start,
            g_cost=0.0,
            h_cost=h_start,
            position=start,
            parent=None
        )
        
        heapq.heappush(open_set, start_node)
        
        # 用於重建路徑的父節點映射
        came_from: Dict[Tuple[float, float], Tuple[float, float]] = {}
        
        max_iterations = 10000
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            # 取出 f 值最小的節點
            current_node = heapq.heappop(open_set)
            current_pos = current_node.position
            
            # 檢查是否到達目標
            if self._is_goal_reached(current_pos, goal):
                return self._reconstruct_path(came_from, current_pos, start)
            
            # 加入已探索集合
            closed_set.add(current_pos)
            
            # 探索鄰居
            neighbors = self._get_neighbors(current_pos, goal, boundary)
            
            for neighbor_pos in neighbors:
                # 跳過已探索的節點
                if neighbor_pos in closed_set:
                    continue
                
                # 計算新的 g 值
                tentative_g = current_node.g_cost + self._calculate_cost(
                    current_pos, neighbor_pos
                )
                
                # 如果找到更好的路徑，或者是新節點
                if neighbor_pos not in g_scores or tentative_g < g_scores[neighbor_pos]:
                    # 更新父節點
                    came_from[neighbor_pos] = current_pos
                    g_scores[neighbor_pos] = tentative_g
                    
                    # 計算 h 值和 f 值
                    h_cost = self.heuristic_func(neighbor_pos, goal) * self.heuristic_weight
                    f_cost = tentative_g + h_cost
                    
                    # 創建新節點並加入 open set
                    neighbor_node = AStarNode(
                        f_cost=f_cost,
                        g_cost=tentative_g,
                        h_cost=h_cost,
                        position=neighbor_pos,
                        parent=current_node
                    )
                    
                    heapq.heappush(open_set, neighbor_node)
        
        # 未找到路徑
        return []
    
    def _get_neighbors(self,
                      position: Tuple[float, float],
                      goal: Tuple[float, float],
                      boundary: Optional[List[Tuple[float, float]]]) -> List[Tuple[float, float]]:
        """
        獲取鄰居節點
        
        參數:
            position: 當前位置
            goal: 目標位置
            boundary: 邊界
        
        返回:
            有效鄰居列表
        """
        neighbors = []
        
        for dx, dy in self.directions:
            # 計算鄰居位置
            neighbor_x = position[0] + dx * self.step_size
            neighbor_y = position[1] + dy * self.step_size
            neighbor_pos = (neighbor_x, neighbor_y)
            
            # 檢查是否有效
            if self._is_valid_position(neighbor_pos, boundary):
                neighbors.append(neighbor_pos)
        
        # 添加朝向目標的直接鄰居（提高效率）
        direct_neighbor = self._get_direct_neighbor(position, goal)
        if direct_neighbor and self._is_valid_position(direct_neighbor, boundary):
            if direct_neighbor not in neighbors:
                neighbors.append(direct_neighbor)
        
        return neighbors
    
    def _get_direct_neighbor(self,
                           position: Tuple[float, float],
                           goal: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """
        獲取朝向目標的直接鄰居
        
        參數:
            position: 當前位置
            goal: 目標位置
        
        返回:
            直接鄰居位置
        """
        dx = goal[0] - position[0]
        dy = goal[1] - position[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 1e-6:
            return None
        
        # 單位方向向量
        ux = dx / distance
        uy = dy / distance
        
        # 沿方向移動一個步長
        neighbor_x = position[0] + ux * self.step_size
        neighbor_y = position[1] + uy * self.step_size
        
        return (neighbor_x, neighbor_y)
    
    def _is_valid_position(self,
                          position: Tuple[float, float],
                          boundary: Optional[List[Tuple[float, float]]]) -> bool:
        """
        檢查位置是否有效
        
        參數:
            position: 位置
            boundary: 邊界
        
        返回:
            是否有效
        """
        # 檢查邊界
        if boundary and not self._point_in_polygon(position, boundary):
            return False
        
        # 檢查碰撞
        if self.collision_checker:
            # 注意：collision_checker 可能需要經緯度座標
            # 這裡假設它可以處理平面座標，實際使用時可能需要轉換
            if self.collision_checker.check_point_collision(position):
                return False
        
        return True
    
    def _point_in_polygon(self,
                         point: Tuple[float, float],
                         polygon: List[Tuple[float, float]]) -> bool:
        """
        判斷點是否在多邊形內（射線法）
        
        參數:
            point: 點座標
            polygon: 多邊形頂點
        
        返回:
            是否在多邊形內
        """
        x, y = point
        n = len(polygon)
        inside = False
        
        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
    
    def _calculate_cost(self,
                       pos1: Tuple[float, float],
                       pos2: Tuple[float, float]) -> float:
        """
        計算兩點間的移動代價
        
        參數:
            pos1: 起點
            pos2: 終點
        
        返回:
            移動代價
        """
        # 歐幾里得距離
        distance = math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)
        
        # 可以根據需要添加額外的代價（如地形、轉向等）
        return distance
    
    def _is_goal_reached(self,
                        position: Tuple[float, float],
                        goal: Tuple[float, float],
                        tolerance: float = 1.0) -> bool:
        """
        檢查是否到達目標
        
        參數:
            position: 當前位置
            goal: 目標位置
            tolerance: 容差（公尺）
        
        返回:
            是否到達
        """
        distance = math.sqrt(
            (position[0] - goal[0])**2 + (position[1] - goal[1])**2
        )
        return distance <= tolerance
    
    def _reconstruct_path(self,
                         came_from: Dict[Tuple[float, float], Tuple[float, float]],
                         current: Tuple[float, float],
                         start: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        重建路徑
        
        參數:
            came_from: 父節點映射
            current: 當前節點
            start: 起點
        
        返回:
            完整路徑
        """
        path = [current]
        
        while current in came_from:
            current = came_from[current]
            path.append(current)
        
        path.reverse()
        return path
    
    def set_heuristic_weight(self, weight: float):
        """
        設置啟發式權重
        
        參數:
            weight: 權重值
                = 1.0: 標準 A*
                > 1.0: 加權 A*（更快但可能不是最優）
                < 1.0: 更保守（更接近 Dijkstra）
        """
        self.heuristic_weight = max(0.0, weight)
    
    def set_heuristic_function(self, heuristic_type: str):
        """
        設置啟發式函數
        
        參數:
            heuristic_type: 函數類型
        """
        heuristic_map = {
            "euclidean": HeuristicType.euclidean,
            "manhattan": HeuristicType.manhattan,
            "chebyshev": HeuristicType.chebyshev,
            "diagonal": HeuristicType.diagonal
        }
        
        if heuristic_type in heuristic_map:
            self.heuristic_func = heuristic_map[heuristic_type]


def compare_heuristics(start: Tuple[float, float],
                      goal: Tuple[float, float],
                      collision_checker: Optional[CollisionChecker] = None,
                      boundary: Optional[List[Tuple[float, float]]] = None) -> Dict[str, any]:
    """
    比較不同啟發式函數的性能
    
    參數:
        start: 起點
        goal: 終點
        collision_checker: 碰撞檢測器
        boundary: 邊界
    
    返回:
        比較結果字典
    """
    import time
    
    heuristics = ["euclidean", "manhattan", "chebyshev", "diagonal"]
    results = {}
    
    for h_type in heuristics:
        planner = AStarPlanner(
            collision_checker=collision_checker,
            heuristic=h_type
        )
        
        start_time = time.time()
        path = planner.plan(start, goal, boundary)
        elapsed_time = time.time() - start_time
        
        path_length = 0.0
        if len(path) > 1:
            for i in range(len(path) - 1):
                dx = path[i+1][0] - path[i][0]
                dy = path[i+1][1] - path[i][1]
                path_length += math.sqrt(dx**2 + dy**2)
        
        results[h_type] = {
            'time': elapsed_time,
            'path_length': path_length,
            'num_points': len(path)
        }
    
    return results
