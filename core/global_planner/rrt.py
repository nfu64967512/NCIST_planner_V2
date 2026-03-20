"""
RRT (Rapidly-exploring Random Tree) 路徑規劃器
實現RRT和RRT*演算法，用於全局路徑規劃
"""

import math
import random
from typing import List, Tuple, Optional, Callable
from dataclasses import dataclass

from ..collision import CollisionChecker


@dataclass
class RRTNode:
    """RRT樹節點"""
    x: float
    y: float
    parent: Optional['RRTNode'] = None
    cost: float = 0.0  # 從根節點到此節點的累積成本（用於RRT*）
    
    def distance_to(self, other: 'RRTNode') -> float:
        """計算到另一個節點的距離"""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def to_tuple(self) -> Tuple[float, float]:
        """轉換為元組"""
        return (self.x, self.y)


class RRTPlanner:
    """RRT路徑規劃器"""
    
    def __init__(self, 
                 collision_checker: CollisionChecker,
                 step_size: float = 5.0,
                 goal_sample_rate: float = 0.05,
                 max_iter: int = 500):
        """
        初始化RRT規劃器
        
        參數:
            collision_checker: 碰撞檢測器
            step_size: 每步擴展距離（公尺）
            goal_sample_rate: 目標採樣率（0-1）
            max_iter: 最大迭代次數
        """
        self.collision_checker = collision_checker
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        
        self.nodes: List[RRTNode] = []
        self.start: Optional[RRTNode] = None
        self.goal: Optional[RRTNode] = None
    
    def plan(self, 
             start: Tuple[float, float],
             goal: Tuple[float, float],
             search_area: Tuple[float, float, float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        規劃從起點到終點的路徑
        
        參數:
            start: 起點座標 (x, y)
            goal: 終點座標 (x, y)
            search_area: 搜索區域 (min_x, min_y, max_x, max_y)
        
        返回:
            路徑點列表，如果找不到路徑則返回None
        """
        # 初始化
        self.start = RRTNode(start[0], start[1])
        self.goal = RRTNode(goal[0], goal[1])
        self.nodes = [self.start]
        
        min_x, min_y, max_x, max_y = search_area
        
        # 主循環
        for i in range(self.max_iter):
            # 採樣隨機點
            if random.random() < self.goal_sample_rate:
                sample = self.goal
            else:
                sample = self._get_random_node(min_x, min_y, max_x, max_y)
            
            # 找到最近的節點
            nearest = self._get_nearest_node(sample)
            
            # 向採樣點擴展
            new_node = self._steer(nearest, sample)
            
            # 檢查碰撞
            if not self._check_collision(nearest, new_node):
                self.nodes.append(new_node)
                
                # 檢查是否到達目標
                if new_node.distance_to(self.goal) <= self.step_size:
                    final_node = self._steer(new_node, self.goal)
                    if not self._check_collision(new_node, final_node):
                        self.nodes.append(final_node)
                        return self._generate_final_path(final_node)
        
        # 未找到路徑
        return None
    
    def _get_random_node(self, min_x: float, min_y: float, 
                        max_x: float, max_y: float) -> RRTNode:
        """生成隨機節點"""
        x = random.uniform(min_x, max_x)
        y = random.uniform(min_y, max_y)
        return RRTNode(x, y)
    
    def _get_nearest_node(self, sample: RRTNode) -> RRTNode:
        """找到距離採樣點最近的節點"""
        distances = [node.distance_to(sample) for node in self.nodes]
        nearest_idx = distances.index(min(distances))
        return self.nodes[nearest_idx]
    
    def _steer(self, from_node: RRTNode, to_node: RRTNode) -> RRTNode:
        """
        從from_node向to_node方向擴展step_size距離
        
        參數:
            from_node: 起始節點
            to_node: 目標節點
        
        返回:
            新節點
        """
        distance = from_node.distance_to(to_node)
        
        if distance <= self.step_size:
            new_node = RRTNode(to_node.x, to_node.y, from_node)
        else:
            # 計算方向
            theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)
            new_node = RRTNode(new_x, new_y, from_node)
        
        new_node.cost = from_node.cost + from_node.distance_to(new_node)
        return new_node
    
    def _check_collision(self, from_node: RRTNode, to_node: RRTNode) -> bool:
        """
        檢查兩節點之間的路徑是否有碰撞
        
        返回:
            True表示有碰撞，False表示無碰撞
        """
        return self.collision_checker.check_segment_collision(
            from_node.to_tuple(), 
            to_node.to_tuple()
        )
    
    def _generate_final_path(self, goal_node: RRTNode) -> List[Tuple[float, float]]:
        """
        從目標節點回溯生成最終路徑
        
        參數:
            goal_node: 目標節點
        
        返回:
            路徑點列表（從起點到終點）
        """
        path = []
        node = goal_node
        
        while node is not None:
            path.append(node.to_tuple())
            node = node.parent
        
        # 反轉路徑（從起點到終點）
        path.reverse()
        return path


class RRTStarPlanner(RRTPlanner):
    """RRT* 路徑規劃器（優化版RRT）"""
    
    def __init__(self,
                 collision_checker: CollisionChecker,
                 step_size: float = 5.0,
                 goal_sample_rate: float = 0.05,
                 max_iter: int = 500,
                 search_radius: float = 20.0):
        """
        初始化RRT*規劃器
        
        參數:
            collision_checker: 碰撞檢測器
            step_size: 每步擴展距離（公尺）
            goal_sample_rate: 目標採樣率（0-1）
            max_iter: 最大迭代次數
            search_radius: 重新佈線搜索半徑（公尺）
        """
        super().__init__(collision_checker, step_size, goal_sample_rate, max_iter)
        self.search_radius = search_radius
    
    def plan(self, 
             start: Tuple[float, float],
             goal: Tuple[float, float],
             search_area: Tuple[float, float, float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        規劃從起點到終點的路徑（RRT*優化版本）
        
        參數:
            start: 起點座標 (x, y)
            goal: 終點座標 (x, y)
            search_area: 搜索區域 (min_x, min_y, max_x, max_y)
        
        返回:
            路徑點列表，如果找不到路徑則返回None
        """
        # 初始化
        self.start = RRTNode(start[0], start[1])
        self.goal = RRTNode(goal[0], goal[1])
        self.nodes = [self.start]
        
        min_x, min_y, max_x, max_y = search_area
        
        # 主循環
        for i in range(self.max_iter):
            # 採樣隨機點
            if random.random() < self.goal_sample_rate:
                sample = self.goal
            else:
                sample = self._get_random_node(min_x, min_y, max_x, max_y)
            
            # 找到最近的節點
            nearest = self._get_nearest_node(sample)
            
            # 向採樣點擴展
            new_node = self._steer(nearest, sample)
            
            # 檢查碰撞
            if not self._check_collision(nearest, new_node):
                # RRT*: 在附近尋找更好的父節點
                near_nodes = self._find_near_nodes(new_node)
                new_node = self._choose_parent(new_node, near_nodes)
                
                self.nodes.append(new_node)
                
                # RRT*: 重新佈線
                self._rewire(new_node, near_nodes)
                
                # 檢查是否到達目標
                if new_node.distance_to(self.goal) <= self.step_size:
                    final_node = self._steer(new_node, self.goal)
                    if not self._check_collision(new_node, final_node):
                        self.nodes.append(final_node)
                        return self._generate_final_path(final_node)
        
        # 未找到路徑
        return None
    
    def _find_near_nodes(self, node: RRTNode) -> List[RRTNode]:
        """
        找到節點附近的所有節點
        
        參數:
            node: 目標節點
        
        返回:
            附近節點列表
        """
        near_nodes = []
        for n in self.nodes:
            if n.distance_to(node) <= self.search_radius:
                near_nodes.append(n)
        return near_nodes
    
    def _choose_parent(self, node: RRTNode, near_nodes: List[RRTNode]) -> RRTNode:
        """
        為新節點選擇最佳父節點
        
        參數:
            node: 新節點
            near_nodes: 附近節點列表
        
        返回:
            更新後的節點
        """
        if not near_nodes:
            return node
        
        # 計算通過每個附近節點到達新節點的成本
        costs = []
        for near_node in near_nodes:
            if not self._check_collision(near_node, node):
                cost = near_node.cost + near_node.distance_to(node)
                costs.append((cost, near_node))
        
        if not costs:
            return node
        
        # 選擇成本最小的父節點
        min_cost, best_parent = min(costs, key=lambda x: x[0])
        node.parent = best_parent
        node.cost = min_cost
        
        return node
    
    def _rewire(self, new_node: RRTNode, near_nodes: List[RRTNode]):
        """
        重新佈線：檢查是否可以通過新節點改善附近節點的路徑
        
        參數:
            new_node: 新加入的節點
            near_nodes: 附近節點列表
        """
        for near_node in near_nodes:
            # 跳過新節點的父節點
            if near_node == new_node.parent:
                continue
            
            # 計算通過新節點到達該節點的成本
            new_cost = new_node.cost + new_node.distance_to(near_node)
            
            # 如果成本更低且無碰撞，則更新父節點
            if new_cost < near_node.cost:
                if not self._check_collision(new_node, near_node):
                    near_node.parent = new_node
                    near_node.cost = new_cost
    
    def get_path_cost(self, path: List[Tuple[float, float]]) -> float:
        """
        計算路徑總成本（長度）
        
        參數:
            path: 路徑點列表
        
        返回:
            路徑總長度
        """
        if len(path) < 2:
            return 0.0
        
        cost = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            cost += math.sqrt(dx**2 + dy**2)
        
        return cost
