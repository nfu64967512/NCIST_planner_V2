"""
人工勢場法（Artificial Potential Field, APF）局部路徑規劃器
實時避障和局部路徑調整
"""

import math
from typing import List, Tuple, Optional, Callable
from dataclasses import dataclass
import numpy as np


@dataclass
class APFConfig:
    """APF 配置參數"""
    attractive_gain: float = 1.0      # 引力增益
    repulsive_gain: float = 50.0      # 斥力增益
    repulsive_range: float = 10.0     # 斥力影響範圍
    step_size: float = 0.5            # 步進大小
    max_force: float = 100.0          # 最大合力
    goal_tolerance: float = 0.5       # 到達目標的容差
    max_iterations: int = 1000        # 最大迭代次數
    local_minimum_threshold: float = 0.01  # 局部最小值判斷閾值
    local_minimum_escape_force: float = 5.0  # 逃離局部最小值的力


class APFLocalPlanner:
    """
    人工勢場法局部路徑規劃器
    
    特點：
    - 實時性高，計算簡單
    - 適用於動態環境
    - 可能陷入局部最小值
    """
    
    def __init__(self,
                 config: Optional[APFConfig] = None,
                 collision_check_fn: Optional[Callable[[Tuple[float, float]], bool]] = None):
        """
        初始化 APF 規劃器
        
        參數:
            config: APF 配置參數
            collision_check_fn: 碰撞檢測函數（可選）
        """
        self.config = config or APFConfig()
        self.collision_check_fn = collision_check_fn
        
        # 障礙物列表
        self.obstacles: List[Tuple[float, float]] = []
        
        # 路徑記錄
        self.path_history: List[Tuple[float, float]] = []
    
    def set_obstacles(self, obstacles: List[Tuple[float, float]]):
        """
        設置障礙物列表
        
        參數:
            obstacles: 障礙物中心點列表
        """
        self.obstacles = obstacles
    
    def plan(self,
            start: Tuple[float, float],
            goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        執行 APF 路徑規劃
        
        參數:
            start: 起點
            goal: 終點
        
        返回:
            路徑點列表，如果失敗則返回 None
        """
        current_pos = np.array(start)
        goal_pos = np.array(goal)
        
        path = [tuple(current_pos)]
        self.path_history = []
        
        stuck_counter = 0
        prev_distance = float('inf')
        
        for iteration in range(self.config.max_iterations):
            # 計算合力
            attractive_force = self._calculate_attractive_force(current_pos, goal_pos)
            repulsive_force = self._calculate_repulsive_force(current_pos)
            
            total_force = attractive_force + repulsive_force
            
            # 限制最大力
            force_magnitude = np.linalg.norm(total_force)
            if force_magnitude > self.config.max_force:
                total_force = total_force / force_magnitude * self.config.max_force
            
            # 檢測局部最小值
            current_distance = np.linalg.norm(current_pos - goal_pos)
            
            if force_magnitude < self.config.local_minimum_threshold:
                stuck_counter += 1
                
                # 嘗試逃離局部最小值
                if stuck_counter > 10:
                    print(f"檢測到局部最小值（迭代 {iteration}），嘗試逃離")
                    escape_force = self._generate_escape_force(current_pos, goal_pos)
                    total_force += escape_force
                    stuck_counter = 0
            else:
                stuck_counter = 0
            
            # 更新位置
            if force_magnitude > 1e-6:
                direction = total_force / max(force_magnitude, 1e-6)
                new_pos = current_pos + direction * self.config.step_size
            else:
                # 力太小，嘗試直接向目標移動
                direction = goal_pos - current_pos
                distance = np.linalg.norm(direction)
                if distance > 1e-6:
                    direction = direction / distance
                    new_pos = current_pos + direction * self.config.step_size
                else:
                    break
            
            # 檢查碰撞（如果有檢測函數）
            if self.collision_check_fn is not None:
                if self.collision_check_fn(tuple(new_pos)):
                    # 發生碰撞，嘗試繞過
                    print(f"檢測到碰撞（迭代 {iteration}），調整路徑")
                    # 嘗試垂直方向
                    perpendicular = np.array([-direction[1], direction[0]])
                    new_pos = current_pos + perpendicular * self.config.step_size
                    
                    if self.collision_check_fn(tuple(new_pos)):
                        # 還是碰撞，嘗試反向
                        new_pos = current_pos - perpendicular * self.config.step_size
            
            current_pos = new_pos
            path.append(tuple(current_pos))
            self.path_history.append(tuple(current_pos))
            
            # 檢查是否到達目標
            if current_distance < self.config.goal_tolerance:
                print(f"到達目標！迭代次數: {iteration + 1}")
                return path
            
            # 檢查是否停止前進
            if abs(current_distance - prev_distance) < 1e-4:
                stuck_counter += 1
                if stuck_counter > 50:
                    print(f"無法前進（迭代 {iteration}）")
                    return None
            
            prev_distance = current_distance
        
        print(f"達到最大迭代次數，未到達目標")
        return path if len(path) > 1 else None
    
    def _calculate_attractive_force(self,
                                   current_pos: np.ndarray,
                                   goal_pos: np.ndarray) -> np.ndarray:
        """
        計算引力（指向目標）
        
        參數:
            current_pos: 當前位置
            goal_pos: 目標位置
        
        返回:
            引力向量
        """
        # 引力: F_att = k_att * (goal - current)
        direction = goal_pos - current_pos
        distance = np.linalg.norm(direction)
        
        if distance < 1e-6:
            return np.zeros(2)
        
        # 歸一化方向向量
        direction_normalized = direction / distance
        
        # 線性引力模型
        force = self.config.attractive_gain * direction_normalized * distance
        
        return force
    
    def _calculate_repulsive_force(self,
                                  current_pos: np.ndarray) -> np.ndarray:
        """
        計算斥力（遠離障礙物）
        
        參數:
            current_pos: 當前位置
        
        返回:
            斥力向量
        """
        if not self.obstacles:
            return np.zeros(2)
        
        total_repulsive_force = np.zeros(2)
        
        for obstacle in self.obstacles:
            obs_pos = np.array(obstacle)
            
            # 計算到障礙物的距離和方向
            diff = current_pos - obs_pos
            distance = np.linalg.norm(diff)
            
            # 只有在影響範圍內才產生斥力
            if distance < self.config.repulsive_range and distance > 1e-6:
                # 斥力模型: F_rep = k_rep * (1/d - 1/d0) * (1/d^2) * direction
                direction_normalized = diff / distance
                
                repulsive_magnitude = self.config.repulsive_gain * \
                    (1.0 / distance - 1.0 / self.config.repulsive_range) * \
                    (1.0 / (distance ** 2))
                
                repulsive_force = repulsive_magnitude * direction_normalized
                total_repulsive_force += repulsive_force
        
        return total_repulsive_force
    
    def _generate_escape_force(self,
                              current_pos: np.ndarray,
                              goal_pos: np.ndarray) -> np.ndarray:
        """
        生成逃離局部最小值的力
        
        參數:
            current_pos: 當前位置
            goal_pos: 目標位置
        
        返回:
            逃離力向量
        """
        # 生成一個隨機方向，但偏向目標
        random_angle = np.random.uniform(-np.pi, np.pi)
        random_direction = np.array([np.cos(random_angle), np.sin(random_angle)])
        
        # 計算指向目標的方向
        to_goal = goal_pos - current_pos
        to_goal_normalized = to_goal / (np.linalg.norm(to_goal) + 1e-6)
        
        # 混合隨機方向和目標方向
        escape_direction = 0.3 * random_direction + 0.7 * to_goal_normalized
        escape_direction = escape_direction / (np.linalg.norm(escape_direction) + 1e-6)
        
        return escape_direction * self.config.local_minimum_escape_force
    
    def calculate_force_at_point(self,
                                point: Tuple[float, float],
                                goal: Tuple[float, float]) -> Tuple[float, float]:
        """
        計算指定點的合力（用於可視化）
        
        參數:
            point: 查詢點
            goal: 目標點
        
        返回:
            合力向量 (fx, fy)
        """
        current_pos = np.array(point)
        goal_pos = np.array(goal)
        
        attractive_force = self._calculate_attractive_force(current_pos, goal_pos)
        repulsive_force = self._calculate_repulsive_force(current_pos)
        
        total_force = attractive_force + repulsive_force
        
        return (total_force[0], total_force[1])


# ==========================================
# 進階 APF - 改進版
# ==========================================
class ImprovedAPFPlanner(APFLocalPlanner):
    """
    改進的 APF 規劃器
    
    改進：
    1. 虛擬目標法：在障礙物附近設置虛擬目標
    2. 切線法：沿著障礙物邊緣移動
    3. 歷史記憶：避免重複訪問相同位置
    """
    
    def __init__(self,
                 config: Optional[APFConfig] = None,
                 collision_check_fn: Optional[Callable[[Tuple[float, float]], bool]] = None):
        super().__init__(config, collision_check_fn)
        
        # 訪問歷史（用於避免重複）
        self.visited_positions: List[Tuple[float, float]] = []
        self.visit_penalty_range = 2.0  # 訪問懲罰範圍
    
    def _calculate_repulsive_force(self,
                                  current_pos: np.ndarray) -> np.ndarray:
        """
        改進的斥力計算（加入切線分量）
        
        參數:
            current_pos: 當前位置
        
        返回:
            斥力向量
        """
        if not self.obstacles:
            return np.zeros(2)
        
        total_repulsive_force = np.zeros(2)
        
        for obstacle in self.obstacles:
            obs_pos = np.array(obstacle)
            
            diff = current_pos - obs_pos
            distance = np.linalg.norm(diff)
            
            if distance < self.config.repulsive_range and distance > 1e-6:
                direction_normalized = diff / distance
                
                # 徑向斥力
                radial_magnitude = self.config.repulsive_gain * \
                    (1.0 / distance - 1.0 / self.config.repulsive_range) * \
                    (1.0 / (distance ** 2))
                
                radial_force = radial_magnitude * direction_normalized
                
                # 切向力（沿著障礙物邊緣）
                tangent_direction = np.array([-direction_normalized[1], direction_normalized[0]])
                tangent_magnitude = radial_magnitude * 0.3  # 切向力較小
                
                tangent_force = tangent_magnitude * tangent_direction
                
                total_repulsive_force += (radial_force + tangent_force)
        
        # 加入訪問歷史懲罰
        history_penalty = self._calculate_history_penalty(current_pos)
        total_repulsive_force += history_penalty
        
        return total_repulsive_force
    
    def _calculate_history_penalty(self, current_pos: np.ndarray) -> np.ndarray:
        """
        計算訪問歷史懲罰力
        
        參數:
            current_pos: 當前位置
        
        返回:
            懲罰力向量
        """
        penalty_force = np.zeros(2)
        
        for visited_pos in self.visited_positions[-50:]:  # 只考慮最近50個位置
            visited = np.array(visited_pos)
            diff = current_pos - visited
            distance = np.linalg.norm(diff)
            
            if distance < self.visit_penalty_range and distance > 1e-6:
                direction_normalized = diff / distance
                magnitude = 2.0 * (1.0 - distance / self.visit_penalty_range)
                penalty_force += magnitude * direction_normalized
        
        return penalty_force
