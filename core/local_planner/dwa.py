"""
DWA (Dynamic Window Approach) 局域路徑規劃器
參考 PythonRobotics 實現，針對多旋翼優化

核心原理：
1. 在速度空間中搜尋最優控制輸入
2. 考慮動力學約束（動態窗口）
3. 多目標代價函數評估軌跡品質
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Any
import time

from ..base.planner_base import (
    LocalPlanner, LocalPlannerConfig, PlannerType,
    PlannerResult, PlannerStatus, PlannerFactory
)
from ..base.vehicle_base import VehicleModel, VehicleState


@dataclass
class DWAConfig(LocalPlannerConfig):
    """DWA 規劃器配置"""
    # 速度採樣參數
    v_resolution: float = 0.1          # 線速度解析度 (m/s)
    w_resolution: float = 0.1          # 角速度解析度 (rad/s)
    
    # 預測參數
    predict_time: float = 3.0          # 軌跡預測時間 (s)
    
    # 代價函數權重
    heading_weight: float = 1.0        # 航向代價權重
    velocity_weight: float = 1.0       # 速度代價權重
    obstacle_weight: float = 1.0       # 障礙物代價權重
    goal_weight: float = 1.0           # 目標代價權重
    path_weight: float = 0.5           # 路徑跟蹤代價權重
    
    # 安全參數
    robot_radius: float = 0.5          # 機器人半徑 (m)
    obstacle_cost_gain: float = 1.0    # 障礙物代價增益
    
    # 目標追蹤
    goal_distance_threshold: float = 0.5  # 到達目標閾值 (m)
    waypoint_lookahead: int = 3           # 航點前瞻數量


@dataclass
class Obstacle:
    """障礙物資料類"""
    position: np.ndarray          # [x, y] 或 [x, y, z]
    radius: float = 1.0           # 障礙物半徑 (m)
    velocity: np.ndarray = None   # 障礙物速度（動態障礙物）
    
    def __post_init__(self):
        if self.velocity is None:
            self.velocity = np.zeros(2)


@PlannerFactory.register(PlannerType.DWA)
class DWAPlanner(LocalPlanner):
    """
    DWA 動態窗口局域規劃器
    
    特點：
    - 考慮飛行器動力學約束
    - 實時避障能力
    - 多目標代價函數優化
    - 支援動態障礙物
    
    使用方法:
        ```python
        config = DWAConfig(
            predict_time=3.0,
            heading_weight=1.0,
            obstacle_weight=2.0
        )
        dwa = DWAPlanner(config, vehicle_model)
        dwa.set_global_path(global_path)
        
        while not dwa.is_path_complete():
            v, w = dwa.compute_velocity(current_state, obstacles)
            # 執行控制
        ```
    """
    
    def __init__(self, config: DWAConfig = None,
                 vehicle_model: VehicleModel = None):
        super().__init__(config or DWAConfig())
        self.vehicle = vehicle_model
        
        # 內部狀態
        self._best_trajectory: List[np.ndarray] = []
        self._all_trajectories: List[List[np.ndarray]] = []
        self._current_goal: Optional[np.ndarray] = None
    
    @property
    def planner_type(self) -> PlannerType:
        return PlannerType.DWA
    
    def set_vehicle(self, vehicle: VehicleModel):
        """設置飛行器模型"""
        self.vehicle = vehicle
    
    def plan(self, start: np.ndarray, goal: np.ndarray,
             obstacles: List[Any] = None) -> PlannerResult:
        """
        規劃單次路徑（局域規劃器通常用 compute_velocity）
        
        這個方法主要用於單次目標規劃
        """
        self._status = PlannerStatus.PLANNING
        start_time = time.time()
        
        # 設置單點路徑
        self._global_path = [start, goal]
        self._current_waypoint_idx = 0
        self._current_goal = goal
        
        result = PlannerResult(
            status=PlannerStatus.SUCCESS,
            path=[start, goal],
            waypoints=[start, goal],
            planning_time=time.time() - start_time,
            message="DWA 局域規劃器已設置目標"
        )
        
        self._status = PlannerStatus.SUCCESS
        return result
    
    def compute_velocity(self, current_state: VehicleState,
                        obstacles: List[Obstacle] = None) -> Tuple[float, float]:
        """
        計算最優控制速度
        
        DWA 核心演算法：
        1. 計算動態窗口（可達速度空間）
        2. 對每個速度採樣預測軌跡
        3. 評估代價函數
        4. 選擇最優速度
        
        Args:
            current_state: 當前飛行器狀態
            obstacles: 障礙物列表
            
        Returns:
            (linear_velocity, angular_velocity) in (m/s, rad/s)
        """
        if self.vehicle is None:
            raise ValueError("需要先設置飛行器模型")
        
        # 更新飛行器狀態
        self.vehicle.state = current_state
        
        # 獲取當前目標
        self._update_current_goal(current_state)
        
        if self._current_goal is None:
            return (0.0, 0.0)
        
        # 檢查是否到達目標
        if self._is_goal_reached(current_state):
            self.advance_waypoint()
            self._update_current_goal(current_state)
            if self._current_goal is None:
                return (0.0, 0.0)
        
        # 獲取動態窗口
        config: DWAConfig = self.config
        dt = config.dt
        
        dynamic_window = self._calculate_dynamic_window(current_state, dt)
        
        # 速度採樣與評估
        best_velocity = (0.0, 0.0)
        min_cost = float('inf')
        self._all_trajectories.clear()
        
        # 轉換障礙物格式
        obstacle_list = self._convert_obstacles(obstacles)
        
        # 遍歷速度空間
        v_min, v_max, w_min, w_max = dynamic_window
        
        v_samples = np.arange(v_min, v_max + config.v_resolution, config.v_resolution)
        w_samples = np.arange(w_min, w_max + config.w_resolution, config.w_resolution)
        
        for v in v_samples:
            for w in w_samples:
                # 預測軌跡
                trajectory = self.vehicle.predict_trajectory(
                    (v, w), dt, config.predict_time
                )
                
                self._all_trajectories.append(trajectory)
                
                # 評估代價
                cost = self._evaluate_trajectory(
                    trajectory, (v, w), current_state,
                    self._current_goal, obstacle_list
                )
                
                if cost < min_cost:
                    min_cost = cost
                    best_velocity = (v, w)
                    self._best_trajectory = trajectory
        
        return best_velocity
    
    def _calculate_dynamic_window(self, state: VehicleState, 
                                  dt: float) -> Tuple[float, float, float, float]:
        """
        計算動態窗口
        
        考慮：
        1. 速度約束（最大/最小速度）
        2. 加速度約束（當前速度 ± 加速度 * dt）
        
        Returns:
            (v_min, v_max, w_min, w_max)
        """
        constraints = self.vehicle.constraints
        
        current_v = state.speed
        current_w = state.yaw_rate
        
        # 速度約束
        v_min_limit = constraints.min_speed
        v_max_limit = constraints.max_speed
        
        w_max_limit = np.radians(constraints.max_yaw_rate)
        
        # 加速度約束
        v_min_accel = current_v - constraints.max_deceleration * dt
        v_max_accel = current_v + constraints.max_acceleration * dt
        
        w_accel = np.radians(constraints.max_yaw_acceleration) * dt
        w_min_accel = current_w - w_accel
        w_max_accel = current_w + w_accel
        
        # 取交集
        v_min = max(v_min_limit, v_min_accel)
        v_max = min(v_max_limit, v_max_accel)
        w_min = max(-w_max_limit, w_min_accel)
        w_max = min(w_max_limit, w_max_accel)
        
        return (v_min, v_max, w_min, w_max)
    
    def _evaluate_trajectory(self, trajectory: List[np.ndarray],
                            velocity: Tuple[float, float],
                            current_state: VehicleState,
                            goal: np.ndarray,
                            obstacles: List[np.ndarray]) -> float:
        """
        評估軌跡代價
        
        代價函數組成：
        1. heading_cost: 與目標方向的偏差
        2. velocity_cost: 與最大速度的差距
        3. obstacle_cost: 與障礙物的接近程度
        4. goal_cost: 與目標的距離
        5. path_cost: 與全域路徑的偏離
        
        Args:
            trajectory: 預測軌跡
            velocity: 控制速度 (v, w)
            current_state: 當前狀態
            goal: 目標位置
            obstacles: 障礙物位置列表 [[x, y, radius], ...]
            
        Returns:
            總代價（越小越好）
        """
        config: DWAConfig = self.config
        
        if not trajectory:
            return float('inf')
        
        # 1. 航向代價 - 軌跡終點與目標的角度偏差
        heading_cost = self._calculate_heading_cost(trajectory, goal)
        
        # 2. 速度代價 - 鼓勵更快的速度
        v, w = velocity
        velocity_cost = self.vehicle.constraints.max_speed - v
        
        # 3. 障礙物代價 - 與障礙物的最小距離
        obstacle_cost = self._calculate_obstacle_cost(trajectory, obstacles)
        
        # 4. 目標代價 - 軌跡終點與目標的距離
        goal_cost = self._calculate_goal_cost(trajectory, goal)
        
        # 5. 路徑跟蹤代價 - 與全域路徑的偏離
        path_cost = self._calculate_path_cost(trajectory)
        
        # 加權總代價
        total_cost = (
            config.heading_weight * heading_cost +
            config.velocity_weight * velocity_cost +
            config.obstacle_weight * obstacle_cost +
            config.goal_weight * goal_cost +
            config.path_weight * path_cost
        )
        
        return total_cost
    
    def _calculate_heading_cost(self, trajectory: List[np.ndarray],
                               goal: np.ndarray) -> float:
        """計算航向代價"""
        if not trajectory:
            return float('inf')
        
        end_pos = trajectory[-1][:2]
        
        # 計算從軌跡終點到目標的方向
        to_goal = goal[:2] - end_pos
        target_heading = np.arctan2(to_goal[1], to_goal[0])
        
        # 計算當前軌跡的航向
        if len(trajectory) >= 2:
            direction = trajectory[-1][:2] - trajectory[-2][:2]
            current_heading = np.arctan2(direction[1], direction[0])
        else:
            current_heading = self.vehicle.state.heading
        
        # 角度差
        angle_diff = abs(target_heading - current_heading)
        angle_diff = min(angle_diff, 2 * np.pi - angle_diff)
        
        return angle_diff
    
    def _calculate_obstacle_cost(self, trajectory: List[np.ndarray],
                                obstacles: List[np.ndarray]) -> float:
        """
        計算障礙物代價
        
        使用反比例距離函數，距離越近代價越高
        """
        config: DWAConfig = self.config
        
        if not obstacles:
            return 0.0
        
        min_distance = float('inf')
        
        for pos in trajectory:
            for obs in obstacles:
                obs_pos = obs[:2]
                obs_radius = obs[2] if len(obs) > 2 else 0.5
                
                # 計算距離
                dist = np.linalg.norm(pos[:2] - obs_pos)
                effective_dist = dist - obs_radius - config.robot_radius
                
                if effective_dist < min_distance:
                    min_distance = effective_dist
                
                # 如果碰撞，返回無限大代價
                if effective_dist <= 0:
                    return float('inf')
        
        # 使用反比例函數
        if min_distance <= 0:
            return float('inf')
        
        return config.obstacle_cost_gain / min_distance
    
    def _calculate_goal_cost(self, trajectory: List[np.ndarray],
                            goal: np.ndarray) -> float:
        """計算目標代價"""
        if not trajectory:
            return float('inf')
        
        end_pos = trajectory[-1][:2]
        return np.linalg.norm(goal[:2] - end_pos)
    
    def _calculate_path_cost(self, trajectory: List[np.ndarray]) -> float:
        """
        計算路徑跟蹤代價
        
        測量軌跡與全域路徑的偏離程度
        """
        if not self._global_path or not trajectory:
            return 0.0
        
        total_deviation = 0.0
        
        for pos in trajectory:
            # 找到最近的全域路徑點
            min_dist = float('inf')
            for path_point in self._global_path:
                dist = np.linalg.norm(pos[:2] - path_point[:2])
                if dist < min_dist:
                    min_dist = dist
            
            total_deviation += min_dist
        
        return total_deviation / len(trajectory)
    
    def _update_current_goal(self, current_state: VehicleState):
        """更新當前目標點"""
        config: DWAConfig = self.config
        
        if not self._global_path:
            self._current_goal = None
            return
        
        # 前瞻選擇目標點
        lookahead_idx = min(
            self._current_waypoint_idx + config.waypoint_lookahead,
            len(self._global_path) - 1
        )
        
        self._current_goal = self._global_path[lookahead_idx]
    
    def _is_goal_reached(self, current_state: VehicleState) -> bool:
        """檢查是否到達當前目標"""
        if self._current_goal is None:
            return False
        
        config: DWAConfig = self.config
        distance = np.linalg.norm(
            current_state.position[:2] - self._current_goal[:2]
        )
        
        return distance < config.goal_distance_threshold
    
    def _convert_obstacles(self, obstacles: List[Any]) -> List[np.ndarray]:
        """轉換障礙物格式"""
        if not obstacles:
            return []
        
        result = []
        for obs in obstacles:
            if isinstance(obs, Obstacle):
                result.append(np.array([
                    obs.position[0], 
                    obs.position[1],
                    obs.radius
                ]))
            elif isinstance(obs, np.ndarray):
                result.append(obs)
            elif isinstance(obs, (list, tuple)):
                result.append(np.array(obs))
        
        return result
    
    def get_best_trajectory(self) -> List[np.ndarray]:
        """獲取當前最優軌跡（用於視覺化）"""
        return self._best_trajectory.copy()
    
    def get_all_trajectories(self) -> List[List[np.ndarray]]:
        """獲取所有評估過的軌跡（用於視覺化）"""
        return self._all_trajectories.copy()
    
    def get_current_goal(self) -> Optional[np.ndarray]:
        """獲取當前目標點"""
        return self._current_goal.copy() if self._current_goal is not None else None


class DWAVisualizer:
    """
    DWA 視覺化工具
    
    用於調試和展示 DWA 的運作過程
    """
    
    def __init__(self, dwa_planner: DWAPlanner):
        self.dwa = dwa_planner
    
    def get_visualization_data(self) -> dict:
        """
        獲取視覺化數據
        
        Returns:
            包含所有視覺化元素的字典
        """
        return {
            'best_trajectory': self.dwa.get_best_trajectory(),
            'all_trajectories': self.dwa.get_all_trajectories(),
            'current_goal': self.dwa.get_current_goal(),
            'global_path': self.dwa._global_path,
            'vehicle_state': self.dwa.vehicle.state if self.dwa.vehicle else None
        }
    
    def plot_matplotlib(self, ax=None):
        """
        使用 Matplotlib 繪製
        
        Args:
            ax: Matplotlib axes 對象
        """
        import matplotlib.pyplot as plt
        
        if ax is None:
            fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        
        data = self.get_visualization_data()
        
        # 繪製所有評估軌跡（灰色）
        for traj in data['all_trajectories']:
            if traj:
                points = np.array(traj)
                ax.plot(points[:, 0], points[:, 1], 
                       'gray', alpha=0.2, linewidth=0.5)
        
        # 繪製最優軌跡（綠色）
        best_traj = data['best_trajectory']
        if best_traj:
            points = np.array(best_traj)
            ax.plot(points[:, 0], points[:, 1], 
                   'g-', linewidth=2, label='Best Trajectory')
        
        # 繪製全域路徑（藍色虛線）
        global_path = data['global_path']
        if global_path:
            points = np.array(global_path)
            ax.plot(points[:, 0], points[:, 1], 
                   'b--', linewidth=1, label='Global Path')
        
        # 繪製當前目標（紅色星號）
        goal = data['current_goal']
        if goal is not None:
            ax.plot(goal[0], goal[1], 'r*', markersize=15, label='Current Goal')
        
        # 繪製飛行器位置（箭頭）
        state = data['vehicle_state']
        if state is not None:
            ax.arrow(state.position[0], state.position[1],
                    np.cos(state.heading) * 0.5,
                    np.sin(state.heading) * 0.5,
                    head_width=0.2, head_length=0.1, fc='blue', ec='blue')
        
        ax.set_aspect('equal')
        ax.legend()
        ax.grid(True)
        
        return ax
