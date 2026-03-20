"""
MPC (Model Predictive Control) 模型預測控制局部路徑規劃器
基於優化的路徑跟踪和避障控制
"""

import math
from typing import List, Tuple, Optional, Callable
from dataclasses import dataclass
import numpy as np
from scipy.optimize import minimize


@dataclass
class MPCConfig:
    """MPC 配置參數"""
    # 預測參數
    horizon: int = 10                # 預測地平線（步數）
    dt: float = 0.1                  # 時間步長 [s]
    
    # 狀態約束
    max_speed: float = 1.0           # 最大速度 [m/s]
    max_yaw_rate: float = 45.0       # 最大角速度 [度/s]
    max_accel: float = 0.5           # 最大加速度 [m/s²]
    
    # 成本函數權重
    position_weight: float = 10.0    # 位置誤差權重
    heading_weight: float = 1.0      # 航向誤差權重
    velocity_weight: float = 0.1     # 速度變化權重
    control_weight: float = 0.1      # 控制輸入權重
    obstacle_weight: float = 100.0   # 障礙物懲罰權重
    
    # 障礙物參數
    robot_radius: float = 0.5        # 機器人半徑 [m]
    safe_distance: float = 0.5       # 安全距離 [m]


@dataclass
class MPCState:
    """MPC 狀態"""
    x: float = 0.0                   # x 位置 [m]
    y: float = 0.0                   # y 位置 [m]
    yaw: float = 0.0                 # 航向角 [度]
    v: float = 0.0                   # 速度 [m/s]


class MPCPlanner:
    """
    模型預測控制（MPC）局部路徑規劃器
    
    特點：
    - 考慮多步預測
    - 可處理約束
    - 適用於路徑跟踪
    """
    
    def __init__(self,
                 config: Optional[MPCConfig] = None):
        """
        初始化 MPC 規劃器
        
        參數:
            config: MPC 配置參數
        """
        self.config = config or MPCConfig()
        
        # 參考路徑
        self.reference_path: List[Tuple[float, float]] = []
        
        # 障礙物列表
        self.obstacles: List[Tuple[float, float]] = []
    
    def set_reference_path(self, path: List[Tuple[float, float]]):
        """
        設置參考路徑
        
        參數:
            path: 參考路徑點列表
        """
        self.reference_path = path
    
    def set_obstacles(self, obstacles: List[Tuple[float, float]]):
        """
        設置障礙物列表
        
        參數:
            obstacles: 障礙物中心點列表
        """
        self.obstacles = obstacles
    
    def plan_control(self,
                    current_state: MPCState) -> Tuple[float, float]:
        """
        規劃控制輸入
        
        參數:
            current_state: 當前狀態
        
        返回:
            最優控制輸入 (v, omega)
        """
        if not self.reference_path:
            return (0.0, 0.0)
        
        # 找到參考路徑上最近的點
        ref_index = self._find_nearest_point_index(current_state)
        
        # 提取預測地平線內的參考點
        ref_trajectory = self._get_reference_trajectory(ref_index)
        
        # 設置優化初始值（上一次的控制輸入）
        initial_guess = np.zeros(self.config.horizon * 2)
        initial_guess[::2] = current_state.v  # 速度
        initial_guess[1::2] = 0.0  # 角速度
        
        # 設置約束
        bounds = []
        for _ in range(self.config.horizon):
            bounds.append((0.0, self.config.max_speed))  # 速度約束
            bounds.append((-self.config.max_yaw_rate, self.config.max_yaw_rate))  # 角速度約束
        
        # 優化
        result = minimize(
            fun=self._cost_function,
            x0=initial_guess,
            args=(current_state, ref_trajectory),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 100, 'ftol': 1e-4}
        )
        
        if result.success:
            # 返回第一步的控制輸入
            optimal_v = result.x[0]
            optimal_omega = result.x[1]
            return (optimal_v, optimal_omega)
        else:
            # 優化失敗，返回保守控制
            return (0.0, 0.0)
    
    def _find_nearest_point_index(self, state: MPCState) -> int:
        """
        找到參考路徑上最近的點索引
        
        參數:
            state: 當前狀態
        
        返回:
            最近點索引
        """
        min_distance = float('inf')
        nearest_index = 0
        
        for i, point in enumerate(self.reference_path):
            distance = math.sqrt(
                (point[0] - state.x) ** 2 +
                (point[1] - state.y) ** 2
            )
            
            if distance < min_distance:
                min_distance = distance
                nearest_index = i
        
        return nearest_index
    
    def _get_reference_trajectory(self, start_index: int) -> List[Tuple[float, float]]:
        """
        提取預測地平線內的參考軌跡
        
        參數:
            start_index: 起始索引
        
        返回:
            參考軌跡點列表
        """
        ref_trajectory = []
        
        for i in range(self.config.horizon):
            index = min(start_index + i, len(self.reference_path) - 1)
            ref_trajectory.append(self.reference_path[index])
        
        return ref_trajectory
    
    def _cost_function(self,
                      control_sequence: np.ndarray,
                      current_state: MPCState,
                      ref_trajectory: List[Tuple[float, float]]) -> float:
        """
        成本函數
        
        參數:
            control_sequence: 控制序列 [v1, omega1, v2, omega2, ...]
            current_state: 初始狀態
            ref_trajectory: 參考軌跡
        
        返回:
            總成本
        """
        # 解析控制序列
        v_seq = control_sequence[::2]
        omega_seq = control_sequence[1::2]
        
        # 預測狀態軌跡
        predicted_states = self._predict_states(current_state, v_seq, omega_seq)
        
        # 計算各項成本
        position_cost = 0.0
        heading_cost = 0.0
        control_cost = 0.0
        obstacle_cost = 0.0
        
        for i, state in enumerate(predicted_states):
            # 位置誤差
            if i < len(ref_trajectory):
                ref_x, ref_y = ref_trajectory[i]
                position_error = math.sqrt(
                    (state.x - ref_x) ** 2 +
                    (state.y - ref_y) ** 2
                )
                position_cost += position_error ** 2
                
                # 航向誤差
                desired_yaw = math.degrees(math.atan2(ref_y - state.y, ref_x - state.x))
                heading_error = abs(desired_yaw - state.yaw)
                if heading_error > 180:
                    heading_error = 360 - heading_error
                heading_cost += heading_error ** 2
            
            # 控制輸入成本
            if i < len(v_seq):
                control_cost += v_seq[i] ** 2 + (omega_seq[i] / 100.0) ** 2
            
            # 障礙物懲罰
            for obstacle in self.obstacles:
                obs_x, obs_y = obstacle
                distance_to_obs = math.sqrt(
                    (state.x - obs_x) ** 2 +
                    (state.y - obs_y) ** 2
                )
                
                safe_threshold = self.config.robot_radius + self.config.safe_distance
                
                if distance_to_obs < safe_threshold:
                    # 距離越近，懲罰越大
                    obstacle_cost += (safe_threshold - distance_to_obs) ** 2 * 10.0
        
        # 計算速度變化成本（平滑控制）
        velocity_cost = 0.0
        for i in range(1, len(v_seq)):
            velocity_cost += (v_seq[i] - v_seq[i-1]) ** 2
        
        # 綜合成本
        total_cost = (
            self.config.position_weight * position_cost +
            self.config.heading_weight * heading_cost +
            self.config.velocity_weight * velocity_cost +
            self.config.control_weight * control_cost +
            self.config.obstacle_weight * obstacle_cost
        )
        
        return total_cost
    
    def _predict_states(self,
                       initial_state: MPCState,
                       v_seq: np.ndarray,
                       omega_seq: np.ndarray) -> List[MPCState]:
        """
        預測未來狀態序列
        
        參數:
            initial_state: 初始狀態
            v_seq: 速度序列
            omega_seq: 角速度序列
        
        返回:
            預測狀態列表
        """
        states = []
        
        # 複製初始狀態
        state = MPCState(
            x=initial_state.x,
            y=initial_state.y,
            yaw=initial_state.yaw,
            v=initial_state.v
        )
        
        # 逐步預測
        for i in range(len(v_seq)):
            # 更新狀態（運動學模型）
            yaw_rad = math.radians(state.yaw)
            state.x += v_seq[i] * math.cos(yaw_rad) * self.config.dt
            state.y += v_seq[i] * math.sin(yaw_rad) * self.config.dt
            state.yaw += omega_seq[i] * self.config.dt
            state.yaw = state.yaw % 360
            state.v = v_seq[i]
            
            # 保存狀態
            states.append(MPCState(
                x=state.x,
                y=state.y,
                yaw=state.yaw,
                v=state.v
            ))
        
        return states
    
    def plan_path(self,
                 start_state: MPCState,
                 max_steps: int = 500,
                 goal_tolerance: float = 0.5) -> Optional[List[Tuple[float, float]]]:
        """
        規劃完整路徑
        
        參數:
            start_state: 起始狀態
            max_steps: 最大步數
            goal_tolerance: 到達目標的容差
        
        返回:
            路徑點列表，如果失敗則返回 None
        """
        if not self.reference_path:
            print("未設置參考路徑")
            return None
        
        current_state = MPCState(
            x=start_state.x,
            y=start_state.y,
            yaw=start_state.yaw,
            v=start_state.v
        )
        
        path = [(current_state.x, current_state.y)]
        goal = self.reference_path[-1]
        
        for step in range(max_steps):
            # 檢查是否到達目標
            distance_to_goal = math.sqrt(
                (goal[0] - current_state.x) ** 2 +
                (goal[1] - current_state.y) ** 2
            )
            
            if distance_to_goal < goal_tolerance:
                print(f"到達目標！步數: {step + 1}")
                return path
            
            # 規劃控制輸入
            v, omega = self.plan_control(current_state)
            
            # 更新狀態
            yaw_rad = math.radians(current_state.yaw)
            current_state.x += v * math.cos(yaw_rad) * self.config.dt
            current_state.y += v * math.sin(yaw_rad) * self.config.dt
            current_state.yaw += omega * self.config.dt
            current_state.yaw = current_state.yaw % 360
            current_state.v = v
            
            path.append((current_state.x, current_state.y))
        
        print(f"達到最大步數 {max_steps}，未到達目標")
        return path if len(path) > 1 else None
