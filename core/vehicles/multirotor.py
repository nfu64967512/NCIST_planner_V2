"""
多旋翼飛行器模型
實現多旋翼的運動學模型與約束
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

from core.base.vehicle_base import (
    VehicleModel, VehicleFactory, VehicleType, VehicleConfig,
    VehicleState, VehicleConstraints, DEFAULT_MULTIROTOR_CONFIG
)


@VehicleFactory.register(VehicleType.MULTIROTOR)
class MultirotorModel(VehicleModel):
    """
    多旋翼飛行器模型
    
    特點：
    - 全向移動能力
    - 可原地懸停
    - 無最小轉彎半徑限制
    - 適用於 DWA 局域規劃
    """
    
    def __init__(self, config: VehicleConfig = None):
        if config is None:
            config = DEFAULT_MULTIROTOR_CONFIG
        super().__init__(config)
        
        # DWA 速度採樣參數
        self.velocity_resolution = 0.1       # 線速度解析度 (m/s)
        self.yaw_rate_resolution = 5.0       # 角速度解析度 (deg/s)
        self.velocity_samples = 21           # 速度採樣數
        self.yaw_rate_samples = 21           # 角速度採樣數
    
    @property
    def vehicle_type(self) -> VehicleType:
        return VehicleType.MULTIROTOR
    
    def get_reachable_velocities(self, dt: float) -> List[Tuple[float, float]]:
        """
        獲取 DWA 可達速度空間
        
        多旋翼的動態窗口考慮：
        1. 當前速度 ± 加速度 * dt
        2. 約束限制（最大/最小速度）
        
        Returns:
            List of (linear_velocity, yaw_rate) in (m/s, rad/s)
        """
        c = self.constraints
        current_v = self.state.speed
        current_w = self.state.yaw_rate
        
        # 計算動態窗口邊界
        # 線速度窗口
        v_min = max(c.min_speed, current_v - c.max_deceleration * dt)
        v_max = min(c.max_speed, current_v + c.max_acceleration * dt)
        
        # 角速度窗口 (轉換為 rad/s)
        max_yaw_rate_rad = np.radians(c.max_yaw_rate)
        max_yaw_accel_rad = np.radians(c.max_yaw_acceleration)
        
        w_min = max(-max_yaw_rate_rad, current_w - max_yaw_accel_rad * dt)
        w_max = min(max_yaw_rate_rad, current_w + max_yaw_accel_rad * dt)
        
        # 生成速度採樣
        velocities = []
        v_step = (v_max - v_min) / (self.velocity_samples - 1) if self.velocity_samples > 1 else 0
        w_step = (w_max - w_min) / (self.yaw_rate_samples - 1) if self.yaw_rate_samples > 1 else 0
        
        for i in range(self.velocity_samples):
            v = v_min + i * v_step if v_step > 0 else v_min
            for j in range(self.yaw_rate_samples):
                w = w_min + j * w_step if w_step > 0 else w_min
                velocities.append((v, w))
        
        return velocities
    
    def predict_trajectory(self, velocity: Tuple[float, float], 
                          dt: float, horizon: float) -> List[np.ndarray]:
        """
        預測軌跡（用於 DWA 代價評估）
        
        使用簡化的運動學模型：
        x' = x + v * cos(θ) * dt
        y' = y + v * sin(θ) * dt
        θ' = θ + ω * dt
        
        Args:
            velocity: (linear_velocity, angular_velocity) in (m/s, rad/s)
            dt: 時間步長 (s)
            horizon: 預測時間範圍 (s)
            
        Returns:
            預測的位置列表 [[x, y, z], ...]
        """
        v, w = velocity
        trajectory = []
        
        # 複製當前狀態
        x, y, z = self.state.position
        theta = self.state.heading
        
        steps = int(horizon / dt)
        for _ in range(steps):
            # 運動學更新
            x += v * np.cos(theta) * dt
            y += v * np.sin(theta) * dt
            theta += w * dt
            
            # 正規化角度
            theta = self._normalize_angle(theta)
            
            trajectory.append(np.array([x, y, z]))
        
        return trajectory
    
    def compute_motion(self, velocity: Tuple[float, float], 
                      dt: float) -> VehicleState:
        """
        計算運動模型，返回新狀態
        
        Args:
            velocity: (linear_velocity, angular_velocity) in (m/s, rad/s)
            dt: 時間步長
            
        Returns:
            新的飛行器狀態
        """
        v, w = velocity
        
        x, y, z = self.state.position
        theta = self.state.heading
        
        # 運動學更新
        if abs(w) < 1e-6:
            # 直線運動
            new_x = x + v * np.cos(theta) * dt
            new_y = y + v * np.sin(theta) * dt
        else:
            # 弧線運動
            new_x = x + (v / w) * (np.sin(theta + w * dt) - np.sin(theta))
            new_y = y + (v / w) * (-np.cos(theta + w * dt) + np.cos(theta))
        
        new_theta = self._normalize_angle(theta + w * dt)
        
        return VehicleState(
            position=np.array([new_x, new_y, z]),
            velocity=np.array([v * np.cos(new_theta), v * np.sin(new_theta), 0]),
            heading=new_theta,
            yaw_rate=w,
            timestamp=self.state.timestamp + dt
        )
    
    def is_feasible_path(self, start: np.ndarray, end: np.ndarray) -> bool:
        """
        檢查路徑是否可行
        
        多旋翼的全向移動能力使得大多數路徑都是可行的
        主要檢查高度約束
        
        Args:
            start: 起點位置 [x, y, z]
            end: 終點位置 [x, y, z]
            
        Returns:
            路徑是否可行
        """
        c = self.constraints
        
        # 檢查高度約束
        if len(start) > 2 and len(end) > 2:
            if start[2] < c.min_altitude or start[2] > c.max_altitude:
                return False
            if end[2] < c.min_altitude or end[2] > c.max_altitude:
                return False
            
            # 檢查爬升/下降率
            dist_2d = np.linalg.norm(end[:2] - start[:2])
            height_diff = abs(end[2] - start[2])
            
            if dist_2d > 0:
                # 計算所需的垂直速度
                horizontal_time = dist_2d / c.max_speed
                required_vertical_speed = height_diff / horizontal_time
                if required_vertical_speed > c.max_vertical_speed:
                    return False
        
        return True
    
    def compute_turn_waypoints(self, p1: np.ndarray, p2: np.ndarray, 
                               p3: np.ndarray) -> List[np.ndarray]:
        """
        計算轉彎航點
        
        多旋翼可以直接轉向，不需要額外的轉彎航點
        但為了平滑飛行，可以添加減速點
        
        Args:
            p1: 進入點
            p2: 轉折點
            p3: 離開點
            
        Returns:
            轉彎航點列表（多旋翼通常只返回轉折點）
        """
        # 計算轉向角度
        v1 = p2[:2] - p1[:2]
        v2 = p3[:2] - p2[:2]
        
        angle = self._angle_between_vectors(v1, v2)
        
        # 大角度轉彎時添加減速點
        if abs(angle) > np.radians(60):
            # 在轉折點前後添加減速緩衝
            decel_dist = 2.0  # 減速距離 (m)
            
            d1 = np.linalg.norm(v1)
            d2 = np.linalg.norm(v2)
            
            points = []
            
            # 進入減速點
            if d1 > decel_dist:
                ratio = (d1 - decel_dist) / d1
                decel_point = p1 + ratio * (p2 - p1)
                points.append(decel_point)
            
            # 轉折點
            points.append(p2.copy())
            
            # 離開加速點
            if d2 > decel_dist:
                ratio = decel_dist / d2
                accel_point = p2 + ratio * (p3 - p2)
                points.append(accel_point)
            
            return points
        
        return [p2.copy()]
    
    def compute_hover_position(self, waypoint: np.ndarray, 
                               hover_time: float) -> List[np.ndarray]:
        """
        計算懸停位置（多旋翼專用）
        
        Args:
            waypoint: 懸停位置
            hover_time: 懸停時間 (s)
            
        Returns:
            懸停航點列表
        """
        return [waypoint.copy()]
    
    def get_optimal_survey_speed(self, camera_trigger_interval: float,
                                 photo_overlap: float = 0.8) -> float:
        """
        計算最佳測繪速度
        
        根據相機觸發間隔和重疊率計算最佳飛行速度
        
        Args:
            camera_trigger_interval: 相機最小觸發間隔 (s)
            photo_overlap: 照片重疊率
            
        Returns:
            建議飛行速度 (m/s)
        """
        # 這是一個簡化計算，實際需要結合相機 FOV
        # 假設相機地面覆蓋範圍已知
        ground_coverage = 50.0  # 假設地面覆蓋 50m（需要從 CameraModel 獲取）
        
        # 非重疊距離
        non_overlap_dist = ground_coverage * (1 - photo_overlap)
        
        # 速度 = 距離 / 時間
        speed = non_overlap_dist / camera_trigger_interval
        
        # 限制在約束範圍內
        return min(speed, self.constraints.max_speed * 0.8)
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """正規化角度到 [-π, π]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    @staticmethod
    def _angle_between_vectors(v1: np.ndarray, v2: np.ndarray) -> float:
        """計算兩向量之間的夾角"""
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        
        cos_angle = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
        angle = np.arccos(cos_angle)
        
        # 計算有向角度（使用叉積判斷方向）
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        if cross < 0:
            angle = -angle
        
        return angle


@dataclass
class MultirotorDynamics:
    """
    多旋翼動力學參數
    用於更精確的軌跡預測和控制
    """
    # 質量與慣性
    mass: float = 2.0                    # 質量 (kg)
    inertia_xx: float = 0.02             # 繞 x 軸轉動慣量
    inertia_yy: float = 0.02             # 繞 y 軸轉動慣量
    inertia_zz: float = 0.04             # 繞 z 軸轉動慣量
    
    # 氣動參數
    drag_coefficient_xy: float = 0.1     # 水平阻力係數
    drag_coefficient_z: float = 0.15     # 垂直阻力係數
    
    # 推力參數
    thrust_coefficient: float = 1.5e-5   # 推力係數
    torque_coefficient: float = 1.2e-6   # 扭矩係數
    max_motor_speed: float = 1000.0      # 最大馬達轉速 (rad/s)
    
    # 響應時間
    attitude_time_constant: float = 0.1  # 姿態響應時間常數
    position_time_constant: float = 0.5  # 位置響應時間常數


class AdvancedMultirotorModel(MultirotorModel):
    """
    進階多旋翼模型
    
    包含更精確的動力學模型，用於高精度軌跡規劃
    """
    
    def __init__(self, config: VehicleConfig = None,
                 dynamics: MultirotorDynamics = None):
        super().__init__(config)
        self.dynamics = dynamics or MultirotorDynamics()
    
    def compute_motion_with_dynamics(self, thrust: float, 
                                     attitude_rate: np.ndarray,
                                     dt: float) -> VehicleState:
        """
        使用完整動力學模型計算運動
        
        Args:
            thrust: 總推力 (N)
            attitude_rate: 姿態角速度 [roll_rate, pitch_rate, yaw_rate]
            dt: 時間步長
            
        Returns:
            新的飛行器狀態
        """
        d = self.dynamics
        
        # 當前狀態
        pos = self.state.position.copy()
        vel = self.state.velocity.copy()
        heading = self.state.heading
        
        # 計算加速度
        # 簡化：假設姿態穩定，主要考慮推力和阻力
        gravity = np.array([0, 0, -9.81])
        
        # 推力方向（假設向上）
        thrust_vec = np.array([0, 0, thrust / d.mass])
        
        # 阻力
        drag = -np.array([
            d.drag_coefficient_xy * vel[0] * abs(vel[0]),
            d.drag_coefficient_xy * vel[1] * abs(vel[1]),
            d.drag_coefficient_z * vel[2] * abs(vel[2])
        ]) / d.mass
        
        # 總加速度
        accel = gravity + thrust_vec + drag
        
        # 更新速度和位置
        new_vel = vel + accel * dt
        new_pos = pos + vel * dt + 0.5 * accel * dt ** 2
        
        # 更新航向
        new_heading = self._normalize_angle(heading + attitude_rate[2] * dt)
        
        return VehicleState(
            position=new_pos,
            velocity=new_vel,
            heading=new_heading,
            yaw_rate=attitude_rate[2],
            timestamp=self.state.timestamp + dt
        )
