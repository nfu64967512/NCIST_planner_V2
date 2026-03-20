"""
固定翼飛行器模型
提供固定翼無人機的運動學模型和動力學約束
"""

import math
from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np


@dataclass
class FixedWingConstraints:
    """固定翼飛行器約束參數"""
    
    # 速度約束
    min_speed: float = 10.0          # 最小速度（失速速度）[m/s]
    max_speed: float = 30.0          # 最大速度 [m/s]
    cruise_speed: float = 18.0       # 巡航速度 [m/s]
    
    # 轉彎約束
    max_bank_angle: float = 45.0     # 最大滾轉角 [度]
    min_turn_radius: float = 50.0    # 最小轉彎半徑 [m]
    
    # 爬升約束
    max_climb_rate: float = 5.0      # 最大爬升率 [m/s]
    max_descent_rate: float = 3.0    # 最大下降率 [m/s]
    max_climb_angle: float = 20.0    # 最大爬升角 [度]
    
    # 加速度約束
    max_acceleration: float = 2.0    # 最大加速度 [m/s²]
    
    # 高度約束
    min_altitude: float = 30.0       # 最小飛行高度 [m]
    max_altitude: float = 500.0      # 最大飛行高度 [m]
    
    def __post_init__(self):
        """驗證約束參數的有效性"""
        if self.min_speed >= self.max_speed:
            raise ValueError("最小速度必須小於最大速度")
        
        if self.cruise_speed < self.min_speed or self.cruise_speed > self.max_speed:
            raise ValueError("巡航速度必須在min_speed和max_speed之間")
        
        # 根據速度和滾轉角計算最小轉彎半徑
        g = 9.81  # 重力加速度
        bank_rad = math.radians(self.max_bank_angle)
        calculated_radius = self.cruise_speed ** 2 / (g * math.tan(bank_rad))
        
        if self.min_turn_radius < calculated_radius:
            self.min_turn_radius = calculated_radius
    
    def get_turn_radius(self, speed: float, bank_angle: float) -> float:
        """
        計算給定速度和滾轉角的轉彎半徑
        
        參數:
            speed: 飛行速度 [m/s]
            bank_angle: 滾轉角 [度]
        
        返回:
            轉彎半徑 [m]
        """
        g = 9.81
        bank_rad = math.radians(min(abs(bank_angle), self.max_bank_angle))
        
        if bank_rad < 1e-6:
            return float('inf')
        
        return speed ** 2 / (g * math.tan(bank_rad))
    
    def get_max_turn_rate(self, speed: float) -> float:
        """
        計算給定速度的最大轉彎率
        
        參數:
            speed: 飛行速度 [m/s]
        
        返回:
            最大轉彎率 [度/秒]
        """
        g = 9.81
        bank_rad = math.radians(self.max_bank_angle)
        
        turn_rate_rad = g * math.tan(bank_rad) / speed
        return math.degrees(turn_rate_rad)


@dataclass
class FixedWingState:
    """固定翼飛行器狀態"""
    
    # 位置（經度、緯度、高度）
    x: float = 0.0          # 東向位置 [m] 或 經度 [度]
    y: float = 0.0          # 北向位置 [m] 或 緯度 [度]
    z: float = 100.0        # 高度 [m]
    
    # 速度
    vx: float = 15.0        # x方向速度 [m/s]
    vy: float = 0.0         # y方向速度 [m/s]
    vz: float = 0.0         # z方向速度（爬升率）[m/s]
    
    # 姿態
    heading: float = 0.0    # 航向角（北為0，順時針為正）[度]
    pitch: float = 0.0      # 俯仰角 [度]
    roll: float = 0.0       # 滾轉角 [度]
    
    # 時間
    time: float = 0.0       # 時間戳 [s]
    
    @property
    def speed(self) -> float:
        """水平速度"""
        return math.sqrt(self.vx ** 2 + self.vy ** 2)
    
    @property
    def ground_speed(self) -> float:
        """地速"""
        return math.sqrt(self.vx ** 2 + self.vy ** 2 + self.vz ** 2)
    
    @property
    def climb_angle(self) -> float:
        """爬升角（度）"""
        if self.speed > 0:
            return math.degrees(math.atan2(self.vz, self.speed))
        return 0.0
    
    def to_tuple(self) -> Tuple[float, ...]:
        """轉換為元組（用於優化算法）"""
        return (self.x, self.y, self.z, self.vx, self.vy, self.vz, 
                self.heading, self.pitch, self.roll, self.time)
    
    @classmethod
    def from_tuple(cls, state_tuple: Tuple[float, ...]) -> 'FixedWingState':
        """從元組創建狀態"""
        return cls(*state_tuple)


class FixedWingModel:
    """固定翼飛行器運動學模型"""
    
    def __init__(self, constraints: Optional[FixedWingConstraints] = None):
        """
        初始化固定翼模型
        
        參數:
            constraints: 飛行器約束參數
        """
        self.constraints = constraints or FixedWingConstraints()
    
    def update(self, state: FixedWingState, 
              control_speed: float,
              control_heading_rate: float,
              control_climb_rate: float,
              dt: float) -> FixedWingState:
        """
        更新飛行器狀態（離散時間步進）
        
        參數:
            state: 當前狀態
            control_speed: 期望速度 [m/s]
            control_heading_rate: 期望轉向率 [度/s]
            control_climb_rate: 期望爬升率 [m/s]
            dt: 時間步長 [s]
        
        返回:
            新狀態
        """
        # 限制控制輸入
        speed = np.clip(control_speed, 
                       self.constraints.min_speed, 
                       self.constraints.max_speed)
        
        max_turn_rate = self.constraints.get_max_turn_rate(speed)
        heading_rate = np.clip(control_heading_rate, 
                              -max_turn_rate, 
                              max_turn_rate)
        
        climb_rate = np.clip(control_climb_rate,
                            -self.constraints.max_descent_rate,
                            self.constraints.max_climb_rate)
        
        # 更新航向
        new_heading = (state.heading + heading_rate * dt) % 360
        
        # 計算速度分量
        heading_rad = math.radians(new_heading)
        new_vx = speed * math.sin(heading_rad)
        new_vy = speed * math.cos(heading_rad)
        new_vz = climb_rate
        
        # 更新位置
        new_x = state.x + state.vx * dt
        new_y = state.y + state.vy * dt
        new_z = state.z + state.vz * dt
        
        # 限制高度
        new_z = np.clip(new_z, 
                       self.constraints.min_altitude,
                       self.constraints.max_altitude)
        
        # 計算滾轉角（從轉向率推算）
        if speed > 0:
            g = 9.81
            required_bank = math.degrees(math.atan(heading_rate * speed / g))
            new_roll = np.clip(required_bank,
                             -self.constraints.max_bank_angle,
                             self.constraints.max_bank_angle)
        else:
            new_roll = 0.0
        
        # 計算俯仰角（從爬升率推算）
        if speed > 0:
            new_pitch = math.degrees(math.atan2(climb_rate, speed))
        else:
            new_pitch = 0.0
        
        return FixedWingState(
            x=new_x,
            y=new_y,
            z=new_z,
            vx=new_vx,
            vy=new_vy,
            vz=new_vz,
            heading=new_heading,
            pitch=new_pitch,
            roll=new_roll,
            time=state.time + dt
        )
    
    def can_reach(self, current_state: FixedWingState,
                 target_point: Tuple[float, float, float],
                 time_limit: float) -> bool:
        """
        檢查是否可以在時間限制內到達目標點
        
        參數:
            current_state: 當前狀態
            target_point: 目標點 (x, y, z)
            time_limit: 時間限制 [s]
        
        返回:
            是否可達
        """
        # 計算直線距離
        dx = target_point[0] - current_state.x
        dy = target_point[1] - current_state.y
        dz = target_point[2] - current_state.z
        
        horizontal_distance = math.sqrt(dx ** 2 + dy ** 2)
        
        # 計算需要的轉彎半徑
        target_heading = math.degrees(math.atan2(dx, dy)) % 360
        heading_diff = abs(target_heading - current_state.heading)
        if heading_diff > 180:
            heading_diff = 360 - heading_diff
        
        # 估算轉彎距離
        turn_radius = self.constraints.min_turn_radius
        turn_distance = turn_radius * math.radians(heading_diff)
        
        # 總水平距離
        total_horizontal = horizontal_distance + turn_distance
        
        # 估算高度變化所需時間
        if dz > 0:
            climb_time = abs(dz) / self.constraints.max_climb_rate
        else:
            climb_time = abs(dz) / self.constraints.max_descent_rate
        
        # 估算水平移動時間
        horizontal_time = total_horizontal / self.constraints.cruise_speed
        
        # 總時間（取較大值，因為爬升和水平移動可以同時進行）
        required_time = max(climb_time, horizontal_time)
        
        return required_time <= time_limit
    
    def compute_dubins_path(self, start_state: FixedWingState,
                           goal_state: FixedWingState) -> Optional[list]:
        """
        計算Dubins路徑（適用於固定翼）
        
        Dubins路徑由三段組成：{L,R}{S,L,R}{L,R}
        L=左轉, R=右轉, S=直線
        
        參數:
            start_state: 起始狀態
            goal_state: 目標狀態
        
        返回:
            路徑點列表或None
        """
        # 簡化實現：返回LSL路徑（左轉-直線-左轉）
        # 完整實現需要比較所有6種Dubins路徑類型
        
        # 提取位置和航向
        x1, y1 = start_state.x, start_state.y
        theta1 = math.radians(start_state.heading)
        
        x2, y2 = goal_state.x, goal_state.y
        theta2 = math.radians(goal_state.heading)
        
        r = self.constraints.min_turn_radius
        
        # 計算圓心
        cx1 = x1 - r * math.cos(theta1)
        cy1 = y1 + r * math.sin(theta1)
        
        cx2 = x2 - r * math.cos(theta2)
        cy2 = y2 + r * math.sin(theta2)
        
        # 計算切線
        d = math.sqrt((cx2 - cx1) ** 2 + (cy2 - cy1) ** 2)
        
        if d < 2 * r:
            # 圓相交，無LSL路徑
            return None
        
        # 外切線角度
        alpha = math.atan2(cy2 - cy1, cx2 - cx1)
        beta = math.acos(2 * r / d)
        
        # 第一段圓弧的結束點
        t1 = alpha + beta
        x_tangent1 = cx1 + r * math.cos(t1)
        y_tangent1 = cy1 + r * math.sin(t1)
        
        # 第二段圓弧的起始點
        t2 = alpha + beta
        x_tangent2 = cx2 + r * math.cos(t2)
        y_tangent2 = cy2 + r * math.sin(t2)
        
        # 生成路徑點（簡化版）
        path_points = []
        
        # 第一段圓弧
        angle_start = theta1
        angle_end = t1 + math.pi / 2
        num_points = 10
        
        for i in range(num_points + 1):
            t = i / num_points
            angle = angle_start + t * (angle_end - angle_start)
            x = cx1 + r * math.sin(angle)
            y = cy1 - r * math.cos(angle)
            path_points.append((x, y, start_state.z))
        
        # 直線段
        path_points.append((x_tangent2, y_tangent2, start_state.z))
        
        # 第二段圓弧
        angle_start = t2 + math.pi / 2
        angle_end = theta2
        
        for i in range(num_points + 1):
            t = i / num_points
            angle = angle_start + t * (angle_end - angle_start)
            x = cx2 + r * math.sin(angle)
            y = cy2 - r * math.cos(angle)
            path_points.append((x, y, goal_state.z))
        
        return path_points
    
    def is_state_valid(self, state: FixedWingState) -> bool:
        """
        檢查狀態是否有效
        
        參數:
            state: 飛行器狀態
        
        返回:
            狀態是否有效
        """
        # 檢查速度
        if not (self.constraints.min_speed <= state.speed <= self.constraints.max_speed):
            return False
        
        # 檢查高度
        if not (self.constraints.min_altitude <= state.z <= self.constraints.max_altitude):
            return False
        
        # 檢查爬升角
        if abs(state.climb_angle) > self.constraints.max_climb_angle:
            return False
        
        # 檢查滾轉角
        if abs(state.roll) > self.constraints.max_bank_angle:
            return False
        
        return True
