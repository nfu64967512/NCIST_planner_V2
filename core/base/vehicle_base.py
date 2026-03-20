"""
飛行器基類模組
定義所有飛行器類型的通用介面和約束
支援多旋翼與固定翼的策略模式切換
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Tuple, Optional, Dict, Any
import numpy as np


class VehicleType(Enum):
    """飛行器類型枚舉"""
    MULTIROTOR = auto()    # 多旋翼
    FIXED_WING = auto()    # 固定翼
    VTOL = auto()          # 垂直起降固定翼


class FlightMode(Enum):
    """飛行模式枚舉"""
    SURVEY = auto()        # 測繪模式
    WAYPOINT = auto()      # 航點模式
    LOITER = auto()        # 盤旋模式
    RTL = auto()           # 返航模式


@dataclass
class VehicleState:
    """飛行器狀態資料類"""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [x, y, z] 公尺
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [vx, vy, vz] m/s
    heading: float = 0.0           # 航向角 (rad)
    yaw_rate: float = 0.0          # 轉向速率 (rad/s)
    timestamp: float = 0.0         # 時間戳
    
    @property
    def speed(self) -> float:
        """獲取當前速度大小"""
        return float(np.linalg.norm(self.velocity[:2]))
    
    @property
    def position_2d(self) -> np.ndarray:
        """獲取 2D 位置"""
        return self.position[:2]


@dataclass
class VehicleConstraints:
    """飛行器動力學約束"""
    # 速度約束
    max_speed: float = 15.0          # 最大速度 (m/s)
    min_speed: float = 0.0           # 最小速度 (m/s) - 固定翼需設定失速速度
    max_vertical_speed: float = 5.0  # 最大垂直速度 (m/s)
    
    # 加速度約束
    max_acceleration: float = 3.0    # 最大加速度 (m/s²)
    max_deceleration: float = 4.0    # 最大減速度 (m/s²)
    
    # 轉向約束
    max_yaw_rate: float = 90.0       # 最大轉向速率 (deg/s)
    max_yaw_acceleration: float = 45.0  # 最大轉向加速度 (deg/s²)
    min_turn_radius: float = 0.0     # 最小轉彎半徑 (m) - 固定翼專用
    
    # 高度約束
    min_altitude: float = 5.0        # 最低飛行高度 (m)
    max_altitude: float = 120.0      # 最高飛行高度 (m)
    
    # 安全約束
    safety_margin: float = 2.0       # 安全邊距 (m)
    collision_radius: float = 1.5    # 碰撞半徑 (m)
    
    def validate(self) -> bool:
        """驗證約束有效性"""
        if self.max_speed <= self.min_speed:
            raise ValueError("最大速度必須大於最小速度")
        if self.max_acceleration <= 0:
            raise ValueError("最大加速度必須大於0")
        if self.min_turn_radius < 0:
            raise ValueError("最小轉彎半徑不能為負")
        return True


@dataclass
class VehicleConfig:
    """飛行器配置"""
    name: str = "Default UAV"
    vehicle_type: VehicleType = VehicleType.MULTIROTOR
    constraints: VehicleConstraints = field(default_factory=VehicleConstraints)
    
    # 物理參數
    mass: float = 2.0                # 質量 (kg)
    max_thrust: float = 40.0         # 最大推力 (N)
    drag_coefficient: float = 0.1    # 阻力係數
    
    # 尺寸參數
    arm_length: float = 0.25         # 軸距/臂長 (m)
    propeller_radius: float = 0.12   # 螺旋槳半徑 (m)
    
    # 電池參數
    battery_capacity: float = 5000   # 電池容量 (mAh)
    flight_time_estimate: float = 25 # 預估飛行時間 (min)


class VehicleModel(ABC):
    """
    飛行器模型抽象基類
    
    定義所有飛行器類型必須實現的介面
    使用策略模式支援不同飛行器類型的切換
    """
    
    def __init__(self, config: VehicleConfig):
        self.config = config
        self.state = VehicleState()
        self._trajectory_history: List[VehicleState] = []
    
    @property
    @abstractmethod
    def vehicle_type(self) -> VehicleType:
        """獲取飛行器類型"""
        pass
    
    @property
    def constraints(self) -> VehicleConstraints:
        """獲取約束"""
        return self.config.constraints
    
    @abstractmethod
    def get_reachable_velocities(self, dt: float) -> List[Tuple[float, float]]:
        """
        獲取可達速度空間（用於 DWA）
        
        Args:
            dt: 時間步長
            
        Returns:
            List of (linear_velocity, angular_velocity) pairs
        """
        pass
    
    @abstractmethod
    def predict_trajectory(self, velocity: Tuple[float, float], 
                          dt: float, horizon: float) -> List[np.ndarray]:
        """
        預測軌跡（用於 DWA）
        
        Args:
            velocity: (linear_velocity, angular_velocity)
            dt: 時間步長
            horizon: 預測時間範圍
            
        Returns:
            預測的位置列表
        """
        pass
    
    @abstractmethod
    def compute_motion(self, velocity: Tuple[float, float], 
                      dt: float) -> VehicleState:
        """
        計算運動模型
        
        Args:
            velocity: 控制輸入 (linear_velocity, angular_velocity)
            dt: 時間步長
            
        Returns:
            新的飛行器狀態
        """
        pass
    
    @abstractmethod
    def is_feasible_path(self, start: np.ndarray, end: np.ndarray) -> bool:
        """
        檢查路徑是否可行（考慮動力學約束）
        
        Args:
            start: 起點位置
            end: 終點位置
            
        Returns:
            路徑是否可行
        """
        pass
    
    @abstractmethod
    def compute_turn_waypoints(self, p1: np.ndarray, p2: np.ndarray, 
                               p3: np.ndarray) -> List[np.ndarray]:
        """
        計算轉彎航點（多旋翼可能直接轉向，固定翼需要弧線）
        
        Args:
            p1: 進入點
            p2: 轉折點
            p3: 離開點
            
        Returns:
            轉彎過程中的航點列表
        """
        pass
    
    def update_state(self, new_state: VehicleState):
        """更新飛行器狀態"""
        self._trajectory_history.append(self.state)
        self.state = new_state
    
    def reset_state(self, position: np.ndarray = None, heading: float = 0.0):
        """重設飛行器狀態"""
        self.state = VehicleState(
            position=position if position is not None else np.zeros(3),
            heading=heading
        )
        self._trajectory_history.clear()
    
    def get_trajectory_history(self) -> List[VehicleState]:
        """獲取軌跡歷史"""
        return self._trajectory_history.copy()
    
    def estimate_travel_time(self, distance: float, 
                            include_acceleration: bool = True) -> float:
        """
        估算行駛時間
        
        Args:
            distance: 距離 (m)
            include_acceleration: 是否包含加減速時間
            
        Returns:
            估算時間 (s)
        """
        c = self.constraints
        cruise_speed = c.max_speed * 0.8  # 巡航速度取最大速度的 80%
        
        if not include_acceleration:
            return distance / cruise_speed
        
        # 計算加速和減速距離
        accel_time = cruise_speed / c.max_acceleration
        decel_time = cruise_speed / c.max_deceleration
        accel_dist = 0.5 * c.max_acceleration * accel_time ** 2
        decel_dist = 0.5 * c.max_deceleration * decel_time ** 2
        
        if distance < accel_dist + decel_dist:
            # 短距離：無法達到巡航速度
            # 簡化計算：假設對稱加減速
            return 2 * np.sqrt(distance / c.max_acceleration)
        else:
            # 長距離：加速 + 巡航 + 減速
            cruise_dist = distance - accel_dist - decel_dist
            cruise_time = cruise_dist / cruise_speed
            return accel_time + cruise_time + decel_time


class VehicleFactory:
    """飛行器工廠類"""
    
    _registry: Dict[VehicleType, type] = {}
    
    @classmethod
    def register(cls, vehicle_type: VehicleType):
        """註冊飛行器類型的裝飾器"""
        def decorator(vehicle_class: type):
            cls._registry[vehicle_type] = vehicle_class
            return vehicle_class
        return decorator
    
    @classmethod
    def create(cls, config: VehicleConfig) -> VehicleModel:
        """
        創建飛行器實例
        
        Args:
            config: 飛行器配置
            
        Returns:
            飛行器模型實例
        """
        vehicle_class = cls._registry.get(config.vehicle_type)
        if vehicle_class is None:
            raise ValueError(f"未註冊的飛行器類型: {config.vehicle_type}")
        return vehicle_class(config)
    
    @classmethod
    def get_available_types(cls) -> List[VehicleType]:
        """獲取可用的飛行器類型"""
        return list(cls._registry.keys())


# 預設配置
DEFAULT_MULTIROTOR_CONFIG = VehicleConfig(
    name="標準多旋翼",
    vehicle_type=VehicleType.MULTIROTOR,
    constraints=VehicleConstraints(
        max_speed=15.0,
        min_speed=0.0,        # 多旋翼可懸停
        max_vertical_speed=5.0,
        max_acceleration=3.0,
        max_deceleration=4.0,
        max_yaw_rate=90.0,
        max_yaw_acceleration=45.0,
        min_turn_radius=0.0,  # 多旋翼可原地轉向
        min_altitude=5.0,
        max_altitude=120.0,
        safety_margin=2.0,
        collision_radius=1.5
    )
)

DEFAULT_FIXED_WING_CONFIG = VehicleConfig(
    name="標準固定翼",
    vehicle_type=VehicleType.FIXED_WING,
    constraints=VehicleConstraints(
        max_speed=25.0,
        min_speed=12.0,       # 失速速度
        max_vertical_speed=8.0,
        max_acceleration=2.0,
        max_deceleration=3.0,
        max_yaw_rate=30.0,
        max_yaw_acceleration=15.0,
        min_turn_radius=30.0, # 固定翼需要最小轉彎半徑
        min_altitude=20.0,
        max_altitude=500.0,
        safety_margin=5.0,
        collision_radius=3.0
    )
)
