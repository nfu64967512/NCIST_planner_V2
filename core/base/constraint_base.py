"""
約束條件基類模組
定義各種飛行約束條件：速度、加速度、高度、地理圍欄等
"""

from abc import ABC, abstractmethod
from typing import Tuple, List, Optional
from dataclasses import dataclass


# ==========================================
# 狀態表示
# ==========================================
@dataclass
class State:
    """飛行器狀態"""
    position: Tuple[float, float, float]  # (x, y, z) 或 (lat, lon, alt)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # (vx, vy, vz)
    acceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # (ax, ay, az)
    heading: float = 0.0  # 航向角（度）
    time: float = 0.0  # 時間戳


# ==========================================
# 約束條件基類
# ==========================================
class Constraint(ABC):
    """約束條件抽象基類"""
    
    def __init__(self, name: str = ""):
        """
        初始化約束
        
        參數:
            name: 約束名稱
        """
        self.name = name
        self.enabled = True
    
    @abstractmethod
    def is_satisfied(self, state: State) -> bool:
        """
        檢查狀態是否滿足約束
        
        參數:
            state: 當前狀態
        
        返回:
            是否滿足約束
        """
        pass
    
    @abstractmethod
    def violation_degree(self, state: State) -> float:
        """
        計算違反程度
        
        參數:
            state: 當前狀態
        
        返回:
            違反程度（0表示滿足，正值表示違反程度）
        """
        pass
    
    def enable(self):
        """啟用約束"""
        self.enabled = True
    
    def disable(self):
        """禁用約束"""
        self.enabled = False
    
    def __str__(self) -> str:
        status = "啟用" if self.enabled else "禁用"
        return f"{self.name} ({status})"


# ==========================================
# 速度約束
# ==========================================
class VelocityConstraint(Constraint):
    """速度約束"""
    
    def __init__(self, min_speed: float = 0.0, max_speed: float = 20.0, 
                 name: str = "速度約束"):
        """
        初始化速度約束
        
        參數:
            min_speed: 最小速度（m/s）
            max_speed: 最大速度（m/s）
            name: 約束名稱
        """
        super().__init__(name)
        self.min_speed = min_speed
        self.max_speed = max_speed
    
    def is_satisfied(self, state: State) -> bool:
        """檢查速度是否在限制範圍內"""
        if not self.enabled:
            return True
        
        vx, vy, vz = state.velocity
        speed = (vx**2 + vy**2 + vz**2) ** 0.5
        
        return self.min_speed <= speed <= self.max_speed
    
    def violation_degree(self, state: State) -> float:
        """計算速度違反程度"""
        if not self.enabled:
            return 0.0
        
        vx, vy, vz = state.velocity
        speed = (vx**2 + vy**2 + vz**2) ** 0.5
        
        if speed < self.min_speed:
            return self.min_speed - speed
        elif speed > self.max_speed:
            return speed - self.max_speed
        else:
            return 0.0
    
    def get_speed(self, state: State) -> float:
        """獲取當前速度"""
        vx, vy, vz = state.velocity
        return (vx**2 + vy**2 + vz**2) ** 0.5


# ==========================================
# 加速度約束
# ==========================================
class AccelerationConstraint(Constraint):
    """加速度約束"""
    
    def __init__(self, max_acceleration: float = 3.0,
                 max_deceleration: float = 4.0,
                 name: str = "加速度約束"):
        """
        初始化加速度約束
        
        參數:
            max_acceleration: 最大加速度（m/s²）
            max_deceleration: 最大減速度（m/s²）
            name: 約束名稱
        """
        super().__init__(name)
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration
    
    def is_satisfied(self, state: State) -> bool:
        """檢查加速度是否在限制範圍內"""
        if not self.enabled:
            return True
        
        ax, ay, az = state.acceleration
        accel_magnitude = (ax**2 + ay**2 + az**2) ** 0.5
        
        # 判斷是加速還是減速（簡化處理）
        vx, vy, vz = state.velocity
        speed = (vx**2 + vy**2 + vz**2) ** 0.5
        
        if speed > 0:
            # 計算加速度方向與速度方向的夾角
            dot_product = (ax * vx + ay * vy + az * vz) / (speed + 1e-10)
            
            if dot_product > 0:
                # 加速
                return accel_magnitude <= self.max_acceleration
            else:
                # 減速
                return accel_magnitude <= self.max_deceleration
        
        return accel_magnitude <= self.max_acceleration
    
    def violation_degree(self, state: State) -> float:
        """計算加速度違反程度"""
        if not self.enabled:
            return 0.0
        
        ax, ay, az = state.acceleration
        accel_magnitude = (ax**2 + ay**2 + az**2) ** 0.5
        
        vx, vy, vz = state.velocity
        speed = (vx**2 + vy**2 + vz**2) ** 0.5
        
        if speed > 0:
            dot_product = (ax * vx + ay * vy + az * vz) / (speed + 1e-10)
            
            if dot_product > 0:
                # 加速
                if accel_magnitude > self.max_acceleration:
                    return accel_magnitude - self.max_acceleration
            else:
                # 減速
                if accel_magnitude > self.max_deceleration:
                    return accel_magnitude - self.max_deceleration
        else:
            if accel_magnitude > self.max_acceleration:
                return accel_magnitude - self.max_acceleration
        
        return 0.0


# ==========================================
# 高度約束
# ==========================================
class AltitudeConstraint(Constraint):
    """高度約束"""
    
    def __init__(self, min_altitude: float = 5.0, 
                 max_altitude: float = 500.0,
                 name: str = "高度約束"):
        """
        初始化高度約束
        
        參數:
            min_altitude: 最小高度（m）
            max_altitude: 最大高度（m）
            name: 約束名稱
        """
        super().__init__(name)
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
    
    def is_satisfied(self, state: State) -> bool:
        """檢查高度是否在限制範圍內"""
        if not self.enabled:
            return True
        
        altitude = state.position[2]
        return self.min_altitude <= altitude <= self.max_altitude
    
    def violation_degree(self, state: State) -> float:
        """計算高度違反程度"""
        if not self.enabled:
            return 0.0
        
        altitude = state.position[2]
        
        if altitude < self.min_altitude:
            return self.min_altitude - altitude
        elif altitude > self.max_altitude:
            return altitude - self.max_altitude
        else:
            return 0.0


# ==========================================
# 地理圍欄約束
# ==========================================
class GeofenceConstraint(Constraint):
    """地理圍欄約束"""
    
    def __init__(self, boundary: List[Tuple[float, float]], 
                 name: str = "地理圍欄"):
        """
        初始化地理圍欄約束
        
        參數:
            boundary: 邊界多邊形頂點列表 [(lat, lon), ...]
            name: 約束名稱
        """
        super().__init__(name)
        self.boundary = boundary
    
    def is_satisfied(self, state: State) -> bool:
        """檢查位置是否在圍欄內"""
        if not self.enabled:
            return True
        
        lat, lon = state.position[0], state.position[1]
        return self._point_in_polygon(lat, lon)
    
    def violation_degree(self, state: State) -> float:
        """計算圍欄違反程度（距離邊界的距離）"""
        if not self.enabled:
            return 0.0
        
        if self.is_satisfied(state):
            return 0.0
        
        # 計算到最近邊界的距離
        lat, lon = state.position[0], state.position[1]
        min_distance = float('inf')
        
        n = len(self.boundary)
        for i in range(n):
            p1 = self.boundary[i]
            p2 = self.boundary[(i + 1) % n]
            
            distance = self._point_to_segment_distance(
                (lat, lon), p1, p2
            )
            min_distance = min(min_distance, distance)
        
        return min_distance
    
    def _point_in_polygon(self, lat: float, lon: float) -> bool:
        """射線法判斷點是否在多邊形內"""
        n = len(self.boundary)
        inside = False
        
        p1_lat, p1_lon = self.boundary[0]
        for i in range(1, n + 1):
            p2_lat, p2_lon = self.boundary[i % n]
            
            if lon > min(p1_lon, p2_lon):
                if lon <= max(p1_lon, p2_lon):
                    if lat <= max(p1_lat, p2_lat):
                        if p1_lon != p2_lon:
                            xinters = (lon - p1_lon) * (p2_lat - p1_lat) / (p2_lon - p1_lon) + p1_lat
                        if p1_lat == p2_lat or lat <= xinters:
                            inside = not inside
            
            p1_lat, p1_lon = p2_lat, p2_lon
        
        return inside
    
    def _point_to_segment_distance(self, point: Tuple[float, float],
                                   p1: Tuple[float, float],
                                   p2: Tuple[float, float]) -> float:
        """計算點到線段的距離（簡化版，使用平面近似）"""
        import math
        
        px, py = point
        x1, y1 = p1
        x2, y2 = p2
        
        dx = x2 - x1
        dy = y2 - y1
        
        if abs(dx) < 1e-10 and abs(dy) < 1e-10:
            return math.sqrt((px - x1)**2 + (py - y1)**2)
        
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
        
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        return math.sqrt((px - closest_x)**2 + (py - closest_y)**2) * 111111.0  # 轉換為公尺


# ==========================================
# 角速度約束
# ==========================================
class YawRateConstraint(Constraint):
    """角速度約束"""
    
    def __init__(self, max_yaw_rate: float = 60.0,
                 name: str = "角速度約束"):
        """
        初始化角速度約束
        
        參數:
            max_yaw_rate: 最大角速度（度/秒）
            name: 約束名稱
        """
        super().__init__(name)
        self.max_yaw_rate = max_yaw_rate
        self.previous_heading: Optional[float] = None
        self.previous_time: Optional[float] = None
    
    def is_satisfied(self, state: State) -> bool:
        """檢查角速度是否在限制範圍內"""
        if not self.enabled:
            return True
        
        if self.previous_heading is None or self.previous_time is None:
            self.previous_heading = state.heading
            self.previous_time = state.time
            return True
        
        dt = state.time - self.previous_time
        if dt <= 0:
            return True
        
        # 計算角速度
        dheading = abs(state.heading - self.previous_heading)
        if dheading > 180:
            dheading = 360 - dheading
        
        yaw_rate = dheading / dt
        
        # 更新歷史
        self.previous_heading = state.heading
        self.previous_time = state.time
        
        return yaw_rate <= self.max_yaw_rate
    
    def violation_degree(self, state: State) -> float:
        """計算角速度違反程度"""
        if not self.enabled or self.previous_heading is None:
            return 0.0
        
        dt = state.time - self.previous_time
        if dt <= 0:
            return 0.0
        
        dheading = abs(state.heading - self.previous_heading)
        if dheading > 180:
            dheading = 360 - dheading
        
        yaw_rate = dheading / dt
        
        if yaw_rate > self.max_yaw_rate:
            return yaw_rate - self.max_yaw_rate
        
        return 0.0
    
    def reset(self):
        """重置歷史"""
        self.previous_heading = None
        self.previous_time = None


# ==========================================
# 組合約束
# ==========================================
class CompositeConstraint(Constraint):
    """組合約束（包含多個子約束）"""
    
    def __init__(self, constraints: Optional[List[Constraint]] = None,
                 name: str = "組合約束"):
        """
        初始化組合約束
        
        參數:
            constraints: 子約束列表
            name: 約束名稱
        """
        super().__init__(name)
        self.constraints = constraints or []
    
    def add_constraint(self, constraint: Constraint):
        """添加子約束"""
        self.constraints.append(constraint)
    
    def remove_constraint(self, constraint: Constraint):
        """移除子約束"""
        if constraint in self.constraints:
            self.constraints.remove(constraint)
    
    def is_satisfied(self, state: State) -> bool:
        """檢查所有子約束是否滿足"""
        if not self.enabled:
            return True
        
        return all(c.is_satisfied(state) for c in self.constraints if c.enabled)
    
    def violation_degree(self, state: State) -> float:
        """計算總違反程度（最大違反）"""
        if not self.enabled:
            return 0.0
        
        violations = [c.violation_degree(state) 
                     for c in self.constraints if c.enabled]
        
        return max(violations) if violations else 0.0
    
    def get_violated_constraints(self, state: State) -> List[Constraint]:
        """獲取所有違反的約束"""
        return [c for c in self.constraints 
                if c.enabled and not c.is_satisfied(state)]
    
    def __len__(self) -> int:
        return len(self.constraints)
    
    def __iter__(self):
        return iter(self.constraints)
