"""
時間最優軌跡規劃模組
基於速度規劃和加速度約束
"""

import math
from typing import List, Tuple, Optional
from dataclasses import dataclass
import numpy as np


@dataclass
class VelocityProfile:
    """速度規劃配置"""
    max_velocity: float = 10.0      # 最大速度 [m/s]
    max_acceleration: float = 2.0   # 最大加速度 [m/s²]
    max_deceleration: float = 3.0   # 最大減速度 [m/s²]
    max_jerk: float = 5.0           # 最大加加速度 [m/s³]


class TimeOptimalPlanner:
    """時間最優軌跡規劃器"""
    
    def __init__(self, profile: Optional[VelocityProfile] = None):
        """初始化"""
        self.profile = profile or VelocityProfile()
    
    def plan_velocity_profile(self, 
                             path: List[Tuple[float, float]],
                             initial_velocity: float = 0.0,
                             final_velocity: float = 0.0) -> List[Tuple[float, float, float]]:
        """
        規劃速度分布
        
        參數:
            path: 路徑點列表
            initial_velocity: 初始速度
            final_velocity: 終點速度
        
        返回:
            [(x, y, v), ...] 帶速度的路徑點
        """
        if len(path) < 2:
            return [(p[0], p[1], 0.0) for p in path]
        
        # 計算路徑段長度
        distances = self._compute_segment_distances(path)
        cumulative_distances = np.cumsum([0] + distances)
        total_distance = cumulative_distances[-1]
        
        # 計算最大允許速度（基於曲率）
        max_velocities = self._compute_curvature_velocities(path)
        
        # 前向傳播（加速）
        velocities_forward = [initial_velocity]
        for i in range(1, len(path)):
            dist = distances[i - 1]
            max_v = max_velocities[i]
            
            # v² = v₀² + 2as
            v_squared = velocities_forward[-1] ** 2 + \
                       2 * self.profile.max_acceleration * dist
            v = min(math.sqrt(max(0, v_squared)), max_v)
            
            velocities_forward.append(v)
        
        # 後向傳播（減速）
        velocities_backward = [final_velocity]
        for i in range(len(path) - 2, -1, -1):
            dist = distances[i]
            max_v = max_velocities[i]
            
            v_squared = velocities_backward[0] ** 2 + \
                       2 * self.profile.max_deceleration * dist
            v = min(math.sqrt(max(0, v_squared)), max_v)
            
            velocities_backward.insert(0, v)
        
        # 取最小值
        velocities = [min(vf, vb) for vf, vb in zip(velocities_forward, velocities_backward)]
        
        # 組合結果
        trajectory = []
        for i, point in enumerate(path):
            trajectory.append((point[0], point[1], velocities[i]))
        
        return trajectory
    
    def _compute_segment_distances(self, path: List[Tuple[float, float]]) -> List[float]:
        """計算路徑段距離"""
        distances = []
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            distances.append(math.sqrt(dx ** 2 + dy ** 2))
        return distances
    
    def _compute_curvature_velocities(self, path: List[Tuple[float, float]]) -> List[float]:
        """基於曲率計算最大允許速度"""
        max_vels = [self.profile.max_velocity]
        
        for i in range(1, len(path) - 1):
            # 計算曲率
            p1, p2, p3 = path[i-1], path[i], path[i+1]
            
            # 使用 Menger 曲率
            a = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
            b = math.sqrt((p3[0] - p2[0]) ** 2 + (p3[1] - p2[1]) ** 2)
            c = math.sqrt((p3[0] - p1[0]) ** 2 + (p3[1] - p1[1]) ** 2)
            
            if a > 1e-6 and b > 1e-6 and c > 1e-6:
                s = (a + b + c) / 2
                area = math.sqrt(max(0, s * (s - a) * (s - b) * (s - c)))
                curvature = 4 * area / (a * b * c) if area > 1e-6 else 0
                
                # v = sqrt(a_max / curvature)
                if curvature > 1e-6:
                    max_v = math.sqrt(self.profile.max_acceleration / curvature)
                    max_vels.append(min(max_v, self.profile.max_velocity))
                else:
                    max_vels.append(self.profile.max_velocity)
            else:
                max_vels.append(self.profile.max_velocity)
        
        max_vels.append(self.profile.max_velocity)
        return max_vels
    
    def compute_time_stamps(self, trajectory: List[Tuple[float, float, float]]) -> List[float]:
        """計算時間戳"""
        times = [0.0]
        
        for i in range(1, len(trajectory)):
            x1, y1, v1 = trajectory[i-1]
            x2, y2, v2 = trajectory[i]
            
            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            avg_velocity = (v1 + v2) / 2
            
            if avg_velocity > 1e-6:
                dt = distance / avg_velocity
            else:
                dt = 0.0
            
            times.append(times[-1] + dt)
        
        return times
