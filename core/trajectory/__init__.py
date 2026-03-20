"""
Trajectory 軌跡處理模組
路徑平滑和軌跡優化
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class TrajectoryConfig:
    """軌跡配置"""
    max_velocity: float = 5.0       # 最大速度 (m/s)
    max_acceleration: float = 2.0   # 最大加速度 (m/s²)
    time_step: float = 0.1          # 時間步長 (s)
    smoothing_factor: float = 0.5   # 平滑係數
    min_segment_length: float = 1.0 # 最小線段長度 (m)


class TrajectoryPoint:
    """軌跡點"""
    
    def __init__(self, position: np.ndarray, 
                 velocity: np.ndarray = None,
                 acceleration: np.ndarray = None,
                 time: float = 0.0):
        self.position = np.array(position)
        self.velocity = np.zeros(3) if velocity is None else np.array(velocity)
        self.acceleration = np.zeros(3) if acceleration is None else np.array(acceleration)
        self.time = time
    
    def __repr__(self):
        return f"TrajectoryPoint(pos={self.position}, t={self.time:.2f})"


class PathSmoother:
    """
    路徑平滑器
    
    使用多種方法平滑路徑：
    - 移動平均
    - Bezier 曲線
    - B-Spline 曲線
    - Douglas-Peucker 簡化
    """
    
    def __init__(self, config: TrajectoryConfig = None):
        self.config = config or TrajectoryConfig()
    
    def smooth_moving_average(self, path: List[np.ndarray], 
                              window_size: int = 3) -> List[np.ndarray]:
        """
        移動平均平滑
        
        Args:
            path: 原始路徑點列表
            window_size: 平滑窗口大小
            
        Returns:
            平滑後的路徑
        """
        if len(path) < window_size:
            return path
        
        smoothed = []
        half_window = window_size // 2
        
        # 保留起點
        smoothed.append(path[0].copy())
        
        # 平滑中間點
        for i in range(1, len(path) - 1):
            start = max(0, i - half_window)
            end = min(len(path), i + half_window + 1)
            
            avg_point = np.mean(path[start:end], axis=0)
            smoothed.append(avg_point)
        
        # 保留終點
        smoothed.append(path[-1].copy())
        
        return smoothed
    
    def smooth_bezier(self, path: List[np.ndarray], 
                      num_points: int = 100) -> List[np.ndarray]:
        """
        Bezier 曲線平滑
        
        使用控制點生成平滑曲線
        
        Args:
            path: 控制點列表
            num_points: 輸出點數量
            
        Returns:
            平滑曲線上的點
        """
        if len(path) < 2:
            return path
        
        n = len(path) - 1
        smoothed = []
        
        for t in np.linspace(0, 1, num_points):
            point = np.zeros_like(path[0])
            
            for i, p in enumerate(path):
                # Bernstein 多項式
                coeff = self._binomial_coefficient(n, i)
                coeff *= (1 - t) ** (n - i) * t ** i
                point = point + coeff * p
            
            smoothed.append(point)
        
        return smoothed
    
    def _binomial_coefficient(self, n: int, k: int) -> int:
        """計算二項式係數 C(n, k)"""
        if k > n:
            return 0
        if k == 0 or k == n:
            return 1
        
        result = 1
        for i in range(min(k, n - k)):
            result = result * (n - i) // (i + 1)
        return result
    
    def smooth_bspline(self, path: List[np.ndarray], 
                       degree: int = 3,
                       num_points: int = 100) -> List[np.ndarray]:
        """
        B-Spline 曲線平滑
        
        Args:
            path: 控制點列表
            degree: 曲線階數
            num_points: 輸出點數量
            
        Returns:
            平滑曲線上的點
        """
        if len(path) <= degree:
            return self.smooth_bezier(path, num_points)
        
        # 生成節點向量（均勻）
        n = len(path)
        k = degree + 1
        num_knots = n + k
        knots = np.concatenate([
            np.zeros(k),
            np.linspace(0, 1, num_knots - 2 * k),
            np.ones(k)
        ])
        
        smoothed = []
        
        for t in np.linspace(0, 1 - 1e-10, num_points):
            point = np.zeros_like(path[0], dtype=float)
            
            for i in range(n):
                basis = self._bspline_basis(i, degree, t, knots)
                point = point + basis * path[i]
            
            smoothed.append(point)
        
        return smoothed
    
    def _bspline_basis(self, i: int, k: int, t: float, 
                       knots: np.ndarray) -> float:
        """
        計算 B-Spline 基函數（Cox-de Boor 遞迴公式）
        """
        if k == 0:
            return 1.0 if knots[i] <= t < knots[i + 1] else 0.0
        
        d1 = knots[i + k] - knots[i]
        d2 = knots[i + k + 1] - knots[i + 1]
        
        c1 = (t - knots[i]) / d1 if d1 > 1e-10 else 0.0
        c2 = (knots[i + k + 1] - t) / d2 if d2 > 1e-10 else 0.0
        
        return (c1 * self._bspline_basis(i, k - 1, t, knots) +
                c2 * self._bspline_basis(i + 1, k - 1, t, knots))
    
    def simplify_douglas_peucker(self, path: List[np.ndarray],
                                 epsilon: float = 1.0) -> List[np.ndarray]:
        """
        Douglas-Peucker 路徑簡化
        
        減少路徑點數量同時保持形狀
        
        Args:
            path: 原始路徑
            epsilon: 最大偏離距離閾值 (m)
            
        Returns:
            簡化後的路徑
        """
        if len(path) < 3:
            return path
        
        # 找到距離線段最遠的點
        max_dist = 0
        max_idx = 0
        
        start = path[0]
        end = path[-1]
        
        for i in range(1, len(path) - 1):
            dist = self._point_to_line_distance(path[i], start, end)
            if dist > max_dist:
                max_dist = dist
                max_idx = i
        
        # 如果最大距離大於閾值，遞迴處理
        if max_dist > epsilon:
            left = self.simplify_douglas_peucker(path[:max_idx + 1], epsilon)
            right = self.simplify_douglas_peucker(path[max_idx:], epsilon)
            return left[:-1] + right
        else:
            return [start, end]
    
    def _point_to_line_distance(self, point: np.ndarray,
                                line_start: np.ndarray,
                                line_end: np.ndarray) -> float:
        """計算點到線段的距離"""
        line_vec = line_end - line_start
        point_vec = point - line_start
        
        line_len = np.linalg.norm(line_vec)
        if line_len < 1e-10:
            return np.linalg.norm(point_vec)
        
        line_unit = line_vec / line_len
        proj_length = np.dot(point_vec, line_unit)
        
        if proj_length < 0:
            return np.linalg.norm(point_vec)
        elif proj_length > line_len:
            return np.linalg.norm(point - line_end)
        else:
            proj_point = line_start + proj_length * line_unit
            return np.linalg.norm(point - proj_point)


class TrajectoryGenerator:
    """
    軌跡生成器
    
    從路徑生成包含速度和加速度的完整軌跡
    """
    
    def __init__(self, config: TrajectoryConfig = None):
        self.config = config or TrajectoryConfig()
        self.smoother = PathSmoother(config)
    
    def generate_trajectory(self, path: List[np.ndarray],
                           smooth: bool = True,
                           smooth_method: str = 'moving_average') -> List[TrajectoryPoint]:
        """
        從路徑生成軌跡
        
        Args:
            path: 路徑點列表
            smooth: 是否平滑路徑
            smooth_method: 平滑方法 ('moving_average', 'bezier', 'bspline')
            
        Returns:
            軌跡點列表
        """
        if len(path) < 2:
            return [TrajectoryPoint(p) for p in path]
        
        # 可選平滑
        if smooth:
            if smooth_method == 'bezier':
                path = self.smoother.smooth_bezier(path)
            elif smooth_method == 'bspline':
                path = self.smoother.smooth_bspline(path)
            else:
                path = self.smoother.smooth_moving_average(path)
        
        # 計算每段的距離和速度
        trajectory = []
        current_time = 0.0
        
        for i, pos in enumerate(path):
            if i == 0:
                # 起點
                velocity = np.zeros(3)
                acceleration = np.zeros(3)
            elif i == len(path) - 1:
                # 終點
                velocity = np.zeros(3)
                acceleration = np.zeros(3)
            else:
                # 中間點：計算速度方向和大小
                prev_pos = path[i - 1]
                next_pos = path[i + 1]
                
                # 速度方向：指向下一點
                direction = next_pos - pos
                distance = np.linalg.norm(direction)
                
                if distance > 1e-10:
                    velocity = direction / distance * self.config.max_velocity
                else:
                    velocity = np.zeros(3)
                
                # 加速度：速度變化
                prev_dir = pos - prev_pos
                prev_dist = np.linalg.norm(prev_dir)
                
                if prev_dist > 1e-10:
                    prev_vel = prev_dir / prev_dist * self.config.max_velocity
                    acceleration = (velocity - prev_vel) / self.config.time_step
                    
                    # 限制加速度
                    acc_mag = np.linalg.norm(acceleration)
                    if acc_mag > self.config.max_acceleration:
                        acceleration = acceleration / acc_mag * self.config.max_acceleration
                else:
                    acceleration = np.zeros(3)
            
            # 計算時間
            if i > 0:
                segment_dist = np.linalg.norm(pos - path[i - 1])
                segment_time = segment_dist / self.config.max_velocity
                current_time += segment_time
            
            trajectory.append(TrajectoryPoint(
                position=pos,
                velocity=velocity,
                acceleration=acceleration,
                time=current_time
            ))
        
        return trajectory
    
    def generate_zigzag_trajectory(self, scan_lines: List[Tuple[np.ndarray, np.ndarray]],
                                   deceleration_distance: float = 5.0) -> List[TrajectoryPoint]:
        """
        為 Zigzag 掃描模式生成軌跡
        
        在轉向點前後添加減速/加速段
        
        Args:
            scan_lines: 掃描線列表 [(start, end), ...]
            deceleration_distance: 減速距離 (m)
            
        Returns:
            軌跡點列表
        """
        if not scan_lines:
            return []
        
        trajectory = []
        current_time = 0.0
        
        for i, (start, end) in enumerate(scan_lines):
            # 計算掃描線方向
            direction = end - start
            length = np.linalg.norm(direction)
            
            if length < 1e-10:
                continue
            
            unit_dir = direction / length
            
            # 確定 zigzag 方向
            if i % 2 == 0:
                line_start, line_end = start, end
            else:
                line_start, line_end = end, start
                unit_dir = -unit_dir
            
            # 添加減速點（如果有前一條線）
            if trajectory and deceleration_distance > 0:
                # 在轉向前添加減速點
                decel_pos = line_start - unit_dir * min(deceleration_distance, length / 4)
                
                # 減速階段
                segment_dist = np.linalg.norm(decel_pos - trajectory[-1].position)
                current_time += segment_dist / (self.config.max_velocity * 0.5)
                
                trajectory.append(TrajectoryPoint(
                    position=decel_pos,
                    velocity=unit_dir * self.config.max_velocity * 0.3,
                    time=current_time
                ))
            
            # 添加掃描線起點
            if trajectory:
                segment_dist = np.linalg.norm(line_start - trajectory[-1].position)
                current_time += segment_dist / (self.config.max_velocity * 0.5)
            
            trajectory.append(TrajectoryPoint(
                position=line_start.copy(),
                velocity=unit_dir * self.config.max_velocity,
                time=current_time
            ))
            
            # 添加掃描線終點
            current_time += length / self.config.max_velocity
            
            trajectory.append(TrajectoryPoint(
                position=line_end.copy(),
                velocity=unit_dir * self.config.max_velocity,
                time=current_time
            ))
        
        return trajectory
    
    def interpolate_trajectory(self, trajectory: List[TrajectoryPoint],
                               dt: float = 0.1) -> List[TrajectoryPoint]:
        """
        軌跡插值
        
        在軌跡點之間進行線性或三次插值
        
        Args:
            trajectory: 原始軌跡
            dt: 插值時間步長
            
        Returns:
            插值後的軌跡
        """
        if len(trajectory) < 2:
            return trajectory
        
        interpolated = []
        
        for i in range(len(trajectory) - 1):
            p1 = trajectory[i]
            p2 = trajectory[i + 1]
            
            # 計算需要的插值點數
            segment_time = p2.time - p1.time
            if segment_time < dt:
                interpolated.append(p1)
                continue
            
            num_steps = int(segment_time / dt)
            
            for step in range(num_steps):
                t = step * dt / segment_time
                
                # 線性插值位置
                pos = p1.position + t * (p2.position - p1.position)
                
                # 線性插值速度
                vel = p1.velocity + t * (p2.velocity - p1.velocity)
                
                # 計算加速度
                acc = (p2.velocity - p1.velocity) / segment_time
                
                interpolated.append(TrajectoryPoint(
                    position=pos,
                    velocity=vel,
                    acceleration=acc,
                    time=p1.time + step * dt
                ))
        
        # 添加最後一點
        interpolated.append(trajectory[-1])
        
        return interpolated


__all__ = [
    'TrajectoryConfig',
    'TrajectoryPoint',
    'PathSmoother',
    'TrajectoryGenerator'
]
