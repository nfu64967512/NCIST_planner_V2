"""
路徑平滑化模組
提供多種平滑化算法
"""

import math
from typing import List, Tuple, Optional
import numpy as np


class PathSmoother:
    """基礎路徑平滑器"""
    
    @staticmethod
    def smooth_corners(path: List[Tuple[float, float]], 
                      radius: float = 1.0) -> List[Tuple[float, float]]:
        """角點平滑（圓弧）"""
        if len(path) < 3:
            return path
        
        smoothed = [path[0]]
        
        for i in range(1, len(path) - 1):
            prev, curr, next = path[i-1], path[i], path[i+1]
            
            # 計算角平分線方向
            v1 = np.array([curr[0] - prev[0], curr[1] - prev[1]])
            v2 = np.array([next[0] - curr[0], next[1] - curr[1]])
            
            v1_norm = v1 / (np.linalg.norm(v1) + 1e-6)
            v2_norm = v2 / (np.linalg.norm(v2) + 1e-6)
            
            # 圓弧起點和終點
            arc_start = np.array(curr) - v1_norm * radius
            arc_end = np.array(curr) + v2_norm * radius
            
            smoothed.append(tuple(arc_start))
            smoothed.append(curr)
            smoothed.append(tuple(arc_end))
        
        smoothed.append(path[-1])
        return smoothed


class BezierSmoother:
    """貝茲曲線平滑器"""
    
    @staticmethod
    def cubic_bezier(p0: Tuple, p1: Tuple, p2: Tuple, p3: Tuple, 
                    num_points: int = 20) -> List[Tuple[float, float]]:
        """三次貝茲曲線"""
        points = []
        for i in range(num_points + 1):
            t = i / num_points
            
            # 貝茲公式
            x = (1-t)**3 * p0[0] + 3*(1-t)**2*t * p1[0] + \
                3*(1-t)*t**2 * p2[0] + t**3 * p3[0]
            y = (1-t)**3 * p0[1] + 3*(1-t)**2*t * p1[1] + \
                3*(1-t)*t**2 * p2[1] + t**3 * p3[1]
            
            points.append((x, y))
        
        return points


class BSplineSmoother:
    """B-Spline 平滑器"""
    
    @staticmethod
    def uniform_bspline(control_points: List[Tuple[float, float]], 
                       degree: int = 3,
                       num_points: int = 100) -> List[Tuple[float, float]]:
        """均勻 B-Spline"""
        n = len(control_points)
        if n < degree + 1:
            return control_points
        
        points = []
        for i in range(num_points + 1):
            u = i / num_points * (n - degree)
            point = BSplineSmoother._evaluate_bspline(control_points, degree, u)
            points.append(point)
        
        return points
    
    @staticmethod
    def _evaluate_bspline(control_points: List[Tuple], degree: int, u: float) -> Tuple[float, float]:
        """計算 B-Spline 點"""
        n = len(control_points)
        if u == 0:
            return control_points[0]
        if u >= n - degree:
            return control_points[-1]
        
        # De Boor 算法
        k = int(u) + degree
        points = [list(control_points[j]) for j in range(k - degree, k + 1)]
        
        for r in range(1, degree + 1):
            for j in range(degree, r - 1, -1):
                alpha = (u - (k - degree + j)) / (degree - r + 1)
                points[j] = [(1 - alpha) * points[j-1][d] + alpha * points[j][d] 
                           for d in range(2)]
        
        return tuple(points[degree])
