"""
樣條曲線模組
提供三次樣條和 Catmull-Rom 樣條插值
"""

import math
from typing import List, Tuple
import numpy as np


class CubicSpline:
    """三次樣條插值"""
    
    def __init__(self, points: List[Tuple[float, float]]):
        """初始化"""
        self.points = np.array(points)
        self.n = len(points)
        
        # 計算樣條係數
        self.coeffs_x = self._compute_coefficients(self.points[:, 0])
        self.coeffs_y = self._compute_coefficients(self.points[:, 1])
    
    def _compute_coefficients(self, values: np.ndarray) -> np.ndarray:
        """計算三次樣條係數"""
        n = len(values)
        h = np.ones(n - 1)
        
        # 構建三對角矩陣
        A = np.zeros((n, n))
        b = np.zeros(n)
        
        A[0, 0] = 1
        A[n-1, n-1] = 1
        
        for i in range(1, n - 1):
            A[i, i-1] = h[i-1]
            A[i, i] = 2 * (h[i-1] + h[i])
            A[i, i+1] = h[i]
            b[i] = 3 * ((values[i+1] - values[i]) / h[i] - 
                       (values[i] - values[i-1]) / h[i-1])
        
        # 求解
        return np.linalg.solve(A, b)
    
    def evaluate(self, t: float) -> Tuple[float, float]:
        """評估樣條在 t 處的值（0 <= t <= 1）"""
        if t <= 0:
            return tuple(self.points[0])
        if t >= 1:
            return tuple(self.points[-1])
        
        # 確定在哪個段
        segment = min(int(t * (self.n - 1)), self.n - 2)
        local_t = t * (self.n - 1) - segment
        
        # 計算 x 和 y
        x = self._evaluate_segment(self.coeffs_x, self.points[:, 0], segment, local_t)
        y = self._evaluate_segment(self.coeffs_y, self.points[:, 1], segment, local_t)
        
        return (x, y)
    
    def _evaluate_segment(self, coeffs: np.ndarray, values: np.ndarray, 
                         segment: int, t: float) -> float:
        """評估單個段"""
        c = coeffs[segment]
        d = coeffs[segment + 1]
        a = values[segment]
        b = values[segment + 1]
        
        return a * (1 - t) ** 3 + b * t ** 3 + c * (1 - t) ** 2 * t + d * (1 - t) * t ** 2
    
    def generate_path(self, num_points: int = 100) -> List[Tuple[float, float]]:
        """生成路徑點"""
        path = []
        for i in range(num_points + 1):
            t = i / num_points
            path.append(self.evaluate(t))
        return path


class CatmullRomSpline:
    """Catmull-Rom 樣條"""
    
    @staticmethod
    def interpolate(points: List[Tuple[float, float]], 
                   num_points: int = 100,
                   alpha: float = 0.5) -> List[Tuple[float, float]]:
        """
        Catmull-Rom 插值
        alpha: 0=均勻, 0.5=向心, 1.0=弦長
        """
        if len(points) < 4:
            return points
        
        result = []
        
        for i in range(len(points) - 3):
            p0, p1, p2, p3 = points[i:i+4]
            
            # 計算參數化距離
            def get_t(t, p0, p1):
                a = (p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2
                b = a ** alpha
                return t + b
            
            t0 = 0
            t1 = get_t(t0, p0, p1)
            t2 = get_t(t1, p1, p2)
            t3 = get_t(t2, p2, p3)
            
            # 插值
            for j in range(num_points + 1):
                t = t1 + (t2 - t1) * j / num_points
                
                A1 = [(t1 - t) / (t1 - t0) * p0[k] + (t - t0) / (t1 - t0) * p1[k] 
                      for k in range(2)]
                A2 = [(t2 - t) / (t2 - t1) * p1[k] + (t - t1) / (t2 - t1) * p2[k] 
                      for k in range(2)]
                A3 = [(t3 - t) / (t3 - t2) * p2[k] + (t - t2) / (t3 - t2) * p3[k] 
                      for k in range(2)]
                
                B1 = [(t2 - t) / (t2 - t0) * A1[k] + (t - t0) / (t2 - t0) * A2[k] 
                      for k in range(2)]
                B2 = [(t3 - t) / (t3 - t1) * A2[k] + (t - t1) / (t3 - t1) * A3[k] 
                      for k in range(2)]
                
                C = [(t2 - t) / (t2 - t1) * B1[k] + (t - t1) / (t2 - t1) * B2[k] 
                     for k in range(2)]
                
                result.append(tuple(C))
        
        return result
