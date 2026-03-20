"""
幾何變換模組
提供2D/3D座標變換、旋轉、平移、縮放等功能
"""

import math
import numpy as np
from typing import Tuple, List, Optional


# ==========================================
# 2D 變換矩陣
# ==========================================
def rotation_matrix(angle_deg: float) -> np.ndarray:
    """
    生成2D旋轉矩陣
    
    參數:
        angle_deg: 旋轉角度（度，逆時針為正）
    
    返回:
        3x3 齊次旋轉矩陣
    """
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    
    return np.array([
        [cos_a, -sin_a, 0],
        [sin_a,  cos_a, 0],
        [0,      0,     1]
    ])


def translation_matrix(dx: float, dy: float) -> np.ndarray:
    """
    生成2D平移矩陣
    
    參數:
        dx, dy: X和Y方向的平移量
    
    返回:
        3x3 齊次平移矩陣
    """
    return np.array([
        [1, 0, dx],
        [0, 1, dy],
        [0, 0, 1]
    ])


def scaling_matrix(sx: float, sy: Optional[float] = None) -> np.ndarray:
    """
    生成2D縮放矩陣
    
    參數:
        sx: X方向縮放係數
        sy: Y方向縮放係數（如未指定，則與sx相同）
    
    返回:
        3x3 齊次縮放矩陣
    """
    if sy is None:
        sy = sx
    
    return np.array([
        [sx, 0,  0],
        [0,  sy, 0],
        [0,  0,  1]
    ])


def reflection_matrix(axis: str = 'x') -> np.ndarray:
    """
    生成2D反射矩陣
    
    參數:
        axis: 反射軸 ('x', 'y', 或 'origin')
    
    返回:
        3x3 齊次反射矩陣
    """
    if axis == 'x':
        # 沿X軸反射
        return np.array([
            [1,  0, 0],
            [0, -1, 0],
            [0,  0, 1]
        ])
    elif axis == 'y':
        # 沿Y軸反射
        return np.array([
            [-1, 0, 0],
            [0,  1, 0],
            [0,  0, 1]
        ])
    elif axis == 'origin':
        # 沿原點反射
        return np.array([
            [-1,  0, 0],
            [0,  -1, 0],
            [0,   0, 1]
        ])
    else:
        raise ValueError(f"未知的反射軸: {axis}")


# ==========================================
# 2D 變換類
# ==========================================
class Transform2D:
    """2D 仿射變換類"""
    
    def __init__(self, matrix: Optional[np.ndarray] = None):
        """
        初始化變換
        
        參數:
            matrix: 3x3 變換矩陣（如未指定，則為單位矩陣）
        """
        if matrix is None:
            self.matrix = np.eye(3)
        else:
            self.matrix = np.array(matrix)
    
    def rotate(self, angle_deg: float, center: Optional[Tuple[float, float]] = None):
        """
        添加旋轉變換
        
        參數:
            angle_deg: 旋轉角度（度）
            center: 旋轉中心（如未指定，則為原點）
        """
        if center is not None:
            cx, cy = center
            # 平移到原點 -> 旋轉 -> 平移回去
            self.matrix = (translation_matrix(cx, cy) @ 
                          rotation_matrix(angle_deg) @ 
                          translation_matrix(-cx, -cy) @ 
                          self.matrix)
        else:
            self.matrix = rotation_matrix(angle_deg) @ self.matrix
        
        return self
    
    def translate(self, dx: float, dy: float):
        """
        添加平移變換
        
        參數:
            dx, dy: 平移量
        """
        self.matrix = translation_matrix(dx, dy) @ self.matrix
        return self
    
    def scale(self, sx: float, sy: Optional[float] = None, 
             center: Optional[Tuple[float, float]] = None):
        """
        添加縮放變換
        
        參數:
            sx: X方向縮放係數
            sy: Y方向縮放係數
            center: 縮放中心
        """
        if center is not None:
            cx, cy = center
            self.matrix = (translation_matrix(cx, cy) @ 
                          scaling_matrix(sx, sy) @ 
                          translation_matrix(-cx, -cy) @ 
                          self.matrix)
        else:
            self.matrix = scaling_matrix(sx, sy) @ self.matrix
        
        return self
    
    def reflect(self, axis: str = 'x'):
        """
        添加反射變換
        
        參數:
            axis: 反射軸
        """
        self.matrix = reflection_matrix(axis) @ self.matrix
        return self
    
    def transform_point(self, x: float, y: float) -> Tuple[float, float]:
        """
        變換單個點
        
        參數:
            x, y: 點的座標
        
        返回:
            變換後的座標
        """
        point = np.array([x, y, 1])
        transformed = self.matrix @ point
        return (transformed[0], transformed[1])
    
    def transform_points(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        批量變換點
        
        參數:
            points: 點列表
        
        返回:
            變換後的點列表
        """
        # 轉換為齊次座標
        homogeneous = np.array([[x, y, 1] for x, y in points]).T
        
        # 應用變換
        transformed = self.matrix @ homogeneous
        
        # 轉換回笛卡爾座標
        return [(transformed[0, i], transformed[1, i]) 
                for i in range(transformed.shape[1])]
    
    def inverse(self) -> 'Transform2D':
        """
        獲取反變換
        
        返回:
            反變換對象
        """
        return Transform2D(np.linalg.inv(self.matrix))
    
    def copy(self) -> 'Transform2D':
        """
        複製變換
        
        返回:
            變換副本
        """
        return Transform2D(self.matrix.copy())
    
    def reset(self):
        """重置為單位變換"""
        self.matrix = np.eye(3)
        return self


# ==========================================
# 座標系轉換
# ==========================================
def latlon_to_local(lat: float, lon: float, 
                   lat0: float, lon0: float, 
                   angle_deg: float = 0.0) -> Tuple[float, float]:
    """
    將經緯度轉換為局部笛卡爾座標系（帶旋轉）
    
    參數:
        lat, lon: 目標點經緯度（度）
        lat0, lon0: 原點經緯度（度）
        angle_deg: 座標系旋轉角度（度）
    
    返回:
        (x, y): 局部座標（公尺）
    """
    # 基本平面投影
    cos_lat0 = math.cos(math.radians(lat0))
    x = (lon - lon0) * 111111.0 * cos_lat0
    y = (lat - lat0) * 111111.0
    
    # 應用旋轉
    if abs(angle_deg) > 1e-6:
        angle_rad = math.radians(angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        x_rot = cos_a * x - sin_a * y
        y_rot = sin_a * x + cos_a * y
        return (x_rot, y_rot)
    
    return (x, y)


def local_to_latlon(x: float, y: float,
                   lat0: float, lon0: float,
                   angle_deg: float = 0.0) -> Tuple[float, float]:
    """
    將局部笛卡爾座標轉換為經緯度（帶反旋轉）
    
    參數:
        x, y: 局部座標（公尺）
        lat0, lon0: 原點經緯度（度）
        angle_deg: 座標系旋轉角度（度）
    
    返回:
        (lat, lon): 經緯度（度）
    """
    # 應用反旋轉
    if abs(angle_deg) > 1e-6:
        angle_rad = math.radians(-angle_deg)  # 反向旋轉
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        x_orig = cos_a * x - sin_a * y
        y_orig = sin_a * x + cos_a * y
    else:
        x_orig = x
        y_orig = y
    
    # 反投影
    cos_lat0 = math.cos(math.radians(lat0))
    lat = y_orig / 111111.0 + lat0
    lon = x_orig / (111111.0 * cos_lat0) + lon0
    
    return (lat, lon)


def project_and_rotate(points: List[Tuple[float, float]], 
                      angle_deg: float) -> Tuple[List[Tuple[float, float]], 
                                                 float, float, float]:
    """
    投影並旋轉點集（用於路徑規劃）
    
    參數:
        points: 經緯度點列表
        angle_deg: 旋轉角度（度）
    
    返回:
        (rotated_points, lat0, lon0, cos_lat0): 
            - 旋轉後的點（公尺座標）
            - 參考點經緯度
            - 緯度餘弦值
    """
    if not points:
        return [], 0.0, 0.0, 1.0
    
    # 計算中心點
    lat0 = sum(p[0] for p in points) / len(points)
    lon0 = sum(p[1] for p in points) / len(points)
    cos_lat0 = math.cos(math.radians(lat0))
    
    # 投影並旋轉
    rotated = []
    angle_rad = math.radians(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    
    for lat, lon in points:
        # 投影到平面
        x = (lon - lon0) * 111111.0 * cos_lat0
        y = (lat - lat0) * 111111.0
        
        # 旋轉
        xr = cos_a * x - sin_a * y
        yr = sin_a * x + cos_a * y
        
        rotated.append((xr, yr))
    
    return rotated, lat0, lon0, cos_lat0


def rotate_back_points(points: List[Tuple[float, float]], 
                      angle_deg: float,
                      lat0: float, lon0: float, 
                      cos_lat0: float) -> List[Tuple[float, float]]:
    """
    反旋轉並反投影點集
    
    參數:
        points: 旋轉後的點（公尺座標）
        angle_deg: 原始旋轉角度（度）
        lat0, lon0: 參考點經緯度
        cos_lat0: 緯度餘弦值
    
    返回:
        經緯度點列表
    """
    if not points:
        return []
    
    result = []
    angle_rad = math.radians(-angle_deg)  # 反向旋轉
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    
    for xr, yr in points:
        # 反旋轉
        x = cos_a * xr - sin_a * yr
        y = sin_a * xr + cos_a * yr
        
        # 反投影
        lat = y / 111111.0 + lat0
        lon = x / (111111.0 * cos_lat0) + lon0
        
        result.append((lat, lon))
    
    return result


# ==========================================
# 仿射變換組合
# ==========================================
def affine_transform(points: List[Tuple[float, float]],
                    rotation: float = 0.0,
                    translation: Tuple[float, float] = (0.0, 0.0),
                    scale: Tuple[float, float] = (1.0, 1.0),
                    center: Optional[Tuple[float, float]] = None) -> List[Tuple[float, float]]:
    """
    應用組合仿射變換
    
    參數:
        points: 點列表
        rotation: 旋轉角度（度）
        translation: 平移量 (dx, dy)
        scale: 縮放係數 (sx, sy)
        center: 變換中心（如未指定，則為原點）
    
    返回:
        變換後的點列表
    """
    transform = Transform2D()
    
    # 如果指定了中心點，先平移到原點
    if center is not None:
        transform.translate(-center[0], -center[1])
    
    # 應用縮放
    if scale != (1.0, 1.0):
        transform.scale(scale[0], scale[1])
    
    # 應用旋轉
    if abs(rotation) > 1e-6:
        transform.rotate(rotation)
    
    # 如果指定了中心點，平移回去
    if center is not None:
        transform.translate(center[0], center[1])
    
    # 應用平移
    if translation != (0.0, 0.0):
        transform.translate(translation[0], translation[1])
    
    return transform.transform_points(points)


# ==========================================
# 座標系對齊
# ==========================================
def align_to_axis(points: List[Tuple[float, float]], 
                 p1: Tuple[float, float], 
                 p2: Tuple[float, float]) -> Tuple[List[Tuple[float, float]], float]:
    """
    將點集旋轉，使指定線段對齊到X軸
    
    參數:
        points: 點列表
        p1, p2: 定義方向的線段端點
    
    返回:
        (aligned_points, angle): 對齊後的點列表和旋轉角度
    """
    # 計算線段方向角
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle = math.degrees(math.atan2(dy, dx))
    
    # 旋轉使線段對齊X軸
    transform = Transform2D()
    transform.rotate(-angle, center=p1)
    
    aligned = transform.transform_points(points)
    
    return aligned, angle


def compute_bounding_box(points: List[Tuple[float, float]]) -> Tuple[float, float, float, float]:
    """
    計算點集的邊界框
    
    參數:
        points: 點列表
    
    返回:
        (min_x, min_y, max_x, max_y): 邊界框
    """
    if not points:
        return (0.0, 0.0, 0.0, 0.0)
    
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    
    return (min(xs), min(ys), max(xs), max(ys))


def normalize_polygon(polygon: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """
    標準化多邊形（平移到原點，縮放到單位尺度）
    
    參數:
        polygon: 多邊形頂點列表
    
    返回:
        標準化後的多邊形
    """
    if not polygon:
        return []
    
    # 計算邊界框
    min_x, min_y, max_x, max_y = compute_bounding_box(polygon)
    
    # 計算中心和尺度
    cx = (min_x + max_x) / 2
    cy = (min_y + max_y) / 2
    scale = max(max_x - min_x, max_y - min_y)
    
    if scale < 1e-10:
        return polygon
    
    # 應用標準化變換
    transform = Transform2D()
    transform.translate(-cx, -cy)
    transform.scale(1.0 / scale)
    
    return transform.transform_points(polygon)
