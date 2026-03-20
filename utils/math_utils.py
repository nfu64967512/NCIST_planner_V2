"""
數學工具模組
提供角度轉換、距離計算、幾何計算等數學工具函數
"""

import math
from typing import Tuple, Optional


# ==========================================
# 常數定義
# ==========================================
EARTH_RADIUS_M = 6378137.0  # 地球半徑（公尺）- WGS84
DEG_PER_METER_LAT = 1.0 / 111111.0  # 緯度每公尺的度數
PI = math.pi
TWO_PI = 2 * math.pi
HALF_PI = math.pi / 2


# ==========================================
# 角度轉換
# ==========================================
def deg_to_rad(degrees: float) -> float:
    """
    角度轉弧度
    
    參數:
        degrees: 角度值
    
    返回:
        弧度值
    """
    return degrees * PI / 180.0


def rad_to_deg(radians: float) -> float:
    """
    弧度轉角度
    
    參數:
        radians: 弧度值
    
    返回:
        角度值
    """
    return radians * 180.0 / PI


def normalize_angle(angle_deg: float, min_angle: float = -180.0) -> float:
    """
    標準化角度到指定範圍
    
    參數:
        angle_deg: 角度值
        min_angle: 最小角度（預設-180）
    
    返回:
        標準化後的角度（在 [min_angle, min_angle+360) 範圍內）
    """
    angle = angle_deg
    while angle >= min_angle + 360.0:
        angle -= 360.0
    while angle < min_angle:
        angle += 360.0
    return angle


def angle_difference(angle1_deg: float, angle2_deg: float) -> float:
    """
    計算兩個角度之間的最小差值（-180 到 180 度）
    
    參數:
        angle1_deg: 第一個角度
        angle2_deg: 第二個角度
    
    返回:
        角度差值
    """
    diff = normalize_angle(angle2_deg - angle1_deg, -180.0)
    return diff


# ==========================================
# 距離計算
# ==========================================
def haversine_distance(lat1: float, lon1: float, 
                      lat2: float, lon2: float) -> float:
    """
    使用 Haversine 公式計算兩點間的大圓距離（更精確）
    
    參數:
        lat1, lon1: 第一個點的緯度和經度（度）
        lat2, lon2: 第二個點的緯度和經度（度）
    
    返回:
        距離（公尺）
    """
    # 轉換為弧度
    lat1_rad = deg_to_rad(lat1)
    lat2_rad = deg_to_rad(lat2)
    dlat = deg_to_rad(lat2 - lat1)
    dlon = deg_to_rad(lon2 - lon1)
    
    # Haversine 公式
    a = (math.sin(dlat / 2) ** 2 + 
         math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    distance = EARTH_RADIUS_M * c
    return distance


def planar_distance(lat1: float, lon1: float, 
                   lat2: float, lon2: float) -> float:
    """
    使用平面近似計算兩點間距離（速度較快，適用於短距離）
    
    參數:
        lat1, lon1: 第一個點的緯度和經度（度）
        lat2, lon2: 第二個點的緯度和經度（度）
    
    返回:
        距離（公尺）
    """
    # 計算平均緯度的經度縮放係數
    avg_lat = (lat1 + lat2) / 2
    cos_lat = math.cos(deg_to_rad(avg_lat))
    
    # 轉換為公尺
    dlat = (lat2 - lat1) * 111111.0
    dlon = (lon2 - lon1) * 111111.0 * cos_lat
    
    # 計算距離
    distance = math.sqrt(dlat * dlat + dlon * dlon)
    return distance


def euclidean_distance(x1: float, y1: float, 
                      x2: float, y2: float) -> float:
    """
    計算歐幾里得距離（笛卡爾座標系）
    
    參數:
        x1, y1: 第一個點的座標
        x2, y2: 第二個點的座標
    
    返回:
        距離
    """
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)


# ==========================================
# 方位角計算
# ==========================================
def bearing_between_points(lat1: float, lon1: float, 
                          lat2: float, lon2: float) -> float:
    """
    計算從點1到點2的方位角（真北為0度，順時針）
    
    參數:
        lat1, lon1: 起點的緯度和經度（度）
        lat2, lon2: 終點的緯度和經度（度）
    
    返回:
        方位角（度，0-360）
    """
    lat1_rad = deg_to_rad(lat1)
    lat2_rad = deg_to_rad(lat2)
    dlon_rad = deg_to_rad(lon2 - lon1)
    
    y = math.sin(dlon_rad) * math.cos(lat2_rad)
    x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
         math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad))
    
    bearing_rad = math.atan2(y, x)
    bearing_deg = rad_to_deg(bearing_rad)
    
    # 標準化到 0-360
    return normalize_angle(bearing_deg, 0.0)


def bearing_to_vector(bearing_deg: float) -> Tuple[float, float]:
    """
    將方位角轉換為單位向量
    
    參數:
        bearing_deg: 方位角（度）
    
    返回:
        (x, y): 單位向量
    """
    bearing_rad = deg_to_rad(bearing_deg)
    x = math.sin(bearing_rad)
    y = math.cos(bearing_rad)
    return (x, y)


def vector_to_bearing(x: float, y: float) -> float:
    """
    將向量轉換為方位角
    
    參數:
        x, y: 向量分量
    
    返回:
        方位角（度，0-360）
    """
    bearing_rad = math.atan2(x, y)
    bearing_deg = rad_to_deg(bearing_rad)
    return normalize_angle(bearing_deg, 0.0)


# ==========================================
# 座標計算
# ==========================================
def point_at_distance_bearing(lat: float, lon: float, 
                              distance_m: float, 
                              bearing_deg: float) -> Tuple[float, float]:
    """
    計算從給定點沿指定方位角和距離的新點座標
    
    參數:
        lat, lon: 起點的緯度和經度（度）
        distance_m: 距離（公尺）
        bearing_deg: 方位角（度）
    
    返回:
        (new_lat, new_lon): 新點的緯度和經度（度）
    """
    lat_rad = deg_to_rad(lat)
    bearing_rad = deg_to_rad(bearing_deg)
    
    # 角距離
    angular_distance = distance_m / EARTH_RADIUS_M
    
    # 計算新緯度
    new_lat_rad = math.asin(
        math.sin(lat_rad) * math.cos(angular_distance) +
        math.cos(lat_rad) * math.sin(angular_distance) * math.cos(bearing_rad)
    )
    
    # 計算新經度
    new_lon_rad = deg_to_rad(lon) + math.atan2(
        math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(lat_rad),
        math.cos(angular_distance) - math.sin(lat_rad) * math.sin(new_lat_rad)
    )
    
    new_lat = rad_to_deg(new_lat_rad)
    new_lon = rad_to_deg(new_lon_rad)
    
    return (new_lat, new_lon)


def midpoint(lat1: float, lon1: float, 
            lat2: float, lon2: float) -> Tuple[float, float]:
    """
    計算兩點之間的中點
    
    參數:
        lat1, lon1: 第一個點的緯度和經度（度）
        lat2, lon2: 第二個點的緯度和經度（度）
    
    返回:
        (mid_lat, mid_lon): 中點的緯度和經度（度）
    """
    lat1_rad = deg_to_rad(lat1)
    lat2_rad = deg_to_rad(lat2)
    dlon_rad = deg_to_rad(lon2 - lon1)
    
    bx = math.cos(lat2_rad) * math.cos(dlon_rad)
    by = math.cos(lat2_rad) * math.sin(dlon_rad)
    
    mid_lat_rad = math.atan2(
        math.sin(lat1_rad) + math.sin(lat2_rad),
        math.sqrt((math.cos(lat1_rad) + bx) ** 2 + by ** 2)
    )
    
    mid_lon_rad = deg_to_rad(lon1) + math.atan2(by, math.cos(lat1_rad) + bx)
    
    mid_lat = rad_to_deg(mid_lat_rad)
    mid_lon = rad_to_deg(mid_lon_rad)
    
    return (mid_lat, mid_lon)


# ==========================================
# 幾何變換
# ==========================================
def rotate_point(x: float, y: float, 
                angle_deg: float, 
                cx: float = 0.0, cy: float = 0.0) -> Tuple[float, float]:
    """
    繞指定中心點旋轉點座標
    
    參數:
        x, y: 點的座標
        angle_deg: 旋轉角度（度，逆時針為正）
        cx, cy: 旋轉中心（預設為原點）
    
    返回:
        (new_x, new_y): 旋轉後的座標
    """
    angle_rad = deg_to_rad(angle_deg)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    
    # 平移到原點
    dx = x - cx
    dy = y - cy
    
    # 旋轉
    new_x = cos_a * dx - sin_a * dy
    new_y = sin_a * dx + cos_a * dy
    
    # 平移回原位置
    new_x += cx
    new_y += cy
    
    return (new_x, new_y)


def translate_point(x: float, y: float, 
                   dx: float, dy: float) -> Tuple[float, float]:
    """
    平移點座標
    
    參數:
        x, y: 點的座標
        dx, dy: 平移量
    
    返回:
        (new_x, new_y): 平移後的座標
    """
    return (x + dx, y + dy)


def scale_point(x: float, y: float, 
               scale: float, 
               cx: float = 0.0, cy: float = 0.0) -> Tuple[float, float]:
    """
    縮放點座標
    
    參數:
        x, y: 點的座標
        scale: 縮放係數
        cx, cy: 縮放中心（預設為原點）
    
    返回:
        (new_x, new_y): 縮放後的座標
    """
    new_x = cx + (x - cx) * scale
    new_y = cy + (y - cy) * scale
    return (new_x, new_y)


# ==========================================
# 線段計算
# ==========================================
def line_intersection(x1: float, y1: float, x2: float, y2: float,
                     x3: float, y3: float, x4: float, y4: float) -> Optional[Tuple[float, float]]:
    """
    計算兩條線段的交點
    
    參數:
        x1, y1, x2, y2: 第一條線段的端點
        x3, y3, x4, y4: 第二條線段的端點
    
    返回:
        (x, y): 交點座標，如果沒有交點則返回 None
    """
    # 計算方向向量
    dx1 = x2 - x1
    dy1 = y2 - y1
    dx2 = x4 - x3
    dy2 = y4 - y3
    
    # 計算行列式
    det = dx1 * dy2 - dy1 * dx2
    
    if abs(det) < 1e-10:
        # 線段平行或共線
        return None
    
    # 計算參數 t 和 u
    t = ((x3 - x1) * dy2 - (y3 - y1) * dx2) / det
    u = ((x3 - x1) * dy1 - (y3 - y1) * dx1) / det
    
    # 檢查是否在線段範圍內
    if 0 <= t <= 1 and 0 <= u <= 1:
        x = x1 + t * dx1
        y = y1 + t * dy1
        return (x, y)
    
    return None


def point_to_line_distance(px: float, py: float,
                          x1: float, y1: float,
                          x2: float, y2: float) -> float:
    """
    計算點到線段的最短距離
    
    參數:
        px, py: 點的座標
        x1, y1, x2, y2: 線段的端點
    
    返回:
        最短距離
    """
    dx = x2 - x1
    dy = y2 - y1
    
    if dx == 0 and dy == 0:
        # 線段退化為點
        return euclidean_distance(px, py, x1, y1)
    
    # 計算投影參數
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    
    # 計算最近點
    nearest_x = x1 + t * dx
    nearest_y = y1 + t * dy
    
    # 返回距離
    return euclidean_distance(px, py, nearest_x, nearest_y)


def point_on_line_segment(px: float, py: float,
                         x1: float, y1: float,
                         x2: float, y2: float,
                         tolerance: float = 1e-6) -> bool:
    """
    判斷點是否在線段上
    
    參數:
        px, py: 點的座標
        x1, y1, x2, y2: 線段的端點
        tolerance: 容差值
    
    返回:
        是否在線段上
    """
    distance = point_to_line_distance(px, py, x1, y1, x2, y2)
    return distance < tolerance


# ==========================================
# 座標系轉換
# ==========================================
def latlon_to_meters(lat: float, lon: float, 
                    lat0: float, lon0: float) -> Tuple[float, float]:
    """
    將經緯度座標轉換為以參考點為原點的平面座標（公尺）
    
    參數:
        lat, lon: 目標點的緯度和經度（度）
        lat0, lon0: 參考點的緯度和經度（度）
    
    返回:
        (x, y): 平面座標（公尺）
    """
    cos_lat0 = math.cos(deg_to_rad(lat0))
    
    x = (lon - lon0) * 111111.0 * cos_lat0
    y = (lat - lat0) * 111111.0
    
    return (x, y)


def meters_to_latlon(x: float, y: float, 
                    lat0: float, lon0: float) -> Tuple[float, float]:
    """
    將平面座標（公尺）轉換回經緯度座標
    
    參數:
        x, y: 平面座標（公尺）
        lat0, lon0: 參考點的緯度和經度（度）
    
    返回:
        (lat, lon): 緯度和經度（度）
    """
    cos_lat0 = math.cos(deg_to_rad(lat0))
    
    lat = y / 111111.0 + lat0
    lon = x / (111111.0 * cos_lat0) + lon0
    
    return (lat, lon)


# ==========================================
# 多邊形計算
# ==========================================
def point_in_polygon(px: float, py: float, 
                    polygon: list) -> bool:
    """
    射線法判斷點是否在多邊形內
    
    參數:
        px, py: 點的座標
        polygon: 多邊形頂點列表 [(x1, y1), (x2, y2), ...]
    
    返回:
        是否在多邊形內
    """
    n = len(polygon)
    inside = False
    
    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        
        if py > min(p1y, p2y):
            if py <= max(p1y, p2y):
                if px <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (py - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or px <= xinters:
                        inside = not inside
        
        p1x, p1y = p2x, p2y
    
    return inside


def polygon_area(polygon: list) -> float:
    """
    計算多邊形面積（使用 Shoelace 公式）
    
    參數:
        polygon: 多邊形頂點列表 [(x1, y1), (x2, y2), ...]
    
    返回:
        面積（絕對值）
    """
    n = len(polygon)
    if n < 3:
        return 0.0
    
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += polygon[i][0] * polygon[j][1]
        area -= polygon[j][0] * polygon[i][1]
    
    return abs(area) / 2.0


def polygon_centroid(polygon: list) -> Tuple[float, float]:
    """
    計算多邊形質心
    
    參數:
        polygon: 多邊形頂點列表 [(x1, y1), (x2, y2), ...]
    
    返回:
        (cx, cy): 質心座標
    """
    n = len(polygon)
    if n == 0:
        return (0.0, 0.0)
    
    cx = sum(p[0] for p in polygon) / n
    cy = sum(p[1] for p in polygon) / n
    
    return (cx, cy)
