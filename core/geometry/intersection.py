"""
交點計算模組
提供線段、圓形、多邊形的交點計算功能
"""

import math
from typing import List, Tuple, Optional


# ==========================================
# 線段-線段交點
# ==========================================
def segment_segment_intersection(
    p1: Tuple[float, float], p2: Tuple[float, float],
    p3: Tuple[float, float], p4: Tuple[float, float],
    check_bounds: bool = True
) -> Optional[Tuple[float, float]]:
    """
    計算兩條線段的交點
    
    參數:
        p1, p2: 第一條線段的端點
        p3, p4: 第二條線段的端點
        check_bounds: 是否檢查交點是否在線段範圍內
    
    返回:
        交點座標，如果沒有交點則返回 None
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    
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
    if check_bounds:
        if not (0 <= t <= 1 and 0 <= u <= 1):
            return None
    
    # 計算交點
    x = x1 + t * dx1
    y = y1 + t * dy1
    
    return (x, y)


def line_line_intersection(
    p1: Tuple[float, float], p2: Tuple[float, float],
    p3: Tuple[float, float], p4: Tuple[float, float]
) -> Optional[Tuple[float, float]]:
    """
    計算兩條直線的交點（不限制在線段範圍內）
    
    參數:
        p1, p2: 第一條直線上的兩個點
        p3, p4: 第二條直線上的兩個點
    
    返回:
        交點座標，如果平行則返回 None
    """
    return segment_segment_intersection(p1, p2, p3, p4, check_bounds=False)


# ==========================================
# 線段-圓形交點
# ==========================================
def line_circle_intersection(
    p1: Tuple[float, float], p2: Tuple[float, float],
    center: Tuple[float, float], radius: float,
    segment: bool = True
) -> List[Tuple[float, float]]:
    """
    計算線段（或直線）與圓的交點
    
    參數:
        p1, p2: 線段（或直線）的端點
        center: 圓心座標
        radius: 圓半徑
        segment: 是否限制為線段（True）還是直線（False）
    
    返回:
        交點列表（0個、1個或2個交點）
    """
    x1, y1 = p1
    x2, y2 = p2
    cx, cy = center
    
    # 將圓心平移到原點
    x1 -= cx
    y1 -= cy
    x2 -= cx
    y2 -= cy
    
    # 線段方向向量
    dx = x2 - x1
    dy = y2 - y1
    
    # 參數方程: (x, y) = (x1, y1) + t * (dx, dy)
    # 代入圓方程: (x1 + t*dx)^2 + (y1 + t*dy)^2 = r^2
    # 得到: a*t^2 + b*t + c = 0
    a = dx * dx + dy * dy
    b = 2 * (x1 * dx + y1 * dy)
    c = x1 * x1 + y1 * y1 - radius * radius
    
    if abs(a) < 1e-10:
        # 線段退化為點
        if abs(c) < 1e-10:
            # 點在圓上
            return [(p1[0], p1[1])]
        return []
    
    # 判別式
    discriminant = b * b - 4 * a * c
    
    if discriminant < 0:
        # 無交點
        return []
    
    # 計算交點參數
    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / (2 * a)
    t2 = (-b + sqrt_disc) / (2 * a)
    
    # 計算交點座標
    intersections = []
    
    for t in [t1, t2]:
        if not segment or (0 <= t <= 1):
            x = x1 + t * dx + cx
            y = y1 + t * dy + cy
            intersections.append((x, y))
    
    return intersections


# ==========================================
# 線段-多邊形交點
# ==========================================
def line_polygon_intersection(
    p1: Tuple[float, float], p2: Tuple[float, float],
    polygon: List[Tuple[float, float]]
) -> List[Tuple[float, float]]:
    """
    計算線段與多邊形的交點
    
    參數:
        p1, p2: 線段的端點
        polygon: 多邊形頂點列表
    
    返回:
        交點列表（按沿線段方向排序）
    """
    n = len(polygon)
    if n < 3:
        return []
    
    intersections = []
    seen_points = set()
    
    # 檢查每條多邊形邊
    for i in range(n):
        p3 = polygon[i]
        p4 = polygon[(i + 1) % n]
        
        # 計算交點
        intersection = segment_segment_intersection(p1, p2, p3, p4)
        
        if intersection is not None:
            # 避免重複點（多邊形頂點）
            point_key = (round(intersection[0], 6), round(intersection[1], 6))
            if point_key not in seen_points:
                seen_points.add(point_key)
                intersections.append(intersection)
    
    # 按沿線段方向排序
    if len(intersections) > 1:
        x1, y1 = p1
        intersections.sort(key=lambda p: (p[0] - x1) ** 2 + (p[1] - y1) ** 2)
    
    return intersections


def horizontal_line_polygon_intersection(
    y: float, polygon: List[Tuple[float, float]]
) -> List[float]:
    """
    計算水平線與多邊形的交點 X 座標
    
    參數:
        y: 水平線的 Y 座標
        polygon: 多邊形頂點列表
    
    返回:
        交點的 X 座標列表（已排序）
    """
    n = len(polygon)
    if n < 3:
        return []
    
    x_coords = []
    
    # 檢查每條多邊形邊
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        
        # 檢查線段是否與水平線相交
        if (y1 <= y <= y2) or (y2 <= y <= y1):
            if abs(y2 - y1) > 1e-10:
                # 計算交點 X 座標
                t = (y - y1) / (y2 - y1)
                x = x1 + t * (x2 - x1)
                x_coords.append(x)
    
    # 排序並去重
    x_coords = sorted(list(set([round(x, 6) for x in x_coords])))
    
    return x_coords


# ==========================================
# 圓形-圓形交點
# ==========================================
def circle_circle_intersection(
    center1: Tuple[float, float], radius1: float,
    center2: Tuple[float, float], radius2: float
) -> List[Tuple[float, float]]:
    """
    計算兩個圓的交點
    
    參數:
        center1: 第一個圓的圓心
        radius1: 第一個圓的半徑
        center2: 第二個圓的圓心
        radius2: 第二個圓的半徑
    
    返回:
        交點列表（0個、1個或2個交點）
    """
    x1, y1 = center1
    x2, y2 = center2
    
    # 計算圓心距離
    d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    # 判斷是否有交點
    if d > radius1 + radius2:
        # 圓相離
        return []
    
    if d < abs(radius1 - radius2):
        # 一個圓在另一個圓內
        return []
    
    if d == 0 and radius1 == radius2:
        # 兩圓重合（無限多交點）
        return []
    
    # 計算交點
    a = (radius1 ** 2 - radius2 ** 2 + d ** 2) / (2 * d)
    h = math.sqrt(radius1 ** 2 - a ** 2)
    
    # 計算中間點
    px = x1 + a * (x2 - x1) / d
    py = y1 + a * (y2 - y1) / d
    
    # 計算兩個交點
    ix1 = px + h * (y2 - y1) / d
    iy1 = py - h * (x2 - x1) / d
    
    ix2 = px - h * (y2 - y1) / d
    iy2 = py + h * (x2 - x1) / d
    
    if abs(h) < 1e-10:
        # 兩圓相切（一個交點）
        return [(px, py)]
    else:
        # 兩個交點
        return [(ix1, iy1), (ix2, iy2)]


# ==========================================
# 點到線段最近點
# ==========================================
def closest_point_on_segment(
    point: Tuple[float, float],
    p1: Tuple[float, float],
    p2: Tuple[float, float]
) -> Tuple[float, float]:
    """
    計算點到線段的最近點
    
    參數:
        point: 查詢點
        p1, p2: 線段端點
    
    返回:
        線段上距離查詢點最近的點
    """
    px, py = point
    x1, y1 = p1
    x2, y2 = p2
    
    dx = x2 - x1
    dy = y2 - y1
    
    if abs(dx) < 1e-10 and abs(dy) < 1e-10:
        # 線段退化為點
        return p1
    
    # 計算投影參數
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    
    # 計算最近點
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    
    return (closest_x, closest_y)


# ==========================================
# 點到線段距離
# ==========================================
def point_to_segment_distance(
    point: Tuple[float, float],
    p1: Tuple[float, float],
    p2: Tuple[float, float]
) -> float:
    """
    計算點到線段的最短距離
    
    參數:
        point: 查詢點
        p1, p2: 線段端點
    
    返回:
        最短距離
    """
    closest = closest_point_on_segment(point, p1, p2)
    px, py = point
    cx, cy = closest
    
    return math.sqrt((px - cx) ** 2 + (py - cy) ** 2)


# ==========================================
# 線段與圓相交判定
# ==========================================
def segment_intersects_circle(
    p1: Tuple[float, float], p2: Tuple[float, float],
    center: Tuple[float, float], radius: float
) -> bool:
    """
    判斷線段是否與圓相交
    
    參數:
        p1, p2: 線段端點
        center: 圓心
        radius: 圓半徑
    
    返回:
        是否相交
    """
    distance = point_to_segment_distance(center, p1, p2)
    return distance <= radius


# ==========================================
# 多邊形與圓相交判定
# ==========================================
def polygon_intersects_circle(
    polygon: List[Tuple[float, float]],
    center: Tuple[float, float],
    radius: float
) -> bool:
    """
    判斷多邊形是否與圓相交
    
    參數:
        polygon: 多邊形頂點列表
        center: 圓心
        radius: 圓半徑
    
    返回:
        是否相交
    """
    n = len(polygon)
    if n < 3:
        return False
    
    # 檢查圓心是否在多邊形內
    from utils.math_utils import point_in_polygon
    if point_in_polygon(center[0], center[1], polygon):
        return True
    
    # 檢查任一多邊形邊是否與圓相交
    for i in range(n):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % n]
        
        if segment_intersects_circle(p1, p2, center, radius):
            return True
    
    return False
