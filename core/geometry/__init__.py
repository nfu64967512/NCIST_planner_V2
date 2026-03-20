"""
Geometry 幾何運算模組
"""

import math
import numpy as np

from .coordinate import (
    CoordinateTransformer,
    GeoPoint,
    UTMConverter
)

from .polygon import (
    PolygonUtils
)

from .region_divider import RegionDivider

# 為了向後相容，提供包裝類別
class CoordinateTransform:
    """
    座標轉換器包裝類別

    提供與 coverage_planner 相容的介面
    """

    def __init__(self, center_lat: float, center_lon: float):
        """初始化座標轉換器"""
        self.transformer = CoordinateTransformer(center_lat, center_lon)
        self.center_lat = center_lat
        self.center_lon = center_lon

    def latlon_to_xy(self, lat: float, lon: float) -> tuple:
        """經緯度轉本地 XY 座標"""
        local = self.transformer.geo_to_local(lat, lon)
        return (local[0], local[1])

    def xy_to_latlon(self, x: float, y: float) -> tuple:
        """本地 XY 座標轉經緯度"""
        geo = self.transformer.local_to_geo(x, y)
        return (geo.latitude, geo.longitude)

    def batch_latlon_to_xy(self, points: list) -> list:
        """批次轉換經緯度到本地 XY"""
        return [self.latlon_to_xy(lat, lon) for lat, lon in points]

    def batch_xy_to_latlon(self, points: list) -> list:
        """批次轉換本地 XY 到經緯度"""
        return [self.xy_to_latlon(x, y) for x, y in points]


class RotatedCoordinateSystem:
    """
    旋轉座標系

    用於在指定角度的旋轉座標系中進行轉換
    """

    def __init__(self, center_lat: float, center_lon: float, angle_deg: float):
        """
        初始化旋轉座標系

        參數:
            center_lat: 中心點緯度
            center_lon: 中心點經度
            angle_deg: 旋轉角度（度）
        """
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.angle_deg = angle_deg
        self.angle_rad = math.radians(angle_deg)

        # 旋轉矩陣
        cos_a = math.cos(self.angle_rad)
        sin_a = math.sin(self.angle_rad)
        self.rotation_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
        self.inverse_rotation = np.array([[cos_a, sin_a], [-sin_a, cos_a]])

        # 座標轉換器
        self.transformer = CoordinateTransformer(center_lat, center_lon)

    def latlon_to_xy(self, lat: float, lon: float) -> tuple:
        """
        經緯度轉換到旋轉的 XY 座標

        返回:
            (x, y) 旋轉後的座標（公尺）
        """
        # 先轉換到本地座標
        local = self.transformer.geo_to_local(lat, lon)
        xy = np.array([local[0], local[1]])

        # 旋轉
        rotated = self.rotation_matrix @ xy
        return (rotated[0], rotated[1])

    def xy_to_latlon(self, x: float, y: float) -> tuple:
        """
        旋轉的 XY 座標轉換回經緯度

        返回:
            (lat, lon)
        """
        # 逆旋轉
        xy = np.array([x, y])
        unrotated = self.inverse_rotation @ xy

        # 轉換回經緯度
        geo = self.transformer.local_to_geo(unrotated[0], unrotated[1])
        return (geo.latitude, geo.longitude)

    def batch_latlon_to_xy(self, points: list) -> list:
        """
        批次轉換經緯度到旋轉 XY

        參數:
            points: [(lat, lon), ...] 列表

        返回:
            [(x, y), ...] 列表
        """
        return [self.latlon_to_xy(lat, lon) for lat, lon in points]

    def batch_xy_to_latlon(self, points: list) -> list:
        """
        批次轉換旋轉 XY 到經緯度

        參數:
            points: [(x, y), ...] 列表

        返回:
            [(lat, lon), ...] 列表
        """
        return [self.xy_to_latlon(x, y) for x, y in points]


__all__ = [
    'CoordinateTransformer',
    'CoordinateTransform',
    'RotatedCoordinateSystem',
    'GeoPoint',
    'UTMConverter',
    'PolygonUtils',
    'RegionDivider',
]
