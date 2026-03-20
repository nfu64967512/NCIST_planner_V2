"""
座標轉換模組
提供 WGS84、UTM、本地座標系之間的轉換
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional
import math


@dataclass
class GeoPoint:
    """地理座標點"""
    latitude: float
    longitude: float
    altitude: float = 0.0
    
    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.latitude, self.longitude, self.altitude)
    
    def to_array(self) -> np.ndarray:
        return np.array([self.latitude, self.longitude, self.altitude])


@dataclass
class LocalPoint:
    """本地座標點 (ENU: East-North-Up)"""
    x: float  # East
    y: float  # North
    z: float = 0.0  # Up
    
    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])


class CoordinateTransformer:
    """
    座標轉換器
    
    支援：
    - WGS84 (經緯度) ↔ 本地 ENU 座標系
    - WGS84 ↔ UTM
    
    使用方法：
        ```python
        # 創建轉換器（以某點為原點）
        transformer = CoordinateTransformer(23.7027, 120.4193)
        
        # 轉換到本地座標
        local = transformer.geo_to_local(23.7030, 120.4200)
        # local = np.array([x, y])
        
        # 轉換回地理座標
        geo = transformer.local_to_geo(100.0, 50.0)
        # geo = GeoPoint(latitude, longitude)
        ```
    """
    
    # WGS84 橢球參數
    WGS84_A = 6378137.0           # 長半軸 (m)
    WGS84_F = 1 / 298.257223563   # 扁率
    WGS84_B = WGS84_A * (1 - WGS84_F)  # 短半軸
    WGS84_E2 = (WGS84_A**2 - WGS84_B**2) / WGS84_A**2  # 第一離心率平方
    
    def __init__(self, origin_lat: float, origin_lon: float, origin_alt: float = 0.0):
        """
        初始化座標轉換器
        
        Args:
            origin_lat: 原點緯度 (度)
            origin_lon: 原點經度 (度)
            origin_alt: 原點高度 (m)
        """
        self.origin = GeoPoint(origin_lat, origin_lon, origin_alt)
        
        # 預計算常用值
        self._origin_lat_rad = math.radians(origin_lat)
        self._origin_lon_rad = math.radians(origin_lon)
        self._cos_lat0 = math.cos(self._origin_lat_rad)
        self._sin_lat0 = math.sin(self._origin_lat_rad)
        
        # 計算在原點處的經緯度到公尺的轉換係數
        # 使用更精確的橢球模型
        self._meters_per_deg_lat = self._calculate_meters_per_deg_lat(origin_lat)
        self._meters_per_deg_lon = self._calculate_meters_per_deg_lon(origin_lat)
    
    def _calculate_meters_per_deg_lat(self, lat: float) -> float:
        """計算緯度方向每度的公尺數"""
        lat_rad = math.radians(lat)
        # 子午線曲率半徑
        m = self.WGS84_A * (1 - self.WGS84_E2) / (
            (1 - self.WGS84_E2 * math.sin(lat_rad)**2) ** 1.5
        )
        return math.radians(1) * m
    
    def _calculate_meters_per_deg_lon(self, lat: float) -> float:
        """計算經度方向每度的公尺數"""
        lat_rad = math.radians(lat)
        # 卯酉圈曲率半徑
        n = self.WGS84_A / math.sqrt(1 - self.WGS84_E2 * math.sin(lat_rad)**2)
        return math.radians(1) * n * math.cos(lat_rad)
    
    def geo_to_local(self, lat: float, lon: float, alt: float = 0.0) -> np.ndarray:
        """
        地理座標轉本地 ENU 座標
        
        Args:
            lat: 緯度 (度)
            lon: 經度 (度)
            alt: 高度 (m)
            
        Returns:
            本地座標 [x, y, z] (m)，x=East, y=North, z=Up
        """
        dlat = lat - self.origin.latitude
        dlon = lon - self.origin.longitude
        dalt = alt - self.origin.altitude
        
        # 轉換到公尺
        x = dlon * self._meters_per_deg_lon  # East
        y = dlat * self._meters_per_deg_lat  # North
        z = dalt                              # Up
        
        return np.array([x, y, z])
    
    def local_to_geo(self, x: float, y: float, z: float = 0.0) -> GeoPoint:
        """
        本地 ENU 座標轉地理座標
        
        Args:
            x: East (m)
            y: North (m)
            z: Up (m)
            
        Returns:
            GeoPoint 對象
        """
        # 轉換到度
        dlon = x / self._meters_per_deg_lon
        dlat = y / self._meters_per_deg_lat
        
        lat = self.origin.latitude + dlat
        lon = self.origin.longitude + dlon
        alt = self.origin.altitude + z
        
        return GeoPoint(lat, lon, alt)
    
    def geo_to_local_batch(self, points: np.ndarray) -> np.ndarray:
        """
        批次轉換地理座標到本地座標
        
        Args:
            points: 形狀為 (N, 2) 或 (N, 3) 的數組，每行 [lat, lon] 或 [lat, lon, alt]
            
        Returns:
            形狀為 (N, 3) 的本地座標數組
        """
        if points.ndim == 1:
            points = points.reshape(1, -1)
        
        n = points.shape[0]
        result = np.zeros((n, 3))
        
        for i in range(n):
            lat, lon = points[i, 0], points[i, 1]
            alt = points[i, 2] if points.shape[1] > 2 else 0.0
            result[i] = self.geo_to_local(lat, lon, alt)
        
        return result
    
    def local_to_geo_batch(self, points: np.ndarray) -> np.ndarray:
        """
        批次轉換本地座標到地理座標
        
        Args:
            points: 形狀為 (N, 2) 或 (N, 3) 的數組，每行 [x, y] 或 [x, y, z]
            
        Returns:
            形狀為 (N, 3) 的地理座標數組 [lat, lon, alt]
        """
        if points.ndim == 1:
            points = points.reshape(1, -1)
        
        n = points.shape[0]
        result = np.zeros((n, 3))
        
        for i in range(n):
            x, y = points[i, 0], points[i, 1]
            z = points[i, 2] if points.shape[1] > 2 else 0.0
            geo = self.local_to_geo(x, y, z)
            result[i] = geo.to_array()
        
        return result
    
    def calculate_distance(self, lat1: float, lon1: float,
                          lat2: float, lon2: float) -> float:
        """
        計算兩點間的距離（Haversine 公式）
        
        Args:
            lat1, lon1: 第一點的經緯度 (度)
            lat2, lon2: 第二點的經緯度 (度)
            
        Returns:
            距離 (m)
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = (math.sin(dlat / 2) ** 2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return self.WGS84_A * c
    
    def calculate_bearing(self, lat1: float, lon1: float,
                         lat2: float, lon2: float) -> float:
        """
        計算方位角
        
        Args:
            lat1, lon1: 起點經緯度 (度)
            lat2, lon2: 終點經緯度 (度)
            
        Returns:
            方位角 (度，0-360，0=北)
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)
        
        x = math.sin(dlon) * math.cos(lat2_rad)
        y = (math.cos(lat1_rad) * math.sin(lat2_rad) -
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon))
        
        bearing = math.degrees(math.atan2(x, y))
        return (bearing + 360) % 360
    
    def project_point(self, lat: float, lon: float,
                     bearing: float, distance: float) -> GeoPoint:
        """
        從一點沿指定方位角投影指定距離
        
        Args:
            lat, lon: 起點經緯度 (度)
            bearing: 方位角 (度)
            distance: 距離 (m)
            
        Returns:
            投影後的點
        """
        lat_rad = math.radians(lat)
        bearing_rad = math.radians(bearing)
        angular_dist = distance / self.WGS84_A
        
        new_lat_rad = math.asin(
            math.sin(lat_rad) * math.cos(angular_dist) +
            math.cos(lat_rad) * math.sin(angular_dist) * math.cos(bearing_rad)
        )
        
        new_lon_rad = math.radians(lon) + math.atan2(
            math.sin(bearing_rad) * math.sin(angular_dist) * math.cos(lat_rad),
            math.cos(angular_dist) - math.sin(lat_rad) * math.sin(new_lat_rad)
        )
        
        return GeoPoint(math.degrees(new_lat_rad), math.degrees(new_lon_rad))


class UTMConverter:
    """
    UTM 座標轉換器
    
    WGS84 經緯度 ↔ UTM 座標
    """
    
    # UTM 參數
    K0 = 0.9996  # 比例因子
    E = 0.00669438  # 第一離心率平方
    E2 = E * E
    E3 = E2 * E
    E_P2 = E / (1 - E)
    
    R = 6378137.0  # 赤道半徑
    
    def __init__(self):
        pass
    
    @staticmethod
    def get_zone_number(lat: float, lon: float) -> int:
        """獲取 UTM 帶號"""
        if 56 <= lat < 64 and 3 <= lon < 12:
            return 32
        if 72 <= lat <= 84:
            if 0 <= lon < 9:
                return 31
            elif 9 <= lon < 21:
                return 33
            elif 21 <= lon < 33:
                return 35
            elif 33 <= lon < 42:
                return 37
        
        return int((lon + 180) / 6) + 1
    
    @staticmethod
    def get_zone_letter(lat: float) -> str:
        """獲取 UTM 帶字母"""
        letters = 'CDEFGHJKLMNPQRSTUVWXX'
        if -80 <= lat <= 84:
            return letters[int((lat + 80) / 8)]
        return ''
    
    def geo_to_utm(self, lat: float, lon: float) -> Tuple[float, float, int, str]:
        """
        WGS84 轉 UTM
        
        Returns:
            (easting, northing, zone_number, zone_letter)
        """
        zone_number = self.get_zone_number(lat, lon)
        zone_letter = self.get_zone_letter(lat)
        
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        
        lon_origin = (zone_number - 1) * 6 - 180 + 3
        lon_origin_rad = math.radians(lon_origin)
        
        n = self.R / math.sqrt(1 - self.E * math.sin(lat_rad)**2)
        t = math.tan(lat_rad)**2
        c = self.E_P2 * math.cos(lat_rad)**2
        a = math.cos(lat_rad) * (lon_rad - lon_origin_rad)
        
        m = self.R * (
            (1 - self.E/4 - 3*self.E2/64 - 5*self.E3/256) * lat_rad
            - (3*self.E/8 + 3*self.E2/32 + 45*self.E3/1024) * math.sin(2*lat_rad)
            + (15*self.E2/256 + 45*self.E3/1024) * math.sin(4*lat_rad)
            - (35*self.E3/3072) * math.sin(6*lat_rad)
        )
        
        easting = self.K0 * n * (
            a + (1-t+c) * a**3/6
            + (5-18*t+t**2+72*c-58*self.E_P2) * a**5/120
        ) + 500000
        
        northing = self.K0 * (
            m + n * math.tan(lat_rad) * (
                a**2/2
                + (5-t+9*c+4*c**2) * a**4/24
                + (61-58*t+t**2+600*c-330*self.E_P2) * a**6/720
            )
        )
        
        if lat < 0:
            northing += 10000000
        
        return (easting, northing, zone_number, zone_letter)
    
    def utm_to_geo(self, easting: float, northing: float,
                  zone_number: int, northern: bool = True) -> GeoPoint:
        """
        UTM 轉 WGS84
        
        Args:
            easting: UTM 東向座標
            northing: UTM 北向座標
            zone_number: UTM 帶號
            northern: 是否在北半球
            
        Returns:
            GeoPoint 對象
        """
        if not northern:
            northing = 10000000 - northing
        
        x = easting - 500000
        y = northing
        
        m = y / self.K0
        mu = m / (self.R * (1 - self.E/4 - 3*self.E2/64 - 5*self.E3/256))
        
        e1 = (1 - math.sqrt(1 - self.E)) / (1 + math.sqrt(1 - self.E))
        
        phi1 = mu + (3*e1/2 - 27*e1**3/32) * math.sin(2*mu)
        phi1 += (21*e1**2/16 - 55*e1**4/32) * math.sin(4*mu)
        phi1 += (151*e1**3/96) * math.sin(6*mu)
        phi1 += (1097*e1**4/512) * math.sin(8*mu)
        
        n1 = self.R / math.sqrt(1 - self.E * math.sin(phi1)**2)
        t1 = math.tan(phi1)**2
        c1 = self.E_P2 * math.cos(phi1)**2
        r1 = self.R * (1 - self.E) / (1 - self.E * math.sin(phi1)**2)**1.5
        d = x / (n1 * self.K0)
        
        lat = phi1 - (n1 * math.tan(phi1) / r1) * (
            d**2/2
            - (5+3*t1+10*c1-4*c1**2-9*self.E_P2) * d**4/24
            + (61+90*t1+298*c1+45*t1**2-252*self.E_P2-3*c1**2) * d**6/720
        )
        
        lon = (
            d - (1+2*t1+c1) * d**3/6
            + (5-2*c1+28*t1-3*c1**2+8*self.E_P2+24*t1**2) * d**5/120
        ) / math.cos(phi1)
        
        lon_origin = (zone_number - 1) * 6 - 180 + 3
        
        lat = math.degrees(lat)
        lon = math.degrees(lon) + lon_origin
        
        if not northern:
            lat = -lat
        
        return GeoPoint(lat, lon)
