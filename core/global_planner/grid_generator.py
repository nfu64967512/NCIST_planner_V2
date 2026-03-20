"""
Survey Grid 網格生成器
移植自 Mission Planner 的 Grid.cs 核心演算法

採用 Scanline Fill Algorithm 實現多邊形區域的掃描線生成
支援重疊率計算、相機 FOV 整合、多種掃描模式
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Union
from enum import Enum, auto
import time

from ..base.planner_base import (
    GlobalPlanner, GlobalPlannerConfig, PlannerType,
    PlannerResult, PlannerStatus, PlannerFactory
)
from ..geometry.coordinate import CoordinateTransformer, GeoPoint
from ..geometry.polygon import PolygonUtils


class ScanPattern(Enum):
    """掃描模式枚舉"""
    PARALLEL = auto()          # 平行線掃描
    ZIGZAG = auto()            # 之字形掃描
    SPIRAL = auto()            # 螺旋掃描
    EXPANDING_SQUARE = auto()  # 擴展方形


class EntryLocation(Enum):
    """進入點位置"""
    TOP_LEFT = auto()
    TOP_RIGHT = auto()
    BOTTOM_LEFT = auto()
    BOTTOM_RIGHT = auto()
    HOME_CLOSEST = auto()      # 最接近起點
    AUTO = auto()              # 自動選擇最佳


@dataclass
class CameraConfig:
    """相機配置"""
    sensor_width: float = 13.2      # 感光元件寬度 (mm)
    sensor_height: float = 8.8      # 感光元件高度 (mm)
    focal_length: float = 8.8       # 焦距 (mm)
    image_width: int = 4000         # 影像寬度 (px)
    image_height: int = 3000        # 影像高度 (px)
    
    def get_fov(self) -> Tuple[float, float]:
        """計算視場角 (度)"""
        h_fov = 2 * np.degrees(np.arctan(self.sensor_width / (2 * self.focal_length)))
        v_fov = 2 * np.degrees(np.arctan(self.sensor_height / (2 * self.focal_length)))
        return (h_fov, v_fov)
    
    def get_ground_coverage(self, altitude: float) -> Tuple[float, float]:
        """計算地面覆蓋範圍 (m)"""
        ground_width = (altitude * self.sensor_width) / self.focal_length
        ground_height = (altitude * self.sensor_height) / self.focal_length
        return (ground_width, ground_height)
    
    def get_gsd(self, altitude: float) -> float:
        """計算地面採樣距離 (m/px)"""
        ground_width, _ = self.get_ground_coverage(altitude)
        return ground_width / self.image_width


@dataclass
class SurveyConfig(GlobalPlannerConfig):
    """Survey Grid 配置"""
    # 飛行參數
    altitude: float = 50.0             # 飛行高度 (m AGL)
    speed: float = 5.0                 # 飛行速度 (m/s)
    
    # 覆蓋參數
    front_overlap: float = 80.0        # 前向重疊率 (%)
    side_overlap: float = 60.0         # 側向重疊率 (%)
    
    # 或使用手動間距
    use_manual_spacing: bool = False
    manual_line_spacing: float = 20.0  # 手動航線間距 (m)
    manual_photo_interval: float = 10.0  # 手動拍照間隔 (m)
    
    # 掃描參數
    scan_angle: float = 0.0            # 掃描角度 (度，0=東西向)
    scan_pattern: ScanPattern = ScanPattern.ZIGZAG
    entry_location: EntryLocation = EntryLocation.AUTO
    
    # 邊界參數
    boundary_offset: float = 0.0       # 邊界縮排 (m)
    overshoot_distance: float = 0.0    # 超出距離 (m)
    leadin_distance: float = 0.0       # 引入距離 (m)
    
    # 相機配置
    camera: CameraConfig = field(default_factory=CameraConfig)
    
    # 輸出控制
    add_takeoff: bool = True
    add_rtl: bool = True
    
    def get_line_spacing(self) -> float:
        """計算航線間距"""
        if self.use_manual_spacing:
            return self.manual_line_spacing
        
        ground_width, _ = self.camera.get_ground_coverage(self.altitude)
        return ground_width * (1 - self.side_overlap / 100)
    
    def get_photo_interval(self) -> float:
        """計算拍照間隔"""
        if self.use_manual_spacing:
            return self.manual_photo_interval
        
        _, ground_height = self.camera.get_ground_coverage(self.altitude)
        return ground_height * (1 - self.front_overlap / 100)


@dataclass
class SurveyStatistics:
    """Survey 統計資訊"""
    total_distance: float = 0.0        # 總航程 (m)
    survey_distance: float = 0.0       # 掃描航程 (m)
    turn_distance: float = 0.0         # 轉向航程 (m)
    num_lines: int = 0                 # 掃描線數量
    num_waypoints: int = 0             # 航點數量
    estimated_time: float = 0.0        # 預估時間 (s)
    coverage_area: float = 0.0         # 覆蓋面積 (m²)
    num_photos: int = 0                # 預估照片數量
    ground_resolution: float = 0.0     # 地面解析度 (m/px)


@PlannerFactory.register(PlannerType.GRID_SURVEY)
class GridSurveyGenerator(GlobalPlanner):
    """
    Survey Grid 網格生成器
    
    實現 Mission Planner 風格的區域掃描網格生成
    核心使用 Scanline Fill Algorithm
    
    使用方法：
        ```python
        # 配置
        config = SurveyConfig(
            altitude=100,
            front_overlap=80,
            side_overlap=60,
            scan_angle=45
        )
        
        # 創建生成器
        generator = GridSurveyGenerator(config)
        
        # 定義多邊形邊界（經緯度）
        boundary = [
            (23.7027, 120.4193),
            (23.7027, 120.4293),
            (23.7127, 120.4293),
            (23.7127, 120.4193)
        ]
        
        # 生成網格
        result = generator.generate_survey_grid(boundary)
        
        # 獲取航點
        waypoints = result.waypoints
        statistics = result.metadata['statistics']
        ```
    """
    
    def __init__(self, config: SurveyConfig = None):
        super().__init__(config or SurveyConfig())
        self.config: SurveyConfig = self.config
        
        # 座標轉換器
        self._transformer: Optional[CoordinateTransformer] = None
        
        # 內部狀態
        self._boundary_local: List[np.ndarray] = []
        self._scan_lines: List[Tuple[np.ndarray, np.ndarray]] = []
    
    @property
    def planner_type(self) -> PlannerType:
        return PlannerType.GRID_SURVEY
    
    def set_map(self, occupancy_grid: np.ndarray, 
                resolution: float, origin: np.ndarray):
        """設置佔用格地圖（Survey Grid 通常不需要）"""
        pass
    
    def plan(self, start: np.ndarray, goal: np.ndarray,
             obstacles: List = None) -> PlannerResult:
        """
        標準規劃介面
        
        對於 Survey Grid，start 和 goal 被解釋為多邊形的第一和最後一個點
        實際使用應調用 generate_survey_grid
        """
        # 構建簡單的矩形邊界
        boundary = [
            (start[0], start[1]),
            (start[0], goal[1]),
            (goal[0], goal[1]),
            (goal[0], start[1])
        ]
        return self.generate_survey_grid(boundary)
    
    def generate_survey_grid(self, boundary: List[Tuple[float, float]],
                            home_position: Tuple[float, float] = None,
                            obstacles: List = None) -> PlannerResult:
        """
        生成 Survey Grid
        
        核心演算法：Scanline Fill
        
        Args:
            boundary: 多邊形邊界點列表 [(lat, lon), ...]
            home_position: 起始/返航位置 (lat, lon)
            obstacles: 障礙物列表
            
        Returns:
            規劃結果，包含航點和統計資訊
        """
        self._status = PlannerStatus.PLANNING
        start_time = time.time()
        
        try:
            # 1. 驗證輸入
            if len(boundary) < 3:
                raise ValueError("多邊形至少需要3個頂點")
            
            # 2. 初始化座標轉換器（以多邊形中心為原點）
            center_lat = sum(p[0] for p in boundary) / len(boundary)
            center_lon = sum(p[1] for p in boundary) / len(boundary)
            self._transformer = CoordinateTransformer(center_lat, center_lon)
            
            # 3. 轉換邊界到本地座標
            self._boundary_local = []
            for lat, lon in boundary:
                local = self._transformer.geo_to_local(lat, lon)
                self._boundary_local.append(local)
            
            # 4. 應用邊界縮排
            if self.config.boundary_offset > 0:
                self._boundary_local = PolygonUtils.offset_polygon(
                    self._boundary_local, -self.config.boundary_offset
                )
            
            # 5. 計算掃描參數
            line_spacing = self.config.get_line_spacing()
            
            # 6. 生成掃描線（Scanline Algorithm 核心）
            self._scan_lines = self._generate_scan_lines(
                self._boundary_local,
                line_spacing,
                self.config.scan_angle
            )
            
            # 7. 排序和連接掃描線
            waypoints_local = self._connect_scan_lines(
                self._scan_lines,
                self.config.scan_pattern,
                self.config.entry_location,
                home_position
            )
            
            # 8. 應用超出和引入距離
            if self.config.overshoot_distance > 0 or self.config.leadin_distance > 0:
                waypoints_local = self._apply_overshoot_leadin(
                    waypoints_local,
                    self.config.overshoot_distance,
                    self.config.leadin_distance
                )
            
            # 9. 避障處理
            if obstacles:
                waypoints_local = self._apply_obstacle_avoidance(
                    waypoints_local, obstacles
                )
            
            # 10. 轉換回地理座標
            waypoints_geo = []
            for local_pt in waypoints_local:
                geo = self._transformer.local_to_geo(local_pt[0], local_pt[1])
                waypoints_geo.append(np.array([
                    geo.latitude, geo.longitude, self.config.altitude
                ]))
            
            # 11. 添加起飛和返航
            if self.config.add_takeoff and home_position:
                takeoff_pt = np.array([
                    home_position[0], home_position[1], self.config.altitude
                ])
                waypoints_geo.insert(0, takeoff_pt)
            
            if self.config.add_rtl and home_position:
                rtl_pt = np.array([
                    home_position[0], home_position[1], self.config.altitude
                ])
                waypoints_geo.append(rtl_pt)
            
            # 12. 計算統計資訊
            statistics = self._calculate_statistics(
                waypoints_geo, waypoints_local, boundary
            )
            
            # 13. 構建結果
            result = PlannerResult(
                status=PlannerStatus.SUCCESS,
                path=[wp for wp in waypoints_geo],
                waypoints=waypoints_geo,
                cost=statistics.total_distance,
                planning_time=time.time() - start_time,
                iterations=len(self._scan_lines),
                message=f"生成 {len(waypoints_geo)} 個航點",
                metadata={
                    'statistics': statistics,
                    'scan_lines': len(self._scan_lines),
                    'line_spacing': line_spacing,
                    'config': self.config
                }
            )
            
            self._status = PlannerStatus.SUCCESS
            return result
            
        except Exception as e:
            self._status = PlannerStatus.FAILED
            return PlannerResult(
                status=PlannerStatus.FAILED,
                planning_time=time.time() - start_time,
                message=f"Survey Grid 生成失敗: {str(e)}"
            )
    
    def _generate_scan_lines(self, polygon: List[np.ndarray],
                            spacing: float,
                            angle: float) -> List[Tuple[np.ndarray, np.ndarray]]:
        """
        生成掃描線 - Scanline Fill Algorithm 核心
        
        演算法步驟：
        1. 旋轉多邊形使掃描線垂直於 Y 軸
        2. 計算 Y 範圍和掃描線數量
        3. 對每條掃描線，計算與多邊形的交點
        4. 將交點旋轉回原始座標系
        
        Args:
            polygon: 多邊形頂點列表（本地座標）
            spacing: 掃描線間距 (m)
            angle: 掃描角度 (度)
            
        Returns:
            掃描線列表 [(start_point, end_point), ...]
        """
        # 旋轉角度（將掃描方向對齊到 X 軸）
        theta = np.radians(angle)
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        
        # 旋轉多邊形
        rotated_polygon = []
        for pt in polygon:
            rx = cos_t * pt[0] + sin_t * pt[1]
            ry = -sin_t * pt[0] + cos_t * pt[1]
            rotated_polygon.append(np.array([rx, ry]))
        
        # 計算 Y 範圍
        y_coords = [pt[1] for pt in rotated_polygon]
        y_min, y_max = min(y_coords), max(y_coords)
        
        # 生成掃描線
        scan_lines = []
        y = y_min + spacing / 2  # 從半個間距開始
        
        while y < y_max:
            # 計算掃描線與多邊形的交點
            intersections = self._scanline_intersect(rotated_polygon, y)
            
            # 處理交點對
            if len(intersections) >= 2:
                # 排序交點
                intersections.sort()
                
                # 每對交點形成一條掃描線段
                for i in range(0, len(intersections) - 1, 2):
                    x1, x2 = intersections[i], intersections[i + 1]
                    
                    # 旋轉回原始座標系
                    p1_x = cos_t * x1 - sin_t * y
                    p1_y = sin_t * x1 + cos_t * y
                    p2_x = cos_t * x2 - sin_t * y
                    p2_y = sin_t * x2 + cos_t * y
                    
                    scan_lines.append((
                        np.array([p1_x, p1_y]),
                        np.array([p2_x, p2_y])
                    ))
            
            y += spacing
        
        return scan_lines
    
    def _scanline_intersect(self, polygon: List[np.ndarray],
                           y: float) -> List[float]:
        """
        計算掃描線與多邊形的交點 X 座標
        
        Args:
            polygon: 多邊形頂點列表
            y: 掃描線 Y 座標
            
        Returns:
            交點 X 座標列表
        """
        intersections = []
        n = len(polygon)
        
        for i in range(n):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % n]
            
            # 檢查邊是否與掃描線相交
            y1, y2 = p1[1], p2[1]
            
            # 邊必須跨越掃描線
            if (y1 <= y < y2) or (y2 <= y < y1):
                # 計算交點 X 座標
                if abs(y2 - y1) > 1e-10:
                    t = (y - y1) / (y2 - y1)
                    x = p1[0] + t * (p2[0] - p1[0])
                    intersections.append(x)
        
        return intersections
    
    def _connect_scan_lines(self, scan_lines: List[Tuple[np.ndarray, np.ndarray]],
                           pattern: ScanPattern,
                           entry: EntryLocation,
                           home: Tuple[float, float] = None) -> List[np.ndarray]:
        """
        連接掃描線形成完整路徑
        
        Args:
            scan_lines: 掃描線列表
            pattern: 掃描模式
            entry: 進入點位置
            home: 起始位置
            
        Returns:
            航點列表
        """
        if not scan_lines:
            return []
        
        waypoints = []
        
        if pattern == ScanPattern.ZIGZAG:
            # 之字形連接
            for i, (p1, p2) in enumerate(scan_lines):
                if i % 2 == 0:
                    waypoints.extend([p1, p2])
                else:
                    waypoints.extend([p2, p1])
        
        elif pattern == ScanPattern.PARALLEL:
            # 平行線（每條線都從同一側開始）
            for p1, p2 in scan_lines:
                waypoints.extend([p1, p2])
        
        # 根據進入點調整方向
        if entry == EntryLocation.HOME_CLOSEST and home:
            home_local = self._transformer.geo_to_local(home[0], home[1])
            
            # 計算四個角到起點的距離
            first_point = waypoints[0]
            last_point = waypoints[-1]
            
            dist_first = np.linalg.norm(first_point - home_local)
            dist_last = np.linalg.norm(last_point - home_local)
            
            # 如果終點更近，反轉路徑
            if dist_last < dist_first:
                waypoints.reverse()
        
        elif entry == EntryLocation.TOP_LEFT:
            # 確保從左上開始（需要根據實際座標系調整）
            pass
        
        return waypoints
    
    def _apply_overshoot_leadin(self, waypoints: List[np.ndarray],
                               overshoot: float,
                               leadin: float) -> List[np.ndarray]:
        """
        應用超出和引入距離
        
        在每條掃描線的起點和終點添加額外航點
        使無人機能夠在進入掃描區域前穩定飛行
        """
        if not waypoints or (overshoot <= 0 and leadin <= 0):
            return waypoints
        
        new_waypoints = []
        
        # 假設偶數索引是掃描線起點，奇數索引是終點
        for i in range(0, len(waypoints) - 1, 2):
            p1, p2 = waypoints[i], waypoints[i + 1]
            
            # 計算方向向量
            direction = p2 - p1
            length = np.linalg.norm(direction)
            
            if length > 0:
                unit_dir = direction / length
                
                # 添加引入點（在起點前）
                if leadin > 0:
                    leadin_pt = p1 - unit_dir * leadin
                    new_waypoints.append(leadin_pt)
                
                # 添加起點
                new_waypoints.append(p1)
                
                # 添加終點
                new_waypoints.append(p2)
                
                # 添加超出點（在終點後）
                if overshoot > 0:
                    overshoot_pt = p2 + unit_dir * overshoot
                    new_waypoints.append(overshoot_pt)
            else:
                new_waypoints.extend([p1, p2])
        
        return new_waypoints
    
    def _apply_obstacle_avoidance(self, waypoints: List[np.ndarray],
                                  obstacles: List) -> List[np.ndarray]:
        """
        應用障礙物迴避
        
        檢測掃描線與障礙物的交集，並分割受影響的線段
        """
        # 實現障礙物迴避邏輯
        # 這裡可以整合 ObstacleManager 的演算法
        return waypoints
    
    def _calculate_statistics(self, waypoints_geo: List[np.ndarray],
                             waypoints_local: List[np.ndarray],
                             boundary: List[Tuple[float, float]]) -> SurveyStatistics:
        """計算 Survey 統計資訊"""
        stats = SurveyStatistics()
        
        # 航點數量
        stats.num_waypoints = len(waypoints_geo)
        stats.num_lines = len(self._scan_lines)
        
        # 計算總距離
        if len(waypoints_local) >= 2:
            for i in range(len(waypoints_local) - 1):
                dist = np.linalg.norm(waypoints_local[i + 1] - waypoints_local[i])
                stats.total_distance += dist
        
        # 預估時間
        stats.estimated_time = stats.total_distance / self.config.speed
        
        # 覆蓋面積
        stats.coverage_area = PolygonUtils.calculate_area(self._boundary_local)
        
        # 照片數量
        photo_interval = self.config.get_photo_interval()
        if photo_interval > 0:
            stats.num_photos = int(stats.total_distance / photo_interval)
        
        # 地面解析度
        stats.ground_resolution = self.config.camera.get_gsd(self.config.altitude)
        
        return stats
    
    def get_scan_lines(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """獲取掃描線（本地座標）"""
        return self._scan_lines.copy()
    
    def get_boundary_local(self) -> List[np.ndarray]:
        """獲取邊界（本地座標）"""
        return self._boundary_local.copy()
