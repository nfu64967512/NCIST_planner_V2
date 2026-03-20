"""
覆蓋路徑規劃器
用於生成完全覆蓋指定區域的掃描路徑
支持網格掃描、螺旋掃描、同心圓擴張上升掃描等多種模式
"""

import math
from enum import Enum
from typing import List, Tuple, Optional
from dataclasses import dataclass, field, replace

from ..geometry import RotatedCoordinateSystem, CoordinateTransform
from ..collision import CollisionChecker


class ScanPattern(Enum):
    """掃描模式"""
    GRID = "grid"                    # 網格掃描（之字形）
    SPIRAL = "spiral"                # 螺旋掃描（阿基米德螺線）
    CIRCULAR = "circular"            # 同心圓擴張上升掃描（Circle Survey）
    FILL = "fill"                    # 填充掃描


@dataclass
class CoverageParameters:
    """覆蓋參數"""
    spacing: float                              # 掃描線間距（公尺）
    angle: float                                # 掃描角度（度）
    pattern: ScanPattern = ScanPattern.GRID
    overlap: float = 0.1                        # 重疊率（0-1）
    start_from_corner: bool = True              # 是否從角落開始

    # 固定翼參數
    is_fixed_wing: bool = False
    turn_radius: float = 50.0
    smooth_turns: bool = True

    # === 螺旋掃描專用參數 ===
    spiral_curvature: float = 1.0               # 螺旋曲率倍數 (1=標準, >1=鬆散, <1=緊密)
    spiral_alt_step: float = 0.0               # 每圈高度步階（公尺，0=等高）
    spiral_base_altitude: float = 50.0         # 螺旋基礎飛行高度

    # === 同心圓擴張上升參數 ===
    circle_direction: int = 1                   # 繞行方向：1=逆時針, -1=順時針
    circle_start_angle: float = 0.0             # 起始角度（度，0=北）
    circle_points_per_ring: int = 36            # 每圈航點數（越多越圓滑）
    circle_altitude_step: float = 0.0           # 每圈高度遞增（公尺，0=等高）
    circle_base_altitude: float = 50.0          # 基礎飛行高度（公尺）
    circle_max_radius: float = 0.0              # 最大半徑限制（0=自動偵測）
    circle_min_radius: float = 0.0              # 起始最小半徑（0=從中心開始）
    circle_roi_altitude: float = 0.0            # ROI 目標高度（地面高度）
    circle_camera_angle: float = 45.0           # 相機俯角（度，用於計算有效覆蓋）
    circle_ring_connection: str = "tangent"      # 圈間連接方式: tangent / radial / direct


class CoveragePlanner:
    """覆蓋路徑規劃器"""

    def __init__(self, collision_checker: Optional[CollisionChecker] = None):
        self.collision_checker = collision_checker

    def plan_coverage(self,
                     polygon: List[Tuple[float, float]],
                     params: CoverageParameters) -> List[Tuple[float, float]]:
        """
        規劃覆蓋路徑

        參數:
            polygon: 多邊形區域（經緯度）
            params: 覆蓋參數

        返回:
            覆蓋路徑點列表
        """
        if params.pattern == ScanPattern.GRID:
            path = self._plan_grid_coverage(polygon, params)
        elif params.pattern == ScanPattern.SPIRAL:
            path = self._plan_spiral_coverage(polygon, params)
        elif params.pattern == ScanPattern.CIRCULAR:
            path = self._plan_circle_survey(polygon, params)
        else:
            raise ValueError(f"不支持的掃描模式: {params.pattern}")

        # 固定翼平滑轉彎（僅對方格路徑有效）
        # 螺旋/同心圓路徑由高密度弧點自帶平滑，_add_smooth_turns 會在已平滑曲線上
        # 製造額外誤差點，導致鋸齒，故僅對 GRID 啟用。
        if (params.is_fixed_wing and params.smooth_turns and len(path) > 2
                and params.pattern == ScanPattern.GRID):
            path = self._add_smooth_turns(path, params.turn_radius, polygon)

        return path

    def plan_subdivided_coverage(self,
                                  polygon: List[Tuple[float, float]],
                                  params: CoverageParameters,
                                  n_regions: int = 1,
                                  spacing_m: float = 0.0,
                                  gap_spacings_m: Optional[List[float]] = None,
                                  v_spacing_m: Optional[float] = None,
                                  ) -> List[List[Tuple[float, float]]]:
        """
        分區覆蓋路徑規劃（支援最多 6 台無人機）

        將多邊形分割為 n_regions 個子區域，各子區域獨立呼叫
        plan_coverage() 生成覆蓋路徑。相鄰子區域的起始方向
        自動交替（start_from_corner 取反），減少子區域間的
        轉場距離。

        分割策略（由 RegionDivider 決定）：
          n=1      : 不分割
          n=2,3,5  : 水平條帶
          n=4      : 2×2 網格
          n=6      : 3×2 網格（3 列 × 2 行）

        參數:
            polygon         : 主多邊形區域（經緯度）
            params          : 覆蓋參數（各子區域共用，方向自動交替）
            n_regions       : 分割數量（1–6）
            spacing_m       : 預設子區域間隔（公尺），0 = 無間隔
            gap_spacings_m  : 條帶模式各邊界獨立間隔列表（長度 = n-1），
                              覆蓋 spacing_m
            v_spacing_m     : 網格模式垂直方向間隔（公尺）；
                              水平方向使用 spacing_m

        返回:
            List[path_region_i]，長度等於實際生成的子區域數
        """
        if not 1 <= n_regions <= 6:
            raise ValueError(f"n_regions 必須在 1–6 之間，收到 {n_regions}")

        if n_regions == 1:
            return [self.plan_coverage(polygon, params)]

        from ..geometry import RegionDivider

        if len(polygon) == 4:
            sub_polygons = RegionDivider.subdivide_rectangle(
                polygon, n_regions,
                spacing_m=spacing_m,
                gap_spacings_m=gap_spacings_m,
                v_spacing_m=v_spacing_m,
            )
        else:
            sub_polygons = RegionDivider.subdivide_polygon(polygon, n_regions, spacing_m)

        paths: List[List[Tuple[float, float]]] = []
        for i, sub_poly in enumerate(sub_polygons):
            if len(sub_poly) < 3:
                continue
            # 相鄰子區域交替起始方向，減少折返距離
            sub_params = replace(params, start_from_corner=(i % 2 == 0))
            paths.append(self.plan_coverage(sub_poly, sub_params))

        return paths

    def plan_spiral_from_center(
        self,
        center_lat: float,
        center_lon: float,
        max_radius_m: float,
        spacing_m: float,
        is_fixed_wing: bool = False,
        turn_radius: float = 50.0,
        curvature: float = 1.0,
        alt_step: float = 0.0,
        base_alt: float = 50.0,
        entry_bearing_deg: float = 0.0,
    ) -> List[Tuple[float, float]]:
        """
        從圓心生成螺旋掃描路徑（弧長自適應步進，支援曲率與高度步階）

        參數:
            curvature   : 曲率倍數，>1 圈距較寬（鬆散），<1 圈距較窄（緊密）
            alt_step    : 每完整旋轉一圈的高度增量（公尺），0 = 等高
            base_alt    : 起始高度（公尺）
        """
        coord_transform = CoordinateTransform(center_lat, center_lon)

        # a 決定每圈半徑增量：r = a * theta，每轉一圈（2π）增加 a*2π = spacing*curvature
        a = max(spacing_m * max(curvature, 0.1) / (2.0 * math.pi), 0.01)

        # 高密度步進確保路徑平滑（避免鋸齒）
        if is_fixed_wing and turn_radius > 0:
            ds_target = max(0.5, min(turn_radius / 10.0, spacing_m / 8.0))
        else:
            ds_target = max(1.0, spacing_m / 4.0)

        path_xy = []
        altitudes = []
        # 固定翼：起始半徑需留 1.5× 轉彎半徑安全餘量（曲率 1/r ≤ 1/turn_r）
        # 多旋翼：起始角度 = 2π，對應半徑 = spacing × curvature，
        #          避免從圓心以極小半徑起飛導致內圈曲率過高
        if is_fixed_wing and turn_radius > 0:
            theta_start = max(1.5 * turn_radius / a, 0.01)
        else:
            theta_start = max(2.0 * math.pi, 0.01)
        theta = theta_start

        # 旋轉偏移：使螺旋第一點面向進場方向
        # 羅盤方位 B → 局部 ENU 數學角 = π/2 - B_rad（x=East, y=North）
        entry_math_angle = math.pi / 2.0 - math.radians(entry_bearing_deg)
        angle_offset = entry_math_angle - theta_start
        cos_off = math.cos(angle_offset)
        sin_off = math.sin(angle_offset)

        while True:
            r = a * theta
            if r > max_radius_m:
                break
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            x_rot = x * cos_off - y * sin_off
            y_rot = x * sin_off + y * cos_off
            path_xy.append((x_rot, y_rot))

            # 每完整一圈（2π）高度遞增 alt_step
            alt = base_alt + (theta / (2.0 * math.pi)) * alt_step
            altitudes.append(alt)

            # 弧長自適應步進：dθ = ds_target / sqrt(r² + a²)
            arc_factor = math.sqrt(r * r + a * a)
            dtheta = ds_target / arc_factor
            dtheta = max(dtheta, math.radians(0.05))
            theta += dtheta

        # RDP 簡化：螺旋路徑中心密、外圍疏，自適應保留曲率大的部分
        # 固定翼：弦長目標 = turn_radius*0.3，epsilon = chord²/(8*r_min)
        if is_fixed_wing and turn_radius > 0:
            r_min = theta_start * a  # 起始半徑（已 ≥ 1.5 × turn_radius）
            chord_target = max(1.0, min(turn_radius * 0.3, spacing_m * 0.5))
            epsilon = max(0.5, (chord_target ** 2) / (8.0 * r_min))
        else:
            epsilon = max(1.0, spacing_m / 8.0)
        path_xy, altitudes = self._rdp_simplify_xy(path_xy, epsilon, altitudes)

        # 固定翼：RDP 後過濾 > 85° 急轉彎（安全網）
        if is_fixed_wing and turn_radius > 0 and len(path_xy) >= 3:
            path_xy, altitudes = self._split_large_turns_xy(path_xy, altitudes, turn_radius)

        self._last_spiral_altitudes = altitudes
        self._last_spiral_center = (center_lat, center_lon)
        return coord_transform.batch_xy_to_latlon(path_xy)

    def get_spiral_altitudes(self) -> List[float]:
        """取得最近一次螺旋掃描的高度列表"""
        return getattr(self, '_last_spiral_altitudes', [])

    def get_spiral_center(self) -> Tuple[float, float]:
        """取得最近一次螺旋掃描的中心點"""
        return getattr(self, '_last_spiral_center', (0.0, 0.0))

    def plan_circle_survey_from_center(
        self,
        center_lat: float,
        center_lon: float,
        max_radius_m: float,
        spacing_m: float,
        base_alt: float = 50.0,
        alt_step: float = 0.0,
        direction: int = 1,
        points_per_ring: int = 72,
        min_radius_m: float = 0.0,
        start_angle_deg: float = 0.0,
    ) -> List[Tuple[float, float]]:
        """
        從圓心生成同心圓掃描路徑
        類似 Mission Planner 的 Create Circle Survey

        不依賴多邊形，從 min_radius_m 到 max_radius_m 生成同心圓環。
        """
        coord_transform = CoordinateTransform(center_lat, center_lon)

        params = CoverageParameters(
            spacing=spacing_m,
            angle=0.0,
            circle_direction=direction,
            circle_points_per_ring=points_per_ring,
            circle_altitude_step=alt_step,
            circle_base_altitude=base_alt,
            circle_start_angle=start_angle_deg,
            circle_ring_connection="tangent",
        )

        min_r = max(min_radius_m, spacing_m * 0.5)
        path_xy, altitudes = self._generate_concentric_rings(max_radius_m, min_r, params)

        path_latlon = coord_transform.batch_xy_to_latlon(path_xy)
        self._last_circle_altitudes = altitudes
        self._last_circle_center = (center_lat, center_lon)

        return path_latlon

    def generate_transit_paths(
        self,
        home: Tuple[float, float],
        sub_paths: List[List[Tuple[float, float]]],
        base_alt: float,
        stagger_alt: float = 10.0,
    ) -> List[dict]:
        """
        生成從起飛點到各子區域的轉場路徑，使用高度錯層避免路線交錯碰撞

        策略：
          - 無人機 i 以高度 (base_alt + i × stagger_alt) 飛往子區域 i 起點
          - 各無人機在不同高度層轉場，物理上不會交錯

        參數:
            home        : 起飛點 (lat, lon)
            sub_paths   : 各子區域的覆蓋路徑
            base_alt    : 任務基礎高度（公尺）
            stagger_alt : 每台無人機高度錯層間距（公尺）

        返回:
            list of dict，每筆包含：
              'path'         : [(lat, lon), (lat, lon)]  起飛點 → 子區域起點
              'altitude'     : float  此條轉場路徑的飛行高度
              'region_index' : int    對應的子區域索引
        """
        transit_paths = []
        for i, sub_path in enumerate(sub_paths):
            if not sub_path:
                continue
            transit_alt = base_alt + i * stagger_alt
            transit_paths.append({
                'path': [home, sub_path[0]],
                'altitude': transit_alt,
                'region_index': i,
            })
        return transit_paths

    # ================================================================
    # 同心圓擴張上升掃描（Circle Survey）
    # 參考 Mission Planner Create Circle Survey 演算法
    #
    # 核心概念：
    #   1. 從中心（或最小半徑）開始，逐圈向外擴張
    #   2. 每圈是一個完整的圓，航點均勻分布
    #   3. 圈間透過切線平滑過渡，避免急轉
    #   4. 可選逐圈高度遞增（螺旋上升效果）
    #   5. 每圈設定 ROI 指向中心點，相機始終朝內
    # ================================================================

    def _plan_circle_survey(self,
                           polygon: List[Tuple[float, float]],
                           params: CoverageParameters) -> List[Tuple[float, float]]:
        """
        同心圓擴張上升掃描路徑規劃

        演算法流程：
        1. 計算多邊形質心作為同心圓中心
        2. 計算最大外接圓半徑（或使用者指定）
        3. 從最小半徑開始，每圈增加 spacing
        4. 每圈生成 N 個均勻航點
        5. 圈間以切線連接實現平滑過渡
        6. 僅保留落在多邊形內的航點

        參數:
            polygon: 多邊形區域（經緯度）
            params: 覆蓋參數

        返回:
            [(lat, lon), ...] 帶高度資訊可透過 get_circle_survey_altitudes() 取得
        """
        # 計算中心點
        center_lat = sum(p[0] for p in polygon) / len(polygon)
        center_lon = sum(p[1] for p in polygon) / len(polygon)

        # 建立座標轉換器
        coord_transform = CoordinateTransform(center_lat, center_lon)
        polygon_xy = coord_transform.batch_latlon_to_xy(polygon)

        # 計算最大半徑
        max_radius = params.circle_max_radius
        if max_radius <= 0:
            max_radius = max(math.sqrt(p[0]**2 + p[1]**2) for p in polygon_xy)

        # 起始半徑
        min_radius = max(params.circle_min_radius, params.spacing * 0.5)

        # 生成同心圓環路徑（平面座標系）
        path_xy, altitudes = self._generate_concentric_rings(
            max_radius, min_radius, params
        )

        # 過濾：僅保留多邊形內的點
        filtered_xy = []
        filtered_alt = []
        for (x, y), alt in zip(path_xy, altitudes):
            if self._point_in_polygon((x, y), polygon_xy):
                filtered_xy.append((x, y))
                filtered_alt.append(alt)

        # 轉換回經緯度
        path_latlon = coord_transform.batch_xy_to_latlon(filtered_xy)

        # 儲存高度資訊供後續匯出使用
        self._last_circle_altitudes = filtered_alt
        self._last_circle_center = (center_lat, center_lon)

        # 碰撞過濾
        if self.collision_checker:
            path_latlon = self._filter_collision_points(path_latlon)

        return path_latlon

    def _generate_concentric_rings(self,
                                   max_radius: float,
                                   min_radius: float,
                                   params: CoverageParameters
                                   ) -> Tuple[List[Tuple[float, float]], List[float]]:
        """
        生成同心圓環路徑（核心演算法）

        改進重點：
        - 自適應每圈航點數（依圓周長計算），確保大圓不鋸齒
        - 螺旋弧線圈間過渡（更多插值點），避免圈間跳躍
        """
        path_xy = []
        altitudes = []
        direction = params.circle_direction
        start_angle_rad = math.radians(params.circle_start_angle)
        spacing = params.spacing
        alt_step = params.circle_altitude_step
        base_alt = params.circle_base_altitude
        connection = params.circle_ring_connection

        # 目標弧段長度：ds_target 越小，路徑越圓滑
        # 固定翼：弦長不能比 turn_radius 小，否則 acceptance_radius 會跳點
        is_fw   = params.is_fixed_wing and params.turn_radius > 0
        turn_r  = params.turn_radius if is_fw else 0.0
        if is_fw:
            # 每段弦長 ≈ turn_radius * 0.3（15m for TR=50m），讓 acceptance_radius 有空間
            # ds ≈ chord：ds_target 即直接當弦長目標
            ds_target = max(1.0, min(turn_r * 0.3, spacing * 0.5))
        else:
            ds_target = max(0.3, spacing / 20.0)

        # 固定翼：最小半徑需留 1.5× 轉彎半徑安全餘量（比 1× 寬鬆一個半徑的緩衝）
        effective_min = max(min_radius, turn_r * 1.5) if is_fw else min_radius
        current_radius = effective_min
        ring_index = 0

        while current_radius <= max_radius:
            ring_alt = base_alt + ring_index * alt_step

            # 每圈航點數計算
            circumference = 2.0 * math.pi * current_radius
            if is_fw:
                # 固定翼：最大 20° 轉彎角/航點，確保飛機不需超過 20° 側傾即可追蹤航線
                # ceil(360°/20°) = 18 點/圈，不受半徑影響（角度決定）
                _max_angle_per_pt = math.radians(20.0)
                points_this_ring = max(18, int(math.ceil(2.0 * math.pi / _max_angle_per_pt)))
            else:
                # 多旋翼：固定 15° 角度步進（24 點/圈），直線段保留端點
                _max_angle_per_pt = math.radians(15.0)
                points_this_ring = max(24, int(math.ceil(2.0 * math.pi / _max_angle_per_pt)))
            angular_step = 2.0 * math.pi / points_this_ring

            # 切線偏移起始角度，使圈間過渡更順暢
            if connection == "tangent" and ring_index > 0:
                ring_start = start_angle_rad + direction * angular_step * 0.5 * ring_index
            else:
                ring_start = start_angle_rad

            # 生成本圈航點
            ring_points = []
            for i in range(points_this_ring):
                angle = ring_start + direction * i * angular_step
                x = current_radius * math.cos(angle)
                y = current_radius * math.sin(angle)
                ring_points.append((x, y))

            # 平滑螺旋弧線圈間過渡（充分插值，消除圈間鋸齒）
            if ring_index > 0 and len(path_xy) > 0 and connection == "tangent":
                inner_r = current_radius - spacing
                transition = self._generate_smooth_ring_transition(
                    path_xy[-1], ring_points[0],
                    inner_r, current_radius,
                    ds_target, direction
                )
                prev_alt = altitudes[-1] if altitudes else ring_alt
                for k, tp in enumerate(transition):
                    path_xy.append(tp)
                    t = (k + 1) / (len(transition) + 1)
                    altitudes.append(prev_alt + t * (ring_alt - prev_alt))

            # 添加本圈航點
            for pt in ring_points:
                path_xy.append(pt)
                altitudes.append(ring_alt)

            current_radius += spacing
            ring_index += 1

        # RDP 簡化：移除冗餘點，同時保持路徑幾何精度
        # 固定翼：epsilon 基於 ds_target²/(8*min_r)，確保弦長 ≥ ds_target
        # 多旋翼：epsilon = spacing/20（精度優先）
        if is_fw and turn_r > 0:
            # chord ≥ ds_target ⟹ epsilon = ds_target²/(8*min_r)
            _min_r = effective_min if effective_min > 0 else turn_r * 1.5
            epsilon = max(0.5, (ds_target ** 2) / (8.0 * _min_r))
        else:
            epsilon = max(1.0, spacing / 8.0)
        path_xy, altitudes = self._rdp_simplify_xy(path_xy, epsilon, altitudes)

        # 固定翼：RDP 後過濾 > 85° 急轉彎（安全網）
        if is_fw and turn_r > 0 and len(path_xy) >= 3:
            path_xy, altitudes = self._split_large_turns_xy(path_xy, altitudes, turn_r)

        return path_xy, altitudes

    def _generate_smooth_ring_transition(self,
                                         last_point: Tuple[float, float],
                                         next_point: Tuple[float, float],
                                         inner_radius: float,
                                         outer_radius: float,
                                         ds_target: float,
                                         direction: int = 1
                                         ) -> List[Tuple[float, float]]:
        """
        生成圈間平滑螺旋弧線過渡航點

        從內圈最後一點沿螺旋弧線平滑過渡到外圈起始點，
        插值點數量依弧長自適應，確保無鋸齒。
        """
        x1, y1 = last_point
        x2, y2 = next_point

        angle1 = math.atan2(y1, x1)
        angle2 = math.atan2(y2, x2)

        # 依繞行方向正規化角度差
        diff = angle2 - angle1
        if direction > 0:
            while diff < 0:
                diff += 2.0 * math.pi
            while diff > 2.0 * math.pi:
                diff -= 2.0 * math.pi
        else:
            while diff > 0:
                diff -= 2.0 * math.pi
            while diff < -2.0 * math.pi:
                diff += 2.0 * math.pi

        dr = outer_radius - inner_radius
        # 角度步進插值：每 15° 一點，與圈內航點密度一致，最少 3 點
        num_pts = max(3, int(math.ceil(math.degrees(abs(diff)) / 15.0)))

        transitions = []
        for i in range(1, num_pts):
            t = i / num_pts
            r = inner_radius + t * dr
            angle = angle1 + t * diff
            transitions.append((r * math.cos(angle), r * math.sin(angle)))

        return transitions

    @staticmethod
    def limit_path_waypoints(
        path: List[Tuple[float, float]],
        max_waypoints: int,
        altitudes: Optional[List[float]] = None,
    ) -> Tuple[List[Tuple[float, float]], List[float]]:
        """
        強制將 lat/lon 路徑縮減至 max_waypoints 個點以內。

        演算法：
          1. 將 lat/lon 轉換為平面 XY（公尺）
          2. 以 iterative RDP 逐步增大 epsilon 直到點數 ≤ max_waypoints
          3. 若 30 次迭代後仍超出，以均勻取樣做最終保底

        params:
            path         : [(lat, lon), ...] 輸入路徑
            max_waypoints: 最大允許點數
            altitudes    : 對應高度列表（可選，同步縮減）
        returns:
            (trimmed_path, trimmed_altitudes)
        """
        n = len(path)
        if n <= max_waypoints:
            return list(path), list(altitudes) if altitudes else []

        # 以路徑中心點建立 XY 座標系
        from ..geometry import CoordinateTransform
        center_lat = sum(p[0] for p in path) / n
        center_lon = sum(p[1] for p in path) / n
        ct = CoordinateTransform(center_lat, center_lon)
        xy = ct.batch_latlon_to_xy(path)

        # 迭代 RDP：每次 epsilon × 1.5，直到點數達標
        epsilon = 0.5
        for _ in range(30):
            pts_xy, pts_alt = CoveragePlanner._rdp_simplify_xy(xy, epsilon, altitudes)
            if len(pts_xy) <= max_waypoints:
                return ct.batch_xy_to_latlon(pts_xy), pts_alt
            epsilon *= 1.5

        # 保底：均勻取樣（保留首尾）
        step = (n - 1) / (max_waypoints - 1)
        indices = sorted({0}
                         | {int(round(i * step)) for i in range(1, max_waypoints - 1)}
                         | {n - 1})[:max_waypoints]
        result_path = [path[i] for i in indices]
        result_alts = ([altitudes[i] for i in indices]
                       if altitudes and len(altitudes) == n else [])
        return result_path, result_alts

    @staticmethod
    def _split_large_turns_xy(
        points: List[Tuple[float, float]],
        altitudes: Optional[List[float]],
        turn_radius: float,
        max_turn_deg: float = 85.0,
    ) -> Tuple[List[Tuple[float, float]], List[float]]:
        """
        固定翼急轉彎過濾器：RDP 後安全網

        對連續航點間方向變化 > max_turn_deg 的轉角，
        在轉角前後各插入一個回退點（setback），
        將一個尖角分解成兩段 ≤ max_turn_deg 的轉彎。

        最多執行 3 次 pass，直到不再有急轉。
        """
        max_turn_rad = math.radians(max_turn_deg)
        setback = turn_radius * 0.8   # 回退距離，留一些餘量

        alts = list(altitudes) if altitudes else []
        pts = list(points)
        use_alts = bool(alts) and len(alts) == len(pts)

        for _ in range(3):
            new_pts: List[Tuple[float, float]] = [pts[0]]
            new_alts: List[float] = [alts[0]] if use_alts else []
            changed = False

            for i in range(1, len(pts) - 1):
                x0, y0 = pts[i - 1]
                x1, y1 = pts[i]
                x2, y2 = pts[i + 1]

                dx_in, dy_in = x1 - x0, y1 - y0
                dx_out, dy_out = x2 - x1, y2 - y1
                len_in = math.hypot(dx_in, dy_in)
                len_out = math.hypot(dx_out, dy_out)

                if len_in < 1e-6 or len_out < 1e-6:
                    new_pts.append(pts[i])
                    if use_alts:
                        new_alts.append(alts[i])
                    continue

                ux_in, uy_in = dx_in / len_in, dy_in / len_in
                ux_out, uy_out = dx_out / len_out, dy_out / len_out

                dot = max(-1.0, min(1.0, ux_in * ux_out + uy_in * uy_out))
                turn_angle = math.acos(dot)  # 0 = 直行, π = 180° 掉頭

                if turn_angle > max_turn_rad and setback < len_in * 0.9 and setback < len_out * 0.9:
                    # 在轉角前/後各退 setback 距離，插入兩個航點取代原轉角點
                    pre_x  = x1 - ux_in  * setback
                    pre_y  = y1 - uy_in  * setback
                    post_x = x1 + ux_out * setback
                    post_y = y1 + uy_out * setback
                    alt_i  = alts[i] if use_alts else 0.0
                    new_pts.append((pre_x, pre_y))
                    new_pts.append((post_x, post_y))
                    if use_alts:
                        new_alts.append(alt_i)
                        new_alts.append(alt_i)
                    changed = True
                else:
                    new_pts.append(pts[i])
                    if use_alts:
                        new_alts.append(alts[i])

            new_pts.append(pts[-1])
            if use_alts:
                new_alts.append(alts[-1])

            pts = new_pts
            if use_alts:
                alts = new_alts

            if not changed:
                break

        return pts, alts if use_alts else []

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """正規化角度到 [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def _rdp_simplify_xy(
        points: List[Tuple[float, float]],
        epsilon: float,
        altitudes: Optional[List[float]] = None,
    ) -> Tuple[List[Tuple[float, float]], List[float]]:
        """
        Ramer-Douglas-Peucker 路徑簡化（平面座標）

        在保持路徑幾何形狀的前提下移除冗餘航點。
        對圓弧、螺旋等高密度採樣路徑效果顯著：
        - 曲率大（小半徑）的段落自動保留較多點
        - 曲率小（大半徑）的段落大幅減少點數

        params:
            points   : [(x,y), ...] 路徑點（公尺平面座標）
            epsilon  : 最大允許偏差（公尺），值越大簡化越激進
            altitudes: 對應高度列表（與 points 等長，可選）
        returns:
            (simplified_points, simplified_altitudes)
        """
        n = len(points)
        if n < 3:
            return list(points), list(altitudes) if altitudes else []

        def _perp_dist_sq(px, py, ax, ay, bx, by):
            dx, dy = bx - ax, by - ay
            if dx == 0.0 and dy == 0.0:
                return (px - ax) ** 2 + (py - ay) ** 2
            t = max(0.0, min(1.0,
                    ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
            return (px - ax - t * dx) ** 2 + (py - ay - t * dy) ** 2

        eps_sq = epsilon * epsilon
        keep = bytearray(n)
        keep[0] = 1
        keep[n - 1] = 1

        # 迭代版（避免 Python 遞迴深度限制）
        stack = [(0, n - 1)]
        while stack:
            s, e = stack.pop()
            if e - s < 2:
                continue
            ax, ay = points[s]
            bx, by = points[e]
            max_dsq, max_i = 0.0, s + 1
            for i in range(s + 1, e):
                px, py = points[i]
                dsq = _perp_dist_sq(px, py, ax, ay, bx, by)
                if dsq > max_dsq:
                    max_dsq, max_i = dsq, i
            if max_dsq > eps_sq:
                keep[max_i] = 1
                stack.append((s, max_i))
                stack.append((max_i, e))

        indices = [i for i in range(n) if keep[i]]
        simplified = [points[i] for i in indices]
        if altitudes and len(altitudes) == n:
            simplified_alts = [altitudes[i] for i in indices]
        else:
            simplified_alts = []
        return simplified, simplified_alts

    def get_circle_survey_altitudes(self) -> List[float]:
        """取得最近一次 Circle Survey 的高度列表"""
        return getattr(self, '_last_circle_altitudes', [])

    def get_circle_survey_center(self) -> Tuple[float, float]:
        """取得最近一次 Circle Survey 的中心點"""
        return getattr(self, '_last_circle_center', (0.0, 0.0))

    def generate_circle_survey_waypoint_lines(self,
                                               path: List[Tuple[float, float]],
                                               params: CoverageParameters,
                                               speed: float = 5.0,
                                               yaw_speed: float = 60.0
                                               ) -> List[str]:
        """
        生成 Circle Survey 的 QGC WPL 110 航點檔案

        特殊指令：
        - MAV_CMD_DO_SET_ROI (201): 設定 ROI 指向中心
        - MAV_CMD_NAV_WAYPOINT (16): 標準航點
        - MAV_CMD_CONDITION_YAW (115): 轉向指令
        - MAV_CMD_DO_CHANGE_SPEED (178): 速度設定

        參數:
            path: 路徑點列表 (lat, lon)
            params: 覆蓋參數
            speed: 飛行速度 (m/s)
            yaw_speed: 轉向速度 (deg/s)

        返回:
            QGC WPL 110 格式的航點行列表
        """
        altitudes = self.get_circle_survey_altitudes()
        center = self.get_circle_survey_center()

        if len(altitudes) != len(path):
            # 如果高度資訊不匹配，使用基礎高度
            altitudes = [params.circle_base_altitude] * len(path)

        lines = ["QGC WPL 110"]
        seq = 0

        # HOME 點
        lines.append(f"{seq}\t0\t3\t179\t0\t0\t0\t0\t0\t0\t0\t1")
        seq += 1

        # 速度設定
        lines.append(f"{seq}\t0\t3\t178\t0\t{speed:.1f}\t0\t0\t0\t0\t0\t1")
        seq += 1

        # ROI 設定（指向中心點）
        roi_alt = params.circle_roi_altitude
        lines.append(
            f"{seq}\t0\t3\t201\t0\t0\t0\t0\t"
            f"{center[0]:.6f}\t{center[1]:.6f}\t{roi_alt:.2f}\t1"
        )
        seq += 1

        # 航點
        prev_lat, prev_lon = None, None
        for i, ((lat, lon), alt) in enumerate(zip(path, altitudes)):
            # 轉向指令
            if prev_lat is not None:
                bearing = self._calculate_bearing(prev_lat, prev_lon, lat, lon)
                lines.append(
                    f"{seq}\t0\t3\t115\t{bearing:.1f}\t{yaw_speed:.1f}\t0\t0\t0\t0\t0\t1"
                )
                seq += 1

            # 航點（帶高度）
            lines.append(
                f"{seq}\t0\t3\t16\t0\t0\t0\t0\t{lat:.6f}\t{lon:.6f}\t{alt:.2f}\t1"
            )
            seq += 1
            prev_lat, prev_lon = lat, lon

        # 清除 ROI
        lines.append(f"{seq}\t0\t3\t201\t0\t0\t0\t0\t0\t0\t0\t1")
        seq += 1

        # RTL
        lines.append(f"{seq}\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1")

        return lines

    def generate_grid_waypoint_lines(self,
                                     path: List[Tuple[float, float]],
                                     altitude: float = 50.0,
                                     speed: float = 10.0,
                                     yaw_speed: float = 60.0
                                     ) -> List[str]:
        """
        生成網格掃描的 QGC WPL 110 航點檔案

        對應 OptimizedWaypointGenerator.generate_grid_waypoints 的輸出格式，
        與 generate_circle_survey_waypoint_lines 結構一致：

        - MAV_CMD_NAV_HOME      (179): HOME 點佔位
        - MAV_CMD_DO_CHANGE_SPEED (178): 起始速度設定
        - MAV_CMD_CONDITION_YAW (115): 每個航點前的偏航指令
        - MAV_CMD_NAV_WAYPOINT  (16) : 標準航點
        - MAV_CMD_DO_CHANGE_SPEED (178): 末段減速至 10 m/s

        參數:
            path: plan_coverage() 回傳的路徑點 (lat, lon)
            altitude: 飛行高度 (m)
            speed: 巡航速度 (m/s)
            yaw_speed: 偏航轉速 (deg/s)

        返回:
            QGC WPL 110 格式的航點行列表
        """
        lines = ["QGC WPL 110"]
        seq = 0

        # HOME 點
        lines.append(f"{seq}\t0\t3\t179\t0\t0\t0\t0\t0\t0\t0\t1")
        seq += 1

        # 起始速度
        lines.append(f"{seq}\t0\t3\t178\t0\t{speed:.1f}\t0\t0\t0\t0\t0\t1")
        seq += 1

        first_lat, first_lon = None, None
        prev_lat, prev_lon = None, None
        for lat, lon in path:
            # 記錄第一個點，供返航使用
            if first_lat is None:
                first_lat, first_lon = lat, lon

            # 轉向指令（除第一個點外）
            if prev_lat is not None:
                bearing = self._calculate_bearing(prev_lat, prev_lon, lat, lon)
                lines.append(
                    f"{seq}\t0\t3\t115\t{bearing:.1f}\t{yaw_speed:.1f}\t0\t0\t0\t0\t0\t1"
                )
                seq += 1

            # 航點
            lines.append(
                f"{seq}\t0\t3\t16\t0\t0\t0\t0\t{lat:.6f}\t{lon:.6f}\t{altitude:.2f}\t1"
            )
            seq += 1
            prev_lat, prev_lon = lat, lon

        # 返回起點（路徑最後一點 → 第一個航點）
        if first_lat is not None and prev_lat is not None:
            bearing = self._calculate_bearing(prev_lat, prev_lon, first_lat, first_lon)
            lines.append(
                f"{seq}\t0\t3\t115\t{bearing:.1f}\t{yaw_speed:.1f}\t0\t0\t0\t0\t0\t1"
            )
            seq += 1
            lines.append(
                f"{seq}\t0\t3\t16\t0\t0\t0\t0\t{first_lat:.6f}\t{first_lon:.6f}\t{altitude:.2f}\t1"
            )
            seq += 1

        # 末段減速後 RTL
        lines.append(f"{seq}\t0\t3\t178\t0\t5.0\t0\t0\t0\t0\t0\t1")
        seq += 1
        lines.append(f"{seq}\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1")

        return lines

    @staticmethod
    def _calculate_bearing(lat1: float, lon1: float,
                           lat2: float, lon2: float) -> float:
        """計算兩點間方位角"""
        dlon = math.radians(lon2 - lon1)
        lat1_r, lat2_r = math.radians(lat1), math.radians(lat2)
        x = math.sin(dlon) * math.cos(lat2_r)
        y = (math.cos(lat1_r) * math.sin(lat2_r) -
             math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon))
        return (math.degrees(math.atan2(x, y)) + 360) % 360

    def calculate_circle_survey_stats(self,
                                      path: List[Tuple[float, float]],
                                      params: CoverageParameters
                                      ) -> dict:
        """
        計算 Circle Survey 任務統計

        返回:
            {
                'total_rings': int,
                'total_waypoints': int,
                'total_distance_m': float,
                'max_altitude_m': float,
                'min_altitude_m': float,
                'estimated_time_s': float,
                'coverage_area_m2': float
            }
        """
        altitudes = self.get_circle_survey_altitudes()
        total_dist = 0.0

        for i in range(len(path) - 1):
            lat1, lon1 = path[i]
            lat2, lon2 = path[i + 1]
            dlat = (lat2 - lat1) * 111111.0
            dlon = (lon2 - lon1) * 111111.0 * math.cos(math.radians((lat1 + lat2) / 2))
            total_dist += math.sqrt(dlat**2 + dlon**2)

        max_radius = params.circle_max_radius
        min_radius = max(params.circle_min_radius, params.spacing * 0.5)
        num_rings = max(1, int((max_radius - min_radius) / params.spacing) + 1)

        return {
            'total_rings': num_rings,
            'total_waypoints': len(path),
            'total_distance_m': total_dist,
            'max_altitude_m': max(altitudes) if altitudes else params.circle_base_altitude,
            'min_altitude_m': min(altitudes) if altitudes else params.circle_base_altitude,
            'estimated_time_s': total_dist / 5.0,  # 預設 5 m/s
            'coverage_area_m2': math.pi * max_radius**2,
        }

    # ================================================================
    # 以下為原有的 Grid / Spiral 方法（保持不變）
    # ================================================================

    def _add_smooth_turns(self,
                         path: List[Tuple[float, float]],
                         turn_radius: float,
                         polygon: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """為路徑添加平滑轉彎（Dubins 路徑風格）"""
        if len(path) < 3:
            return path

        center_lat = sum(p[0] for p in polygon) / len(polygon)
        center_lon = sum(p[1] for p in polygon) / len(polygon)

        radius_lat = turn_radius / 111111.0
        radius_lon = turn_radius / (111111.0 * math.cos(math.radians(center_lat)))

        smoothed_path = [path[0]]

        for i in range(1, len(path) - 1):
            prev_point = path[i - 1]
            curr_point = path[i]
            next_point = path[i + 1]

            heading_in = math.atan2(
                curr_point[1] - prev_point[1],
                curr_point[0] - prev_point[0]
            )
            heading_out = math.atan2(
                next_point[1] - curr_point[1],
                next_point[0] - curr_point[0]
            )

            turn_angle = heading_out - heading_in
            while turn_angle > math.pi:
                turn_angle -= 2 * math.pi
            while turn_angle < -math.pi:
                turn_angle += 2 * math.pi

            if abs(turn_angle) < math.radians(10):
                smoothed_path.append(curr_point)
                continue

            arc_points = self._generate_arc_turn(
                prev_point, curr_point, next_point,
                radius_lat, radius_lon, turn_angle
            )
            smoothed_path.extend(arc_points)

        smoothed_path.append(path[-1])
        return smoothed_path

    def _generate_arc_turn(self,
                          prev_point: Tuple[float, float],
                          curr_point: Tuple[float, float],
                          next_point: Tuple[float, float],
                          radius_lat: float,
                          radius_lon: float,
                          turn_angle: float) -> List[Tuple[float, float]]:
        """生成圓弧轉彎點"""
        arc_points = []

        heading_in = math.atan2(
            curr_point[1] - prev_point[1],
            curr_point[0] - prev_point[0]
        )
        heading_out = math.atan2(
            next_point[1] - curr_point[1],
            next_point[0] - curr_point[0]
        )

        turn_direction = 1 if turn_angle > 0 else -1
        perpendicular_in = heading_in + turn_direction * math.pi / 2

        half_turn = abs(turn_angle) / 2
        offset_distance = abs(radius_lat / math.tan(half_turn)) if abs(half_turn) > 0.01 else 0

        arc_start_lat = curr_point[0] - offset_distance * math.cos(heading_in)
        arc_start_lon = curr_point[1] - offset_distance * math.sin(heading_in)

        arc_end_lat = curr_point[0] + offset_distance * math.cos(heading_out)
        arc_end_lon = curr_point[1] + offset_distance * math.sin(heading_out)

        center_lat = arc_start_lat + radius_lat * math.cos(perpendicular_in)
        center_lon = arc_start_lon + radius_lon * math.sin(perpendicular_in)

        num_arc_points = max(3, int(abs(turn_angle) / math.radians(15)))
        start_angle = math.atan2(arc_start_lon - center_lon, arc_start_lat - center_lat)

        for j in range(num_arc_points + 1):
            t = j / num_arc_points
            angle = start_angle + t * turn_angle * turn_direction
            arc_lat = center_lat + radius_lat * math.cos(angle)
            arc_lon = center_lon + radius_lon * math.sin(angle)
            arc_points.append((arc_lat, arc_lon))

        return arc_points

    def _plan_grid_coverage(self,
                          polygon: List[Tuple[float, float]],
                          params: CoverageParameters) -> List[Tuple[float, float]]:
        """網格掃描路徑規劃"""
        center_lat = sum(p[0] for p in polygon) / len(polygon)
        center_lon = sum(p[1] for p in polygon) / len(polygon)

        rotated_system = RotatedCoordinateSystem(
            center_lat, center_lon, params.angle
        )
        rotated_polygon = rotated_system.batch_latlon_to_xy(polygon)
        path_rotated = self._generate_scan_lines(
            rotated_polygon, params.spacing, params.start_from_corner
        )
        path = rotated_system.batch_xy_to_latlon(path_rotated)

        if self.collision_checker:
            path = self._filter_collision_points(path)

        return path

    def _generate_scan_lines(self,
                           polygon: List[Tuple[float, float]],
                           spacing: float,
                           start_from_left: bool = True) -> List[Tuple[float, float]]:
        """生成掃描線（之字形）

        改進重點（完整參考 OptimizedWaypointGenerator 邏輯）：
        1. margin = spacing * 0.1：邊界擴展 10%，確保靠近邊緣的掃描線
           能完整與多邊形相交，而不是恰好切到頂點
        2. math.ceil + max(1, ...)：比 int() 更精準計算所需掃描線數，
           確保不遺漏最後一條
        3. y > max_y 時夾緊至 max_y：防止最後一條線溢出
        4. 相鄰重複 y 跳過：夾緊後若與前一條重疊則略過
        5. start_from_left 控制第一條掃描線的行進方向
        """
        ys = [p[1] for p in polygon]
        min_y = min(ys)
        max_y = max(ys)

        # 邊界擴展 10%，確保邊緣掃描線完整覆蓋多邊形
        margin = spacing * 0.1
        min_y -= margin
        max_y += margin

        # ceil 確保不遺漏最後一條，至少保留 1 條掃描線
        num_lines = max(1, int(math.ceil((max_y - min_y) / spacing)) + 1)

        path = []
        prev_y = None
        for i in range(num_lines):
            y = min_y + i * spacing

            # 夾緊至上邊界
            if y > max_y:
                y = max_y

            # 跳過因夾緊而重複的掃描線
            if prev_y is not None and abs(y - prev_y) < 1e-10:
                continue
            prev_y = y

            intersections = self._find_line_polygon_intersections(polygon, y)
            if len(intersections) < 2:
                continue
            intersections.sort()

            # 之字形方向：start_from_left=True 時偶數行左→右，奇數行右→左
            go_left_to_right = (i % 2 == 0) if start_from_left else (i % 2 == 1)
            if go_left_to_right:
                path.append((intersections[0], y))
                path.append((intersections[-1], y))
            else:
                path.append((intersections[-1], y))
                path.append((intersections[0], y))

        return path

    def _find_line_polygon_intersections(self,
                                       polygon: List[Tuple[float, float]],
                                       y: float) -> List[float]:
        """計算水平線與多邊形的交點"""
        intersections = []
        n = len(polygon)
        for i in range(n):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % n]
            if (p1[1] <= y <= p2[1]) or (p2[1] <= y <= p1[1]):
                if abs(p2[1] - p1[1]) > 1e-10:
                    t = (y - p1[1]) / (p2[1] - p1[1])
                    x = p1[0] + t * (p2[0] - p1[0])
                    intersections.append(x)
        return intersections

    def _plan_spiral_coverage(self,
                            polygon: List[Tuple[float, float]],
                            params: CoverageParameters) -> List[Tuple[float, float]]:
        """螺旋掃描路徑規劃（阿基米德螺線，弧長自適應步進）

        使用弧長自適應角度步進取代固定角度步進，確保每兩個連續航點間的
        弦長近似恆定。這對固定翼尤其重要：固定角度步進在大半徑時會產生
        過長的直線段（鋸齒），弧長步進可確保路徑曲線平滑。

        阿基米德螺線：r = a * θ，其中 a = spacing / (2π)
        弧長元素：ds = sqrt(r² + a²) * dθ
        因此：dθ = ds_target / sqrt(r² + a²)
        """
        center_lat = sum(p[0] for p in polygon) / len(polygon)
        center_lon = sum(p[1] for p in polygon) / len(polygon)

        coord_transform = CoordinateTransform(center_lat, center_lon)
        polygon_xy = coord_transform.batch_latlon_to_xy(polygon)

        max_radius = max(math.sqrt(p[0]**2 + p[1]**2) for p in polygon_xy)

        # 螺線係數：r = a * theta
        a = max(params.spacing / (2.0 * math.pi), 0.01)

        # 目標弧段長度（決定路徑點密度與光滑度）
        # 固定翼：轉彎半徑/8（確保轉彎段足夠平滑），上限 間距/4，最小 1m
        # 多旋翼：間距/5，最小 1m
        if params.is_fixed_wing and params.turn_radius > 0:
            ds_target = max(1.0, min(params.turn_radius / 8.0, params.spacing / 4.0))
        else:
            ds_target = max(1.0, params.spacing / 5.0)

        path_xy = []

        # 起始角度：
        #   固定翼：從 r = turn_radius 開始，直接省略所有內圈（r < turn_radius 的部分
        #          所需傾斜角 = atan(V²/g/r) 超過機體上限，無法安全飛行）
        #   多旋翼：從 r ≈ 0.5m 開始，無轉彎半徑限制
        if params.is_fixed_wing and params.turn_radius > 0:
            theta = max(params.turn_radius / a, 0.01)
        else:
            theta = max(0.01, 0.5 / a)

        while True:
            r = a * theta
            if r > max_radius:
                break
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            if self._point_in_polygon((x, y), polygon_xy):
                path_xy.append((x, y))

            # 弧長自適應步進：dθ = ds_target / sqrt(r² + a²)
            arc_factor = math.sqrt(r * r + a * a)
            dtheta = ds_target / arc_factor
            # 確保步進不退化為零
            dtheta = max(dtheta, math.radians(0.05))
            theta += dtheta

        # RDP 簡化多邊形螺旋路徑
        # 固定翼：依起始半徑調整 epsilon，與 plan_spiral_from_center 一致
        if params.is_fixed_wing and params.turn_radius > 0:
            r_min = params.turn_radius
            chord_target = max(1.0, min(params.turn_radius * 0.3, params.spacing * 0.5))
            epsilon = max(0.5, (chord_target ** 2) / (8.0 * r_min))
        else:
            epsilon = max(1.0, params.spacing / 8.0)
        path_xy, _ = self._rdp_simplify_xy(path_xy, epsilon)

        # 固定翼：過濾 RDP 後殘留的銳角轉彎（轉向角 > 90° 視為銳角，無法安全飛行）
        if params.is_fixed_wing and params.turn_radius > 0 and len(path_xy) >= 3:
            path_xy, _ = self._split_large_turns_xy(
                path_xy, [], params.turn_radius, max_turn_deg=90.0
            )

        path = coord_transform.batch_xy_to_latlon(path_xy)
        return path

    def _point_in_polygon(self,
                         point: Tuple[float, float],
                         polygon: List[Tuple[float, float]]) -> bool:
        """射線法判斷點是否在多邊形內"""
        x, y = point
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def _filter_collision_points(self,
                                path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """過濾障礙物碰撞點"""
        if not self.collision_checker:
            return path
        return [p for p in path if not self.collision_checker.check_point_collision(p)]

    def calculate_coverage_area(self,
                               polygon: List[Tuple[float, float]]) -> float:
        """計算多邊形面積（平方公尺）"""
        if len(polygon) < 3:
            return 0.0
        center_lat = sum(p[0] for p in polygon) / len(polygon)
        center_lon = sum(p[1] for p in polygon) / len(polygon)
        coord_transform = CoordinateTransform(center_lat, center_lon)
        polygon_xy = coord_transform.batch_latlon_to_xy(polygon)

        area = 0.0
        n = len(polygon_xy)
        for i in range(n):
            j = (i + 1) % n
            area += polygon_xy[i][0] * polygon_xy[j][1]
            area -= polygon_xy[j][0] * polygon_xy[i][1]
        return abs(area) / 2.0

    def estimate_mission_time(self,
                            path: List[Tuple[float, float]],
                            speed: float) -> float:
        """估算任務時間（秒）"""
        if len(path) < 2 or speed <= 0:
            return 0.0
        total_distance = 0.0
        for i in range(len(path) - 1):
            lat1, lon1 = path[i]
            lat2, lon2 = path[i + 1]
            dlat = (lat2 - lat1) * 111111.0
            dlon = (lon2 - lon1) * 111111.0 * math.cos(math.radians((lat1 + lat2) / 2))
            total_distance += math.sqrt(dlat**2 + dlon**2)
        return total_distance / speed


def optimize_scan_angle(polygon: List[Tuple[float, float]],
                       spacing: float,
                       angle_step: float = 5.0) -> float:
    """優化掃描角度（最小化掃描線數量）"""
    planner = CoveragePlanner()
    min_length = float('inf')
    best_angle = 0.0

    for angle in range(0, 180, int(angle_step)):
        params = CoverageParameters(spacing=spacing, angle=angle, pattern=ScanPattern.GRID)
        path = planner.plan_coverage(polygon, params)
        if len(path) > 1:
            length = sum(
                math.sqrt((path[i+1][0] - path[i][0])**2 + (path[i+1][1] - path[i][1])**2)
                for i in range(len(path) - 1)
            )
            if length < min_length:
                min_length = length
                best_angle = angle

    return best_angle