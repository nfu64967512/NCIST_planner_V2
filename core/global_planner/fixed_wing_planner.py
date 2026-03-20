"""
固定翼路徑規劃器
針對固定翼無人機特性設計的三階段任務規劃：
  Phase 1 - 起飛路徑（Takeoff）: MAVLink TAKEOFF + 爬升轉場
  Phase 2 - 主任務（Mission）  : 螺旋掃描 / 同心圓掃描（含轉彎半徑強制）
  Phase 3 - 五邊進場降落（Traffic Pattern + Landing）

轉彎半徑合規性檢查（TurnRadiusChecker）：
  - 根據機體性能（衝浪者: min_turn_radius=30m, max_bank=45°）
  - 計算所需傾斜角並與機體限制比較
  - 回傳 ok / warning / critical 狀態

參考機型：衝浪者 (Surfer)
  失速速度: 11 m/s, 巡航: 18 m/s, 最大: 28 m/s
  最小轉彎半徑: 30 m (@ 18 m/s, 45° bank)
  建議轉彎半徑: 50 m
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional


# ============================================================
# 機型性能規格資料庫
# ============================================================

VEHICLE_SPECS_DB: Dict[str, dict] = {
    "衝浪者 (Surfer)": {
        "stall_speed_mps":             11.0,
        "cruise_speed_mps":            18.0,
        "max_speed_mps":               28.0,
        "min_turn_radius_m":           30.0,
        "recommended_turn_radius_m":   50.0,
        "max_bank_angle_deg":          45.0,
        "takeoff_distance_m":          50.0,
        "approach_speed_mps":          14.0,
        "pattern_altitude_m":          80.0,
        "downwind_offset_m":           200.0,
        "pattern_leg_length_m":        300.0,
        "final_approach_dist_m":       400.0,
    },
    "Generic Fixed Wing": {
        "stall_speed_mps":             12.0,
        "cruise_speed_mps":            18.0,
        "max_speed_mps":               25.0,
        "min_turn_radius_m":           50.0,
        "recommended_turn_radius_m":   80.0,
        "max_bank_angle_deg":          45.0,
        "takeoff_distance_m":          30.0,
        "approach_speed_mps":          15.0,
        "pattern_altitude_m":          100.0,
        "downwind_offset_m":           250.0,
        "pattern_leg_length_m":        400.0,
        "final_approach_dist_m":       500.0,
    },
}


# ============================================================
# 參數結構
# ============================================================

@dataclass
class FixedWingParameters:
    """固定翼完整飛行參數"""

    # ── 基本飛行 ──────────────────────────────────────────────
    cruise_speed_mps: float = 18.0
    mission_altitude_m: float = 100.0

    # ── 轉彎性能 ──────────────────────────────────────────────
    turn_radius_m: float = 50.0          # 當前設定轉彎半徑（公尺）
    min_turn_radius_m: float = 30.0      # 機體最小轉彎半徑（公尺）
    max_bank_angle_deg: float = 45.0     # 機體最大傾斜角（度）

    # ── 起飛設定 ──────────────────────────────────────────────
    takeoff_bearing_deg: float = 0.0     # 起飛方向（0=北，順時針）
    takeoff_runway_m: float = 50.0       # 跑道長度（公尺）
    takeoff_climb_alt_m: float = 30.0    # 離地後達到的離場高度（公尺）

    # ── 五邊降落設定 ──────────────────────────────────────────
    landing_bearing_deg: float = 180.0   # 降落進場方向（飛機飛向跑道的方向）
    pattern_altitude_m: float = 80.0     # 五邊飛行高度 AGL（公尺）
    downwind_offset_m: float = 200.0     # 下風邊與跑道的側偏距離（公尺）
    pattern_leg_length_m: float = 300.0  # 下風邊腿長（公尺）
    final_approach_dist_m: float = 400.0 # 最終進場起始距離（公尺）

    # ── 任務掃描 ──────────────────────────────────────────────
    scan_spacing_m: float = 50.0         # 掃描間距
    circle_direction: int = 1            # 1=逆時針, -1=順時針


# ============================================================
# 轉彎半徑合規性檢查器
# ============================================================

class TurnRadiusChecker:
    """
    固定翼轉彎半徑合規性檢查

    物理公式：
        R = V² / (g × tan(φ))
    其中 R=轉彎半徑(m), V=速度(m/s), g=9.81, φ=傾斜角(rad)

    反算所需傾斜角：
        φ = atan(V² / (g × R))
    """

    G = 9.81  # 重力加速度 m/s²

    @classmethod
    def calc_turn_radius(cls, speed_mps: float, bank_deg: float) -> float:
        """根據速度和傾斜角計算轉彎半徑（公尺）"""
        bank_rad = math.radians(bank_deg)
        tan_bank = math.tan(bank_rad)
        if abs(tan_bank) < 1e-9:
            return float("inf")
        return speed_mps ** 2 / (cls.G * tan_bank)

    @classmethod
    def calc_required_bank(cls, speed_mps: float, turn_radius_m: float) -> float:
        """計算達到指定轉彎半徑所需傾斜角（度）"""
        if turn_radius_m <= 0:
            return 90.0
        return math.degrees(math.atan(speed_mps ** 2 / (cls.G * turn_radius_m)))

    @classmethod
    def check(cls,
              turn_radius_m: float,
              speed_mps: float,
              vehicle_model: str = "衝浪者 (Surfer)") -> dict:
        """
        檢查轉彎半徑是否符合指定機型規格

        返回 dict：
            status          : 'ok' | 'warning' | 'critical'
            compliant       : bool
            required_bank   : float  所需傾斜角（度）
            min_radius      : float  機體最小轉彎半徑（公尺）
            max_bank        : float  機體最大傾斜角（度）
            message         : str    說明訊息
            color           : str    '#4CAF50' / '#FF9800' / '#f44336'
        """
        specs = VEHICLE_SPECS_DB.get(vehicle_model, VEHICLE_SPECS_DB["Generic Fixed Wing"])
        min_radius = specs["min_turn_radius_m"]
        max_bank = specs["max_bank_angle_deg"]
        required_bank = cls.calc_required_bank(speed_mps, turn_radius_m)

        if turn_radius_m < min_radius:
            return {
                "status": "critical",
                "compliant": False,
                "required_bank": required_bank,
                "min_radius": min_radius,
                "max_bank": max_bank,
                "message": (
                    f"危險：轉彎半徑 {turn_radius_m:.0f} m 小於\n"
                    f"{vehicle_model} 最小限制 {min_radius:.0f} m！\n"
                    f"理論需要傾斜角 {required_bank:.1f}°，超過最大 {max_bank:.1f}°"
                ),
                "color": "#f44336",
            }
        elif required_bank > max_bank * 0.80:
            return {
                "status": "warning",
                "compliant": True,
                "required_bank": required_bank,
                "min_radius": min_radius,
                "max_bank": max_bank,
                "message": (
                    f"警告：轉彎半徑 {turn_radius_m:.0f} m 接近極限\n"
                    f"所需傾斜角 {required_bank:.1f}°（最大 {max_bank:.1f}°）\n"
                    f"建議增大至 {specs.get('recommended_turn_radius_m', min_radius*2):.0f} m 以上"
                ),
                "color": "#FF9800",
            }
        else:
            return {
                "status": "ok",
                "compliant": True,
                "required_bank": required_bank,
                "min_radius": min_radius,
                "max_bank": max_bank,
                "message": (
                    f"符合規格：轉彎半徑 {turn_radius_m:.0f} m\n"
                    f"所需傾斜角 {required_bank:.1f}°（最大 {max_bank:.1f}°）"
                ),
                "color": "#4CAF50",
            }


# ============================================================
# 座標輔助函式
# ============================================================

def _offset_latlon(lat: float, lon: float,
                   distance_m: float, bearing_deg: float) -> Tuple[float, float]:
    """
    從指定點沿 bearing_deg 方向移動 distance_m 公尺

    參數:
        lat, lon     : 起始點（度）
        distance_m   : 移動距離（公尺）
        bearing_deg  : 方向（度，0=北，順時針）

    返回: (new_lat, new_lon)
    """
    bearing_rad = math.radians(bearing_deg)
    dlat_per_m = 1.0 / 111_111.0
    dlon_per_m = 1.0 / (111_111.0 * math.cos(math.radians(lat)) + 1e-12)
    new_lat = lat + distance_m * math.cos(bearing_rad) * dlat_per_m
    new_lon = lon + distance_m * math.sin(bearing_rad) * dlon_per_m
    return new_lat, new_lon


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """計算兩點間地表距離（公尺，Haversine 公式）"""
    R = 6_371_000.0
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    dφ = math.radians(lat2 - lat1)
    dλ = math.radians(lon2 - lon1)
    a = math.sin(dφ / 2) ** 2 + math.cos(φ1) * math.cos(φ2) * math.sin(dλ / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _compute_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """計算兩點間羅盤方位角（度，0=北，順時針）"""
    lat1_r, lat2_r = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2_r)
    y = (math.cos(lat1_r) * math.sin(lat2_r)
         - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


# ============================================================
# 固定翼路徑規劃器
# ============================================================

class FixedWingPlanner:
    """
    固定翼三階段任務路徑規劃器

    三個階段：
      1. generate_takeoff_path()  : 起飛 + 爬升轉場
      2. CoveragePlanner 生成主任務（在 generate_full_mission 中呼叫）
      3. generate_landing_path()  : 五邊進場降落

    MAVLink 指令：
      22 NAV_TAKEOFF     : 起飛
      16 NAV_WAYPOINT    : 一般航點（param2 = 到達半徑/轉彎半徑）
      21 NAV_LAND        : 降落
      178 DO_CHANGE_SPEED: 速度指令
    """

    # ──────────────────────────────────────────────────────────
    # Phase 1：起飛路徑
    # ──────────────────────────────────────────────────────────

    def generate_takeoff_path(
        self,
        home_lat: float,
        home_lon: float,
        params: FixedWingParameters,
    ) -> List[Tuple[float, float, float]]:
        """
        生成起飛路徑（含跑道 + 爬升 + 轉場高度）

        路徑結構（每點為 (lat, lon, alt_m)）：
          [0] 起飛點（Home，地面）
          [1] 跑道末端（離地，到達 takeoff_climb_alt_m）  ← NAV_TAKEOFF
          [2] 爬升過渡點（繼續前進 + 開始爬升至任務高度）  ← NAV_WAYPOINT
          [3] 進入任務區前的等待點（任務高度）              ← NAV_WAYPOINT
        """
        bearing = params.takeoff_bearing_deg
        runway_m = params.takeoff_runway_m
        turn_r = params.turn_radius_m
        climb_alt = params.takeoff_climb_alt_m
        mission_alt = params.mission_altitude_m

        path: List[Tuple[float, float, float]] = []

        # [0] Home（地面）
        path.append((home_lat, home_lon, 0.0))

        # [1] 跑道末端（NAV_TAKEOFF 目標點，初始爬升高度）
        end_lat, end_lon = _offset_latlon(home_lat, home_lon, runway_m, bearing)
        path.append((end_lat, end_lon, climb_alt))

        # [2..N] 余弦漸進爬升：從 climb_alt 平滑上升至 mission_alt
        # 使用 ease-out 余弦：起初快速爬升，頂部平滑收斂
        CLIMB_STEPS = 5
        climb_span = 4.0 * turn_r          # 爬升段總水平距離
        for i in range(1, CLIMB_STEPS + 1):
            t = i / CLIMB_STEPS
            dist = runway_m + t * climb_span
            # ease-out cosine：alt_frac = 1 - cos(π·t/2)，t=0→0，t=1→1
            alt_frac = 1.0 - math.cos(math.pi * t / 2.0)
            alt = climb_alt + (mission_alt - climb_alt) * alt_frac
            pt_lat, pt_lon = _offset_latlon(home_lat, home_lon, dist, bearing)
            path.append((pt_lat, pt_lon, alt))

        return path

    # ──────────────────────────────────────────────────────────
    # Phase 3：五邊進場降落路徑
    # ──────────────────────────────────────────────────────────

    def generate_landing_path(
        self,
        landing_lat: float,
        landing_lon: float,
        params: FixedWingParameters,
    ) -> List[Tuple[float, float, float]]:
        """
        生成五邊進場降落路徑（右手飛行模式）

        五邊腿（每點為 (lat, lon, alt_m)）：
          [0] 入五邊點   - 下風邊起始（來自任務區）
          [1] 下風邊終點 - 轉底邊前的位置（pattern_altitude）
          [2] 底邊終點   - 轉最終進場前的位置（降至 40%）
          [3] 最終進場起 - Final Approach Start（降至 20%）
          [4] 觸地點     - NAV_LAND 目標（高度 0）

        幾何說明（右手模式）：
          - 跑道方向: landing_bearing_deg（飛機飛向跑道的方向）
          - 下風邊: 與跑道平行，位於跑道右側 downwind_offset_m
          - 下風邊方向: 與降落方向相反（從遠端飛向近端）
          - 底邊: 垂直連接下風邊與最終進場
        """
        runway_hdg = params.landing_bearing_deg   # 進場飛行方向
        opp_hdg    = (runway_hdg + 180.0) % 360.0 # 下風邊飛行方向
        right_hdg  = (runway_hdg + 90.0)  % 360.0 # 右側 90°（下風邊在此側）

        pat_alt  = params.pattern_altitude_m
        offset   = params.downwind_offset_m
        leg      = params.pattern_leg_length_m
        fin_dist = params.final_approach_dist_m

        # 觸地點（跑道頭）
        td_lat, td_lon = landing_lat, landing_lon

        # 最終進場起點（Final Approach Fix）
        # 在觸地點「後方」（沿進場反方向）fin_dist 公尺
        faf_lat, faf_lon = _offset_latlon(td_lat, td_lon, fin_dist, opp_hdg)

        # 底邊點（Base Turn Point）
        # 在最終進場起點的右側 offset 公尺
        base_lat, base_lon = _offset_latlon(faf_lat, faf_lon, offset, right_hdg)

        # 下風邊終點（Downwind → Base 轉彎點）
        # 與底邊等側偏，但沿跑道延伸方向 = base_lat/lon 即是
        downwind_end_lat, downwind_end_lon = base_lat, base_lon

        # 下風邊起點（入五邊點）
        # 從下風邊終點沿下風邊方向（opp_hdg）再延伸 leg 公尺
        downwind_start_lat, downwind_start_lon = _offset_latlon(
            downwind_end_lat, downwind_end_lon, leg, opp_hdg
        )

        path: List[Tuple[float, float, float]] = []

        # [0] 入五邊點（下風邊起始，等高飛行）
        path.append((downwind_start_lat, downwind_start_lon, pat_alt))

        # [1] 下風邊終點 = 底邊轉彎點（開始下降）
        path.append((downwind_end_lat, downwind_end_lon, pat_alt))

        # [2..N] 余弦漸進下降：底邊轉彎 → FAF → 觸地
        # 下降段幾何點：底邊起點、FAF、觸地點
        descent_pts = [
            (downwind_end_lat, downwind_end_lon),   # 下降起點（t=0）
            (faf_lat, faf_lon),                       # 最終進場起點
            (td_lat, td_lon),                         # 觸地點（t=1）
        ]
        seg_dists = [
            _haversine_m(descent_pts[i][0], descent_pts[i][1],
                         descent_pts[i + 1][0], descent_pts[i + 1][1])
            for i in range(len(descent_pts) - 1)
        ]
        total_d = sum(seg_dists) or 1.0

        def _desc_alt(cum_d: float) -> float:
            """余弦 ease-in-out：從 pat_alt 平滑降至 0"""
            t = min(cum_d / total_d, 1.0)
            return pat_alt * 0.5 * (1.0 + math.cos(math.pi * t))

        STEPS_PER_SEG = 3   # 每段的中間插值點數量
        cum = 0.0
        for seg_i in range(len(seg_dists)):
            lat1, lon1 = descent_pts[seg_i]
            lat2, lon2 = descent_pts[seg_i + 1]
            seg_len = seg_dists[seg_i]

            # 每段內的中間點
            for j in range(1, STEPS_PER_SEG + 1):
                t_seg = j / (STEPS_PER_SEG + 1)
                mid_lat = lat1 + t_seg * (lat2 - lat1)
                mid_lon = lon1 + t_seg * (lon2 - lon1)
                path.append((mid_lat, mid_lon, _desc_alt(cum + t_seg * seg_len)))

            # 段落終點（跳過最後一段終點，由觸地點單獨加入）
            if seg_i < len(seg_dists) - 1:
                cum += seg_len
                path.append((lat2, lon2, _desc_alt(cum)))

        # 觸地點（NAV_LAND，高度=0）
        path.append((td_lat, td_lon, 0.0))

        # 以內切圓弧平滑五邊進場的轉彎（下風→底邊、底邊→最終進場各約 90°）
        if params.turn_radius_m > 0:
            path = FixedWingPlanner.smooth_path_with_arcs(
                path, params.turn_radius_m
            )

        return path

    # ──────────────────────────────────────────────────────────
    # 完整三階段任務
    # ──────────────────────────────────────────────────────────

    def generate_full_mission(
        self,
        polygon: List[Tuple[float, float]],
        home_lat: float,
        home_lon: float,
        params: FixedWingParameters,
        scan_pattern: str = "spiral",
        scan_angle_deg: float = 0.0,
    ) -> dict:
        """
        生成完整固定翼三階段任務路徑

        參數:
            polygon        : 任務多邊形（經緯度列表）
            home_lat/lon   : 起降點座標
            params         : FixedWingParameters
            scan_pattern   : 'spiral' | 'circular' | 'grid'
            scan_angle_deg : 網格掃描角度（度，僅 grid 模式有效）

        返回 dict：
            takeoff_path   : [(lat, lon, alt), ...]
            mission_path   : [(lat, lon, alt), ...]  含高度
            mission_latlon : [(lat, lon), ...]        僅 lat/lon（供地圖顯示）
            landing_path   : [(lat, lon, alt), ...]
            full_path      : [(lat, lon), ...]        全程（供地圖連線顯示）
        """
        from core.global_planner.coverage_planner import (
            CoveragePlanner, CoverageParameters, ScanPattern,
        )

        # Phase 1：起飛路徑
        takeoff_path = self.generate_takeoff_path(home_lat, home_lon, params)

        # Phase 2：主任務掃描路徑
        if scan_pattern == "spiral":
            pattern = ScanPattern.SPIRAL
        elif scan_pattern == "grid":
            pattern = ScanPattern.GRID
        else:
            pattern = ScanPattern.CIRCULAR
        coverage_params = CoverageParameters(
            spacing=params.scan_spacing_m,
            angle=scan_angle_deg,
            pattern=pattern,
            is_fixed_wing=True,
            turn_radius=params.turn_radius_m,
            # Grid 模式由 generate_fw_grid_path() 自行產生 Omega 迴轉弧，
            # 不需要 coverage_planner 的 smooth_turns（那會在邊界內轉彎）
            smooth_turns=(scan_pattern != "grid"),
            circle_base_altitude=params.mission_altitude_m,
            circle_direction=params.circle_direction,
        )
        cov_planner = CoveragePlanner()
        mission_latlon_raw = cov_planner.plan_coverage(polygon, coverage_params)

        # 格子模式：插入固定翼 Omega 迴轉弧
        #   每條掃描線向兩端延伸 turn_radius（在多邊形邊界外轉彎）
        #   相鄰掃描線以兩個 90° 圓弧 + 直段連接（Omega 迴轉）
        if scan_pattern == "grid":
            mission_latlon = self.generate_fw_grid_path(
                mission_latlon_raw,
                params.turn_radius_m,
            )
        else:
            mission_latlon = mission_latlon_raw

        # 固定翼任務路徑內切圓弧平滑：取代所有銳角轉彎（含 spiral/circular/grid）
        mission_path = [
            (lat, lon, params.mission_altitude_m) for lat, lon in mission_latlon
        ]
        if mission_path and params.turn_radius_m > 0:
            mission_path = FixedWingPlanner.smooth_path_with_arcs(
                mission_path, params.turn_radius_m
            )
            mission_latlon = [(lat, lon) for lat, lon, _ in mission_path]

        # Phase 3：五邊降落路徑（以 home 作為降落觸地點）
        landing_path = self.generate_landing_path(home_lat, home_lon, params)

        # 合併所有點供地圖顯示
        full_path: List[Tuple[float, float]] = []
        full_path += [(lat, lon) for lat, lon, _ in takeoff_path]
        full_path += mission_latlon
        full_path += [(lat, lon) for lat, lon, _ in landing_path]

        return {
            "takeoff_path": takeoff_path,
            "mission_path": mission_path,
            "mission_latlon": mission_latlon,
            "landing_path": landing_path,
            "full_path": full_path,
        }

    # ──────────────────────────────────────────────────────────
    # 銳角轉彎過濾器（適用所有固定翼路徑模式）
    # ──────────────────────────────────────────────────────────

    @staticmethod
    def _remove_acute_turns(
        path: List[Tuple[float, float]],
        turn_radius_m: float,
        max_turn_deg: float = 90.0,
    ) -> List[Tuple[float, float]]:
        """
        固定翼路徑銳角轉彎過濾器（lat/lon 版本）。

        對連續航點間方向變化 > max_turn_deg 的轉角，在轉角前後各插入一個
        回退點（setback = turn_radius * 0.8），將銳角分解為兩段較緩轉彎。
        最多執行 3 次迭代直到不再有銳角。

        使用局部米制座標（lat/lon 差值乘以公尺換算係數），在小區域內
        近似直角坐標系，精度對百公尺量級路徑足夠。

        參數:
            path         : [(lat, lon), ...] 路徑航點
            turn_radius_m: 最小轉彎半徑（公尺），決定回退距離
            max_turn_deg : 允許最大轉向角（度）；超過此值視為銳角
        返回:
            過濾後的 [(lat, lon), ...] 路徑
        """
        if len(path) < 3 or turn_radius_m <= 0:
            return list(path)

        max_turn_rad = math.radians(max_turn_deg)
        setback = turn_radius_m * 0.8   # 回退距離，保留一些餘量

        pts = list(path)
        for _ in range(3):
            new_pts = [pts[0]]
            changed = False

            for i in range(1, len(pts) - 1):
                lat0, lon0 = pts[i - 1]
                lat1, lon1 = pts[i]
                lat2, lon2 = pts[i + 1]

                cos_lat = math.cos(math.radians(lat1))

                # 轉為局部米制向量（東=x, 北=y）
                dx_in  = (lon1 - lon0) * 111_111.0 * cos_lat
                dy_in  = (lat1 - lat0) * 111_111.0
                dx_out = (lon2 - lon1) * 111_111.0 * cos_lat
                dy_out = (lat2 - lat1) * 111_111.0

                len_in  = math.hypot(dx_in,  dy_in)
                len_out = math.hypot(dx_out, dy_out)

                if len_in < 1e-6 or len_out < 1e-6:
                    new_pts.append(pts[i])
                    continue

                ux_in,  uy_in  = dx_in  / len_in,  dy_in  / len_in
                ux_out, uy_out = dx_out / len_out, dy_out / len_out

                dot = max(-1.0, min(1.0, ux_in * ux_out + uy_in * uy_out))
                turn_angle = math.acos(dot)  # 0=直行, π=掉頭

                if (turn_angle > max_turn_rad
                        and setback < len_in  * 0.9
                        and setback < len_out * 0.9):
                    # 在轉角前/後各退 setback，插入兩個航點取代原銳角點
                    pre_lat  = lat1 - uy_in  * setback / 111_111.0
                    pre_lon  = lon1 - ux_in  * setback / (111_111.0 * cos_lat)
                    post_lat = lat1 + uy_out * setback / 111_111.0
                    post_lon = lon1 + ux_out * setback / (111_111.0 * cos_lat)
                    new_pts.append((pre_lat, pre_lon))
                    new_pts.append((post_lat, post_lon))
                    changed = True
                else:
                    new_pts.append(pts[i])

            new_pts.append(pts[-1])
            pts = new_pts
            if not changed:
                break

        return pts

    # ──────────────────────────────────────────────────────────
    # 內切圓弧平滑器（適用所有固定翼路徑）
    # ──────────────────────────────────────────────────────────

    @staticmethod
    def smooth_path_with_arcs(
        path: List[Tuple[float, float, float]],
        turn_radius_m: float,
        min_turn_deg: float = 5.0,
        pts_per_90deg: int = 6,
    ) -> List[Tuple[float, float, float]]:
        """
        對 3D 路徑所有轉彎點以內切圓弧取代尖角。

        原理（內切圓 / 切線過渡）：
          1. 計算切線距離 t = R × tan(|δ|/2)，δ 為轉向角（正=右轉）
          2. 在轉角前退 t 設切線入點，在轉角後進 t 設切線出點
          3. 以半徑 R 的圓弧從切線入點掃向切線出點，插入 n 個平滑點

        高度：弧段各點使用原轉角點的高度（弧長通常在幾十公尺內，高度誤差可忽略）。

        參數:
            path          : [(lat, lon, alt), ...] 3D 路徑
            turn_radius_m : 轉彎半徑（公尺）
            min_turn_deg  : 小於此角度的轉角不插弧（度）
            pts_per_90deg : 每 90° 弧插入的中間點數

        返回:
            [(lat, lon, alt), ...] 插入圓弧後的平滑路徑
        """
        if len(path) < 3 or turn_radius_m <= 0:
            return list(path)

        out: List[Tuple[float, float, float]] = [path[0]]

        for i in range(1, len(path) - 1):
            prev, cur, nxt = path[i - 1], path[i], path[i + 1]

            hdg_in  = _compute_bearing(prev[0], prev[1], cur[0], cur[1])
            hdg_out = _compute_bearing(cur[0], cur[1], nxt[0], nxt[1])
            delta   = ((hdg_out - hdg_in + 180) % 360) - 180  # [-180, 180]

            if abs(delta) < min_turn_deg:
                out.append(cur)
                continue

            # 切線距離，受相鄰段長度約束（各至多使用 45%）
            half_rad = math.radians(abs(delta) / 2.0)
            tan_dist = turn_radius_m * math.tan(half_rad)
            dist_in  = _haversine_m(prev[0], prev[1], cur[0], cur[1])
            dist_out = _haversine_m(cur[0], cur[1], nxt[0], nxt[1])
            actual_t = min(tan_dist, dist_in * 0.45, dist_out * 0.45)

            if actual_t < 1.0:
                out.append(cur)
                continue

            actual_R = actual_t / math.tan(half_rad)

            # 切線入點：在轉角之前，沿入射方向反向退 actual_t
            t_in_lat, t_in_lon = _offset_latlon(
                cur[0], cur[1], actual_t, (hdg_in + 180) % 360
            )

            # 轉彎方向：delta > 0 = 右轉，delta < 0 = 左轉
            turn_dir = 1 if delta > 0 else -1
            perp = (hdg_in + turn_dir * 90) % 360
            c_lat, c_lon = _offset_latlon(t_in_lat, t_in_lon, actual_R, perp)

            start_bear = (perp + 180) % 360
            n   = max(2, int(abs(delta) / 90.0 * pts_per_90deg))
            alt = cur[2]

            out.append((t_in_lat, t_in_lon, alt))
            for j in range(1, n + 1):
                sweep = turn_dir * abs(delta) * j / n
                bear  = (start_bear + sweep) % 360
                la, lo = _offset_latlon(c_lat, c_lon, actual_R, bear)
                out.append((la, lo, alt))
            # 最後弧段點即切線出點，無需再加

        out.append(path[-1])
        return out

    # ──────────────────────────────────────────────────────────
    # 進場弧：起飛末端 → 任務第一點的平滑銜接弧段
    # ──────────────────────────────────────────────────────────

    @staticmethod
    def generate_entry_arc(
        p_end: Tuple[float, float],
        h_end_deg: float,
        p_first: Tuple[float, float],
        turn_radius_m: float,
        mission_alt_m: float,
        min_turn_deg: float = 15.0,
    ) -> List[Tuple[float, float, float]]:
        """
        生成起飛路徑末端 → 任務第一點的平滑進場弧段。

        原理：以起飛末端為出發點，沿 h_end_deg 方向的垂直側偏一個轉彎半徑
        作為轉彎圓心，以圓弧平滑地從 h_end_deg 轉向 h_approach
        （從末端指向第一任務點的方位角）。

        參數:
            p_end         : 起飛路徑最後一點 (lat, lon)
            h_end_deg     : 起飛方向（度，0=北，順時針）
            p_first       : 任務第一點 (lat, lon)
            turn_radius_m : 固定翼轉彎半徑（公尺）
            mission_alt_m : 任務高度（公尺，弧段點高度）
            min_turn_deg  : 轉角門檻，小於此值則不插入弧段（度）

        返回:
            [(lat, lon, alt), ...] 弧段中間點，若轉角不足則為空列表
        """
        if turn_radius_m <= 0 or p_end == p_first:
            return []

        h_approach = _compute_bearing(p_end[0], p_end[1], p_first[0], p_first[1])
        delta = ((h_approach - h_end_deg + 180) % 360) - 180  # [-180, 180]

        if abs(delta) < min_turn_deg:
            return []

        turn_dir = 1 if delta > 0 else -1   # +1=右轉(順時針), -1=左轉(逆時針)
        perp = (h_end_deg + turn_dir * 90) % 360
        c_lat, c_lon = _offset_latlon(p_end[0], p_end[1], turn_radius_m, perp)

        # 圓心 → p_end 的方位（弧起始角）
        start_bear = (perp + 180) % 360

        n = max(3, int(abs(delta) / 15))    # 每 15° 插一個中間點
        arc: List[Tuple[float, float, float]] = []
        for i in range(1, n + 1):
            sweep = turn_dir * abs(delta) * i / n
            bear = (start_bear + sweep) % 360
            lat, lon = _offset_latlon(c_lat, c_lon, turn_radius_m, bear)
            arc.append((lat, lon, mission_alt_m))

        return arc

    # ──────────────────────────────────────────────────────────
    # 固定翼格子迴轉路徑生成
    # ──────────────────────────────────────────────────────────

    def generate_fw_grid_path(
        self,
        grid_waypoints: List[Tuple[float, float]],
        turn_radius_m: float,
        n_arc: int = 8,
    ) -> List[Tuple[float, float]]:
        """
        固定翼割草機格子航線生成。

        輸入：coverage_planner 輸出的扁平格子航點（成對出現，每對一條掃描線）

        處理流程：
          1. 每條掃描線向兩端各延伸 turn_radius_m（在多邊形邊界外轉彎）
          2. 第一條掃描線前加入進入點（供起飛後對齊使用）
          3. 相鄰掃描線之間插入 Omega 迴轉弧：
               90° 圓弧 → 直段（若間距 > 2R）→ 90° 圓弧

        返回：適合固定翼飛行的平滑迴轉路徑
        """
        if len(grid_waypoints) < 4:
            return list(grid_waypoints)

        R = turn_radius_m

        # 解析掃描線段（每兩點一條線）
        segs: List[Tuple[Tuple[float, float], Tuple[float, float]]] = [
            (grid_waypoints[i], grid_waypoints[i + 1])
            for i in range(0, len(grid_waypoints) - 1, 2)
        ]

        result: List[Tuple[float, float]] = []

        for idx, (p_start, p_end) in enumerate(segs):
            lat_ref = (p_start[0] + p_end[0]) / 2.0
            cos_lat = math.cos(math.radians(lat_ref))

            # 計算掃描線方向（米制單位向量）
            dy_m = (p_end[0] - p_start[0]) * 111_111.0
            dx_m = (p_end[1] - p_start[1]) * 111_111.0 * cos_lat
            seg_len = math.hypot(dx_m, dy_m)
            if seg_len < 1.0:
                continue
            ux, uy = dx_m / seg_len, dy_m / seg_len   # 單位向量 (東, 北)

            def _off(lat, lon, east_m, north_m, _cl=cos_lat):
                return (
                    lat + north_m / 111_111.0,
                    lon + east_m / (111_111.0 * _cl),
                )

            # 進入點（掃描線起點往後延伸 R，固定翼從此點對齊進入）
            app_pt  = _off(p_start[0], p_start[1], -ux * R, -uy * R)
            # 離開點（掃描線終點往前延伸 R，在邊界外再開始轉彎）
            exit_pt = _off(p_end[0],   p_end[1],    ux * R,  uy * R)

            if idx == 0:
                result.append(app_pt)    # 第一條線的進入點（起飛後對齊此點）

            result.append(p_start)
            result.append(p_end)
            result.append(exit_pt)

            # 生成轉彎弧銜接下一條掃描線
            if idx + 1 < len(segs):
                n_start, n_end = segs[idx + 1]
                n_lat_ref = (n_start[0] + n_end[0]) / 2.0
                n_cos_lat = math.cos(math.radians(n_lat_ref))
                n_dy_m = (n_end[0] - n_start[0]) * 111_111.0
                n_dx_m = (n_end[1] - n_start[1]) * 111_111.0 * n_cos_lat
                n_len = math.hypot(n_dx_m, n_dy_m)
                if n_len < 1.0:
                    continue
                n_ux, n_uy = n_dx_m / n_len, n_dy_m / n_len

                def _n_off(lat, lon, east_m, north_m, _cl=n_cos_lat):
                    return (
                        lat + north_m / 111_111.0,
                        lon + east_m / (111_111.0 * _cl),
                    )

                # 下一條掃描線的進入點
                n_app_pt = _n_off(n_start[0], n_start[1], -n_ux * R, -n_uy * R)

                # Omega 迴轉弧
                arc = self._omega_turn_arc(
                    exit_pt,  (ux, uy),
                    n_app_pt, (n_ux, n_uy),
                    R, lat_ref, n_arc,
                )
                result.extend(arc)

        return result

    def _omega_turn_arc(
        self,
        exit_pt: Tuple[float, float],
        exit_dir: Tuple[float, float],   # 離開方向單位向量 (東, 北)
        entry_pt: Tuple[float, float],
        entry_dir: Tuple[float, float],  # 進入方向單位向量 (東, 北)
        turn_radius_m: float,
        lat_ref: float,
        n_arc: int = 8,
    ) -> List[Tuple[float, float]]:
        """
        Omega 迴轉弧：兩個 90° 圓弧（+ 中間直段）

        連接 exit_pt（飛行方向 exit_dir）→ entry_pt（飛行方向 entry_dir）。
        適用於平行反向掃描線之間的轉彎（割草機模式）。

        幾何說明：
          • 間距 = 2R：純 180° U 形迴轉（兩弧合一）
          • 間距 > 2R：第一個 90° 弧 → 直段（往掃描線法方向移動）→ 第二個 90° 弧
        """
        R = turn_radius_m
        cos_lat = math.cos(math.radians(lat_ref))

        # 以 exit_pt 為原點的局部米制 XY（X=東, Y=北）
        def _to_xy(lat, lon):
            return (
                (lon - exit_pt[1]) * 111_111.0 * cos_lat,
                (lat - exit_pt[0]) * 111_111.0,
            )

        def _from_xy(x, y):
            return (
                exit_pt[0] + y / 111_111.0,
                exit_pt[1] + x / (111_111.0 * cos_lat),
            )

        ex_x, ex_y = 0.0, 0.0
        en_x, en_y = _to_xy(*entry_pt)
        ux, uy = exit_dir
        n_ux, n_uy = entry_dir

        # 決定轉彎方向（左/右）：以 entry 在哪一側判斷
        # CCW（左轉）垂直: (-uy,  ux)
        # CW （右轉）垂直: ( uy, -ux)
        dot_ccw = (en_x - ex_x) * (-uy) + (en_y - ex_y) * ux
        turn_sign = 1 if dot_ccw >= 0 else -1   # +1=CCW/左轉, -1=CW/右轉
        side_x = -uy * turn_sign
        side_y =  ux * turn_sign

        # ── 第一個 90° 圓弧（exit_pt → q1，離開方向轉為側方向）──────────
        c1x = ex_x + side_x * R
        c1y = ex_y + side_y * R
        a1_start = math.atan2(ex_y - c1y, ex_x - c1x)
        a1_end   = a1_start + turn_sign * math.pi / 2
        q1x = c1x + R * math.cos(a1_end)
        q1y = c1y + R * math.sin(a1_end)

        # ── 第二個 90° 圓弧（q2 → entry_pt，側方向轉為進入方向）──────────
        # 中心從 entry_pt 反推，選靠近 q1 的那側
        c2a = (en_x + (-n_uy) * R, en_y + n_ux * R)
        c2b = (en_x +  n_uy  * R, en_y - n_ux * R)
        if math.hypot(q1x - c2a[0], q1y - c2a[1]) <= math.hypot(q1x - c2b[0], q1y - c2b[1]):
            c2x, c2y, ts2 = c2a[0], c2a[1], 1
        else:
            c2x, c2y, ts2 = c2b[0], c2b[1], -1

        a2_end   = math.atan2(en_y - c2y, en_x - c2x)
        a2_start = a2_end - ts2 * math.pi / 2
        q2x = c2x + R * math.cos(a2_start)
        q2y = c2y + R * math.sin(a2_start)

        pts: List[Tuple[float, float]] = []

        # 第一個 90° 弧
        for k in range(1, n_arc + 1):
            a = a1_start + (k / n_arc) * turn_sign * math.pi / 2
            pts.append(_from_xy(c1x + R * math.cos(a), c1y + R * math.sin(a)))

        # 直段（間距 > 2R 時存在）
        straight_len = math.hypot(q2x - q1x, q2y - q1y)
        if straight_len > R * 0.05:
            n_seg = max(2, int(straight_len / (R * 0.5)))
            for k in range(1, n_seg):
                t = k / n_seg
                pts.append(_from_xy(q1x + t * (q2x - q1x), q1y + t * (q2y - q1y)))

        # 第二個 90° 弧
        for k in range(1, n_arc + 1):
            a = a2_start + (k / n_arc) * ts2 * math.pi / 2
            pts.append(_from_xy(c2x + R * math.cos(a), c2y + R * math.sin(a)))

        return pts

    # ──────────────────────────────────────────────────────────
    # MAVLink 航點生成（QGC WPL 110 格式）
    # ──────────────────────────────────────────────────────────

    def generate_mavlink_waypoints(
        self,
        mission_result: dict,
        params: FixedWingParameters,
        speed_mps: float = 18.0,
    ) -> List[str]:
        """
        將三階段路徑轉換為 Mission Planner / QGC WPL 110 格式

        指令對應：
          seq 0    : DO_SET_HOME (179)       - Home 佔位
          seq 1    : DO_CHANGE_SPEED (178)   - 空速設定
          seq 2    : NAV_TAKEOFF (22)        - 起飛到指定高度
          seq 3..n : NAV_WAYPOINT (16)       - 爬升 + 任務航點
                     param2 = turn_radius_m（告訴 APM 早轉彎）
          seq n+1..: NAV_WAYPOINT (16)       - 五邊各腿
          seq last : NAV_LAND (21)           - 降落觸地

        返回：
            ['QGC WPL 110', ...]  可直接 join('\n') 寫入 .waypoints
        """
        lines = ["QGC WPL 110"]
        seq = 0
        turn_r = params.turn_radius_m
        runway_hdg = params.takeoff_bearing_deg

        takeoff = mission_result.get("takeoff_path", [])
        mission = mission_result.get("mission_path", [])
        landing = mission_result.get("landing_path", [])

        def wp_line(seq, cmd, p1, p2, p3, p4, lat, lon, alt, cur=0, ac=1):
            return (
                f"{seq}\t{cur}\t3\t{cmd}\t"
                f"{p1:.4f}\t{p2:.4f}\t{p3:.4f}\t{p4:.4f}\t"
                f"{lat:.7f}\t{lon:.7f}\t{alt:.2f}\t{ac}"
            )

        # seq 0：Home
        home_lat = takeoff[0][0] if takeoff else 0.0
        home_lon = takeoff[0][1] if takeoff else 0.0
        lines.append(wp_line(seq, 179, 0, 0, 0, 0, home_lat, home_lon, 0))
        seq += 1

        # seq 1：DO_CHANGE_SPEED（空速）
        # cmd 178: param1=1(airspeed), param2=speed
        lines.append(wp_line(seq, 178, 1, speed_mps, 0, 0, 0, 0, 0))
        seq += 1

        # ── Phase 1: 起飛 + 爬升 ─────────────────────────────
        # [1] 跑道末端 → NAV_TAKEOFF
        # param1=爬升角度, param4=yaw(跑道方向)
        if len(takeoff) >= 2:
            t_lat, t_lon, t_alt = takeoff[1]
            lines.append(wp_line(seq, 22,
                                 15.0,        # param1: pitch (爬升角)
                                 0, 0,
                                 runway_hdg,  # param4: yaw
                                 t_lat, t_lon, t_alt))
            seq += 1

        # [2+] 爬升到任務高度
        for pt in takeoff[2:]:
            lat, lon, alt = pt
            # param2 = acceptance_radius（=turn_radius 讓飛機提前轉彎）
            lines.append(wp_line(seq, 16, 0, turn_r, 0, 0, lat, lon, alt))
            seq += 1

        # ── Phase 2: 任務掃描航點 ─────────────────────────────
        # acceptance_radius = min(turn_r, dist_to_next/3)
        # 避免圓形路徑密集航點時 acceptance_radius 過大導致跳點
        for idx, (lat, lon, alt) in enumerate(mission):
            if idx < len(mission) - 1:
                n_lat, n_lon, _ = mission[idx + 1]
                dist_next = _haversine_m(lat, lon, n_lat, n_lon)
                accept_r = max(turn_r / 10.0, min(dist_next / 3.0, turn_r))
            else:
                accept_r = turn_r
            lines.append(wp_line(seq, 16, 0, accept_r, 0, 0, lat, lon, alt))
            seq += 1

        # ── Phase 3: 五邊進場降落 ─────────────────────────────
        # [0~3] 各腿：NAV_WAYPOINT
        for lat, lon, alt in landing[:-1]:
            lines.append(wp_line(seq, 16, 0, turn_r, 0, 0, lat, lon, alt))
            seq += 1

        # [4] 觸地點：NAV_LAND (21)
        # param4 = yaw（對準跑道方向）
        if landing:
            l_lat, l_lon, l_alt = landing[-1]
            land_hdg = params.landing_bearing_deg
            lines.append(wp_line(seq, 21, 0, 0, 0, land_hdg, l_lat, l_lon, l_alt))
            seq += 1

        return lines

    # ──────────────────────────────────────────────────────────
    # 工具方法
    # ──────────────────────────────────────────────────────────

    @staticmethod
    def estimate_mission_stats(mission_result: dict, speed_mps: float) -> dict:
        """
        估算三階段任務統計

        返回 dict：
            takeoff_dist_m   : float
            mission_dist_m   : float
            landing_dist_m   : float
            total_dist_m     : float
            estimated_time_s : float
            waypoint_count   : int
        """
        def path_dist(pts):
            d = 0.0
            for i in range(len(pts) - 1):
                d += _haversine_m(pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1])
            return d

        takeoff_dist = path_dist(mission_result.get("takeoff_path", []))
        mission_dist = path_dist(mission_result.get("mission_path", []))
        landing_dist = path_dist(mission_result.get("landing_path", []))
        total = takeoff_dist + mission_dist + landing_dist
        wp_count = (
            len(mission_result.get("takeoff_path", []))
            + len(mission_result.get("mission_path", []))
            + len(mission_result.get("landing_path", []))
        )

        return {
            "takeoff_dist_m":   takeoff_dist,
            "mission_dist_m":   mission_dist,
            "landing_dist_m":   landing_dist,
            "total_dist_m":     total,
            "estimated_time_s": total / max(speed_mps, 0.1),
            "waypoint_count":   wp_count,
        }
