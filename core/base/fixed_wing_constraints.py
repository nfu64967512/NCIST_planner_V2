"""
固定翼無人機物理幾何約束模組
=================================
整合 PX4 L1 導航邏輯與 asl_fixedwing 最小轉彎半徑模型，
負責定義與驗證固定翼飛行器的運動學限制。

核心公式：
    R_min = V_a² / (g · tan(φ_max))

    來源：經典航空動力學穩態協調轉彎 (Steady Coordinated Turn)，
    假設升力分量完全補償離心力。PX4 中對應參數為
    FW_R_LIM (Bank Angle Limit) 與 FW_AIRSPD_TRIM (Trim Airspeed)。

作者: NCIST_planner_V1
版本: 1.0.0
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Sequence
from enum import Enum, auto

from utils.math_utils import (
    deg_to_rad, rad_to_deg, haversine_distance,
    bearing_between_points, latlon_to_meters, meters_to_latlon,
    normalize_angle
)


# ==========================================
# 風場介面 (預留擴充)
# ==========================================
class WindModel(Enum):
    """風場模型類型枚舉"""
    CALM = auto()          # 無風
    CONSTANT = auto()      # 定常風
    GRID = auto()          # 網格風場 (未來)


@dataclass(frozen=True)
class WindVector:
    """
    風向量資料類。

    Attributes:
        speed_mps: 風速 [m/s]
        direction_deg: 風來向 (氣象慣例，0=北，順時針) [度]
    """
    speed_mps: float = 0.0
    direction_deg: float = 0.0

    @property
    def headwind_component(self) -> float:
        """返回正北方向的頂風分量 (正值=頂風)"""
        return self.speed_mps * math.cos(deg_to_rad(self.direction_deg))

    @property
    def crosswind_component(self) -> float:
        """返回東向的側風分量"""
        return self.speed_mps * math.sin(deg_to_rad(self.direction_deg))


# ==========================================
# 可行性檢查結果
# ==========================================
@dataclass
class FeasibilityResult:
    """
    路徑可行性檢查結果。

    Attributes:
        is_feasible: 路徑整體是否可行
        violations: 違規航段列表 [(段索引, 所需半徑, 允許半徑, 轉折角度)]
        min_required_radius: 路徑中最嚴苛的所需轉彎半徑
        total_segments: 總航段數
    """
    is_feasible: bool
    violations: List[Tuple[int, float, float, float]] = field(default_factory=list)
    min_required_radius: float = float('inf')
    total_segments: int = 0

    def summary(self) -> str:
        """產生可讀的檢查摘要"""
        if self.is_feasible:
            return (
                f"✓ 路徑可行 | 共 {self.total_segments} 段 | "
                f"最嚴苛所需半徑: {self.min_required_radius:.1f} m"
            )
        lines = [
            f"✗ 路徑不可行 | {len(self.violations)}/{self.total_segments} 段違規"
        ]
        for seg_idx, r_need, r_allow, angle in self.violations:
            lines.append(
                f"  段 {seg_idx}: 轉折角 {angle:.1f}° → "
                f"所需 R={r_need:.1f} m > 允許 R={r_allow:.1f} m"
            )
        return "\n".join(lines)


# ==========================================
# 核心約束類別
# ==========================================
class FixedWingConstraints:
    """
    固定翼無人機物理幾何約束引擎。

    整合 PX4 導航參數設計，負責：
    1. 基於空速與側傾角動態計算最小轉彎半徑
    2. 驗證航點序列的幾何可行性
    3. 預留風場修正介面

    PX4 對應參數映射:
        V_a       → FW_AIRSPD_TRIM
        φ_max     → FW_R_LIM
        R_min     → 內部 L1 控制器計算值

    Args:
        cruise_airspeed_mps: 巡航空速 V_a [m/s]，對應 PX4 FW_AIRSPD_TRIM
        max_bank_angle_deg: 最大側傾角 φ_max [度]，對應 PX4 FW_R_LIM
        gravity: 重力加速度 [m/s²]
        safety_factor: 安全係數 (>= 1.0)，放大理論 R_min
        stall_speed_mps: 失速速度 [m/s]，用於速度下界保護
        max_speed_mps: 最大空速 [m/s]，用於速度上界保護
    """

    __slots__ = (
        '_cruise_airspeed', '_max_bank_angle_rad', '_max_bank_angle_deg',
        '_gravity', '_safety_factor', '_stall_speed', '_max_speed',
        '_wind', '_wind_model'
    )

    def __init__(
        self,
        cruise_airspeed_mps: float = 18.0,
        max_bank_angle_deg: float = 45.0,
        gravity: float = 9.81,
        safety_factor: float = 1.2,
        stall_speed_mps: float = 12.0,
        max_speed_mps: float = 25.0,
    ) -> None:
        if cruise_airspeed_mps <= 0:
            raise ValueError(f"巡航空速必須 > 0, 收到 {cruise_airspeed_mps}")
        if not (0 < max_bank_angle_deg < 90):
            raise ValueError(f"側傾角必須在 (0, 90) 度, 收到 {max_bank_angle_deg}")
        if safety_factor < 1.0:
            raise ValueError(f"安全係數必須 >= 1.0, 收到 {safety_factor}")
        if stall_speed_mps >= cruise_airspeed_mps:
            raise ValueError("失速速度必須 < 巡航空速")

        self._cruise_airspeed: float = cruise_airspeed_mps
        self._max_bank_angle_deg: float = max_bank_angle_deg
        self._max_bank_angle_rad: float = deg_to_rad(max_bank_angle_deg)
        self._gravity: float = gravity
        self._safety_factor: float = safety_factor
        self._stall_speed: float = stall_speed_mps
        self._max_speed: float = max_speed_mps

        # 風場 (預留)
        self._wind: WindVector = WindVector()
        self._wind_model: WindModel = WindModel.CALM

    # ------------------------------------------
    # 唯讀屬性
    # ------------------------------------------
    @property
    def cruise_airspeed(self) -> float:
        """巡航空速 V_a [m/s]"""
        return self._cruise_airspeed

    @property
    def max_bank_angle_deg(self) -> float:
        """最大側傾角 [度]"""
        return self._max_bank_angle_deg

    @property
    def max_bank_angle_rad(self) -> float:
        """最大側傾角 [弧度]"""
        return self._max_bank_angle_rad

    @property
    def gravity(self) -> float:
        """重力加速度 [m/s²]"""
        return self._gravity

    @property
    def safety_factor(self) -> float:
        """安全係數"""
        return self._safety_factor

    @property
    def stall_speed(self) -> float:
        """失速速度 [m/s]"""
        return self._stall_speed

    @property
    def max_speed(self) -> float:
        """最大空速 [m/s]"""
        return self._max_speed

    @property
    def wind(self) -> WindVector:
        """當前風場向量"""
        return self._wind

    # ------------------------------------------
    # 動態半徑計算
    # ------------------------------------------
    def get_min_turn_radius(
        self,
        airspeed_mps: Optional[float] = None,
        bank_angle_deg: Optional[float] = None,
        apply_safety: bool = True,
    ) -> float:
        """
        計算最小轉彎半徑。

        公式:
            R_min = V_a² / (g · tan(φ))

        推導: 穩態協調轉彎中，升力水平分量提供向心力:
            L · sin(φ) = m · V² / R
            L · cos(φ) = m · g
            → tan(φ) = V² / (g · R)
            → R = V² / (g · tan(φ))

        Args:
            airspeed_mps: 覆寫空速 [m/s]，None 則使用巡航空速
            bank_angle_deg: 覆寫側傾角 [度]，None 則使用最大側傾角
            apply_safety: 是否套用安全係數

        Returns:
            最小轉彎半徑 [m]
        """
        va = airspeed_mps if airspeed_mps is not None else self._cruise_airspeed
        phi = (deg_to_rad(bank_angle_deg)
               if bank_angle_deg is not None
               else self._max_bank_angle_rad)

        if va <= 0:
            raise ValueError(f"空速必須 > 0, 收到 {va}")
        if phi <= 0 or phi >= math.pi / 2:
            raise ValueError(f"側傾角弧度必須在 (0, π/2), 收到 {phi}")

        tan_phi = math.tan(phi)
        r_min = (va ** 2) / (self._gravity * tan_phi)

        if apply_safety:
            r_min *= self._safety_factor

        return r_min

    def get_turn_radius_with_wind(
        self,
        heading_deg: float,
        airspeed_mps: Optional[float] = None,
    ) -> float:
        """
        考慮風場修正後的等效轉彎半徑 (預留介面)。

        在有風條件下，地速 ≠ 空速。順風時地速增大，轉彎半徑
        相對地面會變大；逆風反之。此方法計算地面座標系中的
        等效轉彎半徑。

        簡化模型:
            V_ground ≈ V_air + W · cos(heading - wind_dir)
            R_ground = R_air · (V_ground / V_air)

        Args:
            heading_deg: 飛行航向 [度]，真北為 0
            airspeed_mps: 覆寫空速 [m/s]

        Returns:
            風場修正後的等效轉彎半徑 [m]
        """
        va = airspeed_mps if airspeed_mps is not None else self._cruise_airspeed
        r_air = self.get_min_turn_radius(airspeed_mps=va)

        if self._wind_model == WindModel.CALM or self._wind.speed_mps < 0.1:
            return r_air

        # 計算航向與風向的夾角 → 順逆風分量
        relative_angle = deg_to_rad(heading_deg - self._wind.direction_deg)
        wind_along = self._wind.speed_mps * math.cos(relative_angle)
        v_ground = va + wind_along  # 簡化: 僅沿航向投影

        if v_ground <= 0:
            v_ground = self._stall_speed * 0.5  # 極端情況保護

        r_ground = r_air * (v_ground / va)
        return r_ground

    # ------------------------------------------
    # 路徑可行性驗證
    # ------------------------------------------
    def is_path_executable(
        self,
        points: Sequence[Tuple[float, float]],
        coord_type: str = "latlon",
        airspeed_mps: Optional[float] = None,
    ) -> FeasibilityResult:
        """
        驗證航點序列的幾何可行性。

        對每個連續三點 (P_{i-1}, P_i, P_{i+1})，計算轉折處的
        內切圓半徑需求，與飛行器的最小轉彎半徑比較。

        幾何原理:
            兩線段 AB, BC 的夾角 θ 處，飛行器需要的最小內切圓半徑為:
            R_required = R_min / sin(θ/2)
            當 θ 趨近 0 (銳角折返)，R_required → ∞，不可行。
            當 θ = 180° (直線)，R_required = R_min，最寬鬆。

            但更直觀的判斷: 轉折角 α = 180° - θ (偏轉角)，
            需要的迴旋半徑: R_need = R_min / tan(α/2)
            α 越大 (越急轉)，R_need 越大。

        Args:
            points: 航點序列 [(lat, lon), ...] 或 [(x, y), ...]
            coord_type: "latlon" (地理座標) 或 "metric" (公尺座標)
            airspeed_mps: 覆寫空速

        Returns:
            FeasibilityResult 包含可行性判定與違規細節
        """
        if len(points) < 3:
            return FeasibilityResult(
                is_feasible=True, total_segments=max(0, len(points) - 1)
            )

        r_min = self.get_min_turn_radius(airspeed_mps=airspeed_mps)
        violations: List[Tuple[int, float, float, float]] = []
        min_required = float('inf')

        # 轉換為公尺座標以統一計算
        if coord_type == "latlon":
            ref_lat, ref_lon = points[0]
            metric_pts = [
                latlon_to_meters(lat, lon, ref_lat, ref_lon)
                for lat, lon in points
            ]
        else:
            metric_pts = list(points)

        total_segs = len(metric_pts) - 1

        for i in range(1, len(metric_pts) - 1):
            p_prev = metric_pts[i - 1]
            p_curr = metric_pts[i]
            p_next = metric_pts[i + 1]

            deflection_deg = self._compute_deflection_angle(
                p_prev, p_curr, p_next
            )
            abs_deflection = abs(deflection_deg)

            if abs_deflection < 1.0:
                # 近乎直線，無需轉彎
                continue

            # 所需半徑: 偏轉角越大，需要的迴旋空間越大
            half_deflect_rad = deg_to_rad(abs_deflection / 2.0)
            tan_half = math.tan(half_deflect_rad)

            if tan_half < 1e-6:
                r_required = float('inf')
            else:
                r_required = r_min / tan_half

            min_required = min(min_required, r_required)

            # 檢查兩側線段長度是否足以容納轉彎弧
            seg_a = math.hypot(
                p_curr[0] - p_prev[0], p_curr[1] - p_prev[1]
            )
            seg_b = math.hypot(
                p_next[0] - p_curr[0], p_next[1] - p_curr[1]
            )
            tangent_length = r_min / tan_half if tan_half > 1e-6 else float('inf')

            if tangent_length > seg_a / 2 or tangent_length > seg_b / 2:
                violations.append((i, r_required, r_min, abs_deflection))

        return FeasibilityResult(
            is_feasible=len(violations) == 0,
            violations=violations,
            min_required_radius=min_required if min_required != float('inf') else 0.0,
            total_segments=total_segs,
        )

    # ------------------------------------------
    # 風場設定
    # ------------------------------------------
    def set_wind(
        self,
        speed_mps: float = 0.0,
        direction_deg: float = 0.0,
        model: WindModel = WindModel.CONSTANT,
    ) -> None:
        """
        設定風場條件。

        Args:
            speed_mps: 風速 [m/s]
            direction_deg: 風來向 [度]
            model: 風場模型類型
        """
        self._wind = WindVector(speed_mps=speed_mps, direction_deg=direction_deg)
        self._wind_model = model if speed_mps > 0.1 else WindModel.CALM

    # ------------------------------------------
    # 速度邊界保護
    # ------------------------------------------
    def clamp_airspeed(self, desired_mps: float) -> float:
        """
        將期望空速限制在安全飛行包線內。

        Args:
            desired_mps: 期望空速 [m/s]

        Returns:
            夾限後的安全空速 [m/s]
        """
        return max(self._stall_speed * 1.1, min(desired_mps, self._max_speed))

    # ------------------------------------------
    # 輔助: Dubins 轉彎所需距離估算
    # ------------------------------------------
    def estimate_dubins_distance(
        self,
        start: Tuple[float, float],
        start_heading_deg: float,
        end: Tuple[float, float],
        end_heading_deg: float,
        coord_type: str = "latlon",
    ) -> float:
        """
        估算 Dubins 路徑長度下界 (CSC 模型)。

        Dubins 路徑是固定翼在最小轉彎半徑約束下的最短路徑，
        由兩段圓弧 (C) 和一段直線 (S) 組成。此方法提供
        快速下界估算，供路徑規劃器做成本評估。

        Args:
            start: 起點座標
            start_heading_deg: 起始航向 [度]
            end: 終點座標
            end_heading_deg: 終止航向 [度]
            coord_type: "latlon" 或 "metric"

        Returns:
            Dubins 路徑長度下界 [m]
        """
        if coord_type == "latlon":
            straight_dist = haversine_distance(
                start[0], start[1], end[0], end[1]
            )
        else:
            straight_dist = math.hypot(
                end[0] - start[0], end[1] - start[1]
            )

        r = self.get_min_turn_radius()

        # 航向差
        heading_diff = abs(normalize_angle(end_heading_deg - start_heading_deg))
        turn_arc = deg_to_rad(heading_diff) * r

        # Dubins 下界 ≈ 直線距離 + 轉向弧長
        return max(straight_dist, straight_dist + turn_arc - 2 * r)

    # ------------------------------------------
    # 內部方法
    # ------------------------------------------
    @staticmethod
    def _compute_deflection_angle(
        p_prev: Tuple[float, float],
        p_curr: Tuple[float, float],
        p_next: Tuple[float, float],
    ) -> float:
        """
        計算三點構成的偏轉角 (deflection angle)。

        偏轉角定義: 從 AB 方向到 BC 方向的轉向角度。
        0° = 直行，±180° = 完全折返。

        Args:
            p_prev, p_curr, p_next: 連續三個航點 (x, y) 公尺座標

        Returns:
            偏轉角 [度]，正值=右轉，負值=左轉
        """
        # 向量 AB 和 BC
        ax = p_curr[0] - p_prev[0]
        ay = p_curr[1] - p_prev[1]
        bx = p_next[0] - p_curr[0]
        by = p_next[1] - p_curr[1]

        # 各自的方位角
        heading_ab = math.atan2(ax, ay)  # 注意: atan2(x,y) 對應北基準
        heading_bc = math.atan2(bx, by)

        # 偏轉角
        deflection = heading_bc - heading_ab

        # 正規化到 [-π, π]
        while deflection > math.pi:
            deflection -= 2 * math.pi
        while deflection < -math.pi:
            deflection += 2 * math.pi

        return rad_to_deg(deflection)

    # ------------------------------------------
    # 工廠方法
    # ------------------------------------------
    @classmethod
    def from_yaml_profile(cls, profile: dict) -> FixedWingConstraints:
        """
        從 vehicle_profiles.yaml 的固定翼 profile 字典建立實例。

        對應 yaml 路徑: fixed_wing.generic_fixed_wing

        Args:
            profile: yaml 解析後的字典

        Returns:
            FixedWingConstraints 實例
        """
        return cls(
            cruise_airspeed_mps=profile.get('cruise_speed_mps', 18.0),
            max_bank_angle_deg=profile.get('max_bank_angle_deg', 45),
            stall_speed_mps=profile.get('stall_speed_mps', 12.0),
            max_speed_mps=profile.get('max_speed_mps', 25.0),
            safety_factor=profile.get('safety_factor', 1.2),
        )

    # ------------------------------------------
    # 資訊輸出
    # ------------------------------------------
    def __repr__(self) -> str:
        r = self.get_min_turn_radius()
        return (
            f"FixedWingConstraints("
            f"Va={self._cruise_airspeed}m/s, "
            f"φ_max={self._max_bank_angle_deg}°, "
            f"R_min={r:.1f}m, "
            f"SF={self._safety_factor})"
        )

    def info(self) -> str:
        """輸出完整參數報告"""
        r_theory = self.get_min_turn_radius(apply_safety=False)
        r_safe = self.get_min_turn_radius(apply_safety=True)
        return (
            f"═══ 固定翼約束參數 ═══\n"
            f"  巡航空速:     {self._cruise_airspeed:.1f} m/s\n"
            f"  失速速度:     {self._stall_speed:.1f} m/s\n"
            f"  最大空速:     {self._max_speed:.1f} m/s\n"
            f"  最大側傾角:   {self._max_bank_angle_deg:.1f}°\n"
            f"  理論 R_min:   {r_theory:.1f} m\n"
            f"  安全 R_min:   {r_safe:.1f} m (SF={self._safety_factor})\n"
            f"  風場:         {self._wind_model.name} "
            f"({self._wind.speed_mps:.1f} m/s @ {self._wind.direction_deg:.0f}°)"
        )
