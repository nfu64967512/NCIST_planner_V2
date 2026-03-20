"""
固定翼進階掃描模式 — 螺旋 & 同心圓
====================================
擴充 CoveragePathPlanner，新增螺旋內縮 (Inward Spiral) 與
同心圓 (Concentric Circle) 兩種掃描模式，專為固定翼最小
轉彎半徑約束設計。

設計理念:
    Zigzag 掃描在窄間距時需要大量 Bulb Turn，效率低。
    螺旋與同心圓模式天然利用曲線路徑，曲率可控，
    特別適合:
    - 圓形/橢圓形偵察區域
    - 需要持續視線覆蓋的目標 (如森林火場外圍)
    - 固定翼 R_min 較大、轉彎代價高的場景

    參考 NCIST 計畫文件中的 State Lattice 思路:
    在固定空速下，沿路徑每個離散點的曲率 κ ≤ 1/R_min，
    確保無過激動作。螺旋的曲率隨半徑遞減天然滿足此條件
    (只要起始半徑 > R_min)。

依賴:
    - fixed_wing_constraints.py → FixedWingConstraints
    - dubins_trajectory.py → DubinsTrajectoryGenerator, Pose3D
    - math_utils.py → 幾何工具
    - coverage_path_planner.py → CoveragePathPlanner (可選對接)

作者: NCIST_planner_V1
版本: 1.0.0
"""

from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Sequence
from enum import Enum, auto

from core.base.fixed_wing_constraints import FixedWingConstraints
from core.trajectory.dubins_trajectory import DubinsTrajectoryGenerator, Pose3D, DubinsPath3D
from utils.math_utils import deg_to_rad, rad_to_deg, normalize_angle


# ==========================================
# 列舉與資料結構
# ==========================================
class ScanPattern(Enum):
    """掃描模式列舉"""
    ZIGZAG = auto()
    SPIRAL_INWARD = auto()
    SPIRAL_OUTWARD = auto()
    CONCENTRIC = auto()


@dataclass
class SpiralParams:
    """
    螺旋掃描參數。

    Attributes:
        center: 螺旋中心 (x, y) [m]
        max_radius: 外圍半徑 [m]
        spacing: 圈間距 [m] (等於掃描線間距)
        direction: 1=逆時針, -1=順時針
        start_angle_deg: 起始角度 [度]
    """
    center: Tuple[float, float] = (0.0, 0.0)
    max_radius: float = 500.0
    spacing: float = 50.0
    direction: int = 1
    start_angle_deg: float = 0.0


@dataclass
class ConcentricParams:
    """
    同心圓掃描參數。

    Attributes:
        center: 圓心 (x, y) [m]
        max_radius: 最外圈半徑 [m]
        spacing: 圈間距 [m]
        arc_step_deg: 離散化角度步長 [度]
    """
    center: Tuple[float, float] = (0.0, 0.0)
    max_radius: float = 500.0
    spacing: float = 50.0
    arc_step_deg: float = 5.0


@dataclass
class AdvancedScanResult:
    """
    進階掃描結果。

    Attributes:
        pattern: 使用的掃描模式
        poses: 3D 姿態序列
        transition_paths: 圈間 Dubins 過渡路徑
        total_scan_length: 掃描段總長 [m]
        total_transition_length: 過渡段總長 [m]
        total_length: 全路徑長 [m]
        num_rings: 螺旋/同心圓圈數
        min_curvature_radius: 路徑中最小曲率半徑 [m]
        is_feasible: 是否全程滿足 R_min
        warnings: 警告列表
    """
    pattern: ScanPattern = ScanPattern.SPIRAL_INWARD
    poses: List[Pose3D] = field(default_factory=list)
    transition_paths: List[DubinsPath3D] = field(default_factory=list)
    total_scan_length: float = 0.0
    total_transition_length: float = 0.0
    total_length: float = 0.0
    num_rings: int = 0
    min_curvature_radius: float = float('inf')
    is_feasible: bool = True
    warnings: List[str] = field(default_factory=list)

    def summary(self) -> str:
        mins = (self.total_length / 18.0) / 60.0  # 假設 18 m/s
        return (
            f"═══ {self.pattern.name} 掃描結果 ═══\n"
            f"  圈數:         {self.num_rings}\n"
            f"  掃描總長:     {self.total_scan_length:.0f} m\n"
            f"  過渡總長:     {self.total_transition_length:.0f} m\n"
            f"  路徑總長:     {self.total_length:.0f} m\n"
            f"  最小曲率半徑: {self.min_curvature_radius:.1f} m\n"
            f"  可行性:       {'✓' if self.is_feasible else '✗'}\n"
            f"  航點數:       {len(self.poses)}\n"
            f"  警告:         {len(self.warnings)} 條"
        )


# ==========================================
# 進階掃描模式生成器
# ==========================================
class AdvancedScanGenerator:
    """
    固定翼進階掃描模式生成器。

    產出螺旋與同心圓路徑，保證曲率約束，
    並透過 DubinsTrajectoryGenerator 處理圈間過渡。

    Args:
        constraints: 固定翼物理約束
        dubins_gen: Dubins 生成器 (可選)
        altitude_m: 飛行高度 [m]
    """

    __slots__ = (
        '_constraints', '_dubins_gen', '_altitude', '_r_min'
    )

    def __init__(
        self,
        constraints: FixedWingConstraints,
        dubins_gen: Optional[DubinsTrajectoryGenerator] = None,
        altitude_m: float = 100.0,
    ) -> None:
        self._constraints = constraints
        self._dubins_gen = dubins_gen or DubinsTrajectoryGenerator(constraints)
        self._altitude = altitude_m
        self._r_min = constraints.get_min_turn_radius()

    # --- 唯讀屬性 ---
    @property
    def constraints(self) -> FixedWingConstraints:
        return self._constraints

    @property
    def r_min(self) -> float:
        return self._r_min

    @property
    def altitude(self) -> float:
        return self._altitude

    # ==========================================
    # 公開: 螺旋掃描
    # ==========================================
    def generate_spiral(
        self,
        params: SpiralParams,
        inward: bool = True,
    ) -> AdvancedScanResult:
        """
        生成阿基米德螺旋掃描路徑。

        阿基米德螺旋方程:
            r(θ) = a + b·θ
            其中 b = spacing / (2π)，控制圈間距

        曲率分析:
            螺旋曲率 κ(θ) = (r² + 2(dr/dθ)² - r·d²r/dθ²) / (r² + (dr/dθ)²)^(3/2)
            對阿基米德螺旋簡化為:
            κ ≈ 1/r (當 r >> b 時)
            因此只要 r > R_min，曲率自動滿足約束。
            內圈 r 接近 R_min 時需截斷或增大間距。

        Args:
            params: 螺旋參數
            inward: True=由外向內收縮, False=由內向外擴張

        Returns:
            AdvancedScanResult
        """
        result = AdvancedScanResult(
            pattern=ScanPattern.SPIRAL_INWARD if inward else ScanPattern.SPIRAL_OUTWARD
        )

        R = self._r_min
        cx, cy = params.center
        spacing = max(params.spacing, 1.0)
        alt = self._altitude

        # 螺旋參數: r(θ) = a + b·θ
        # b = spacing / (2π)
        b = spacing / (2.0 * math.pi)

        # 內圈截斷半徑: 不能小於 R_min (否則曲率超限)
        r_inner = max(R * 1.1, spacing)  # 留 10% 餘量
        r_outer = params.max_radius

        if r_outer <= r_inner:
            result.warnings.append(
                f"外圍半徑 {r_outer:.0f}m <= 內圈截斷 {r_inner:.0f}m，無法生成螺旋"
            )
            result.is_feasible = False
            return result

        # 總圈數
        num_rings = int(math.ceil((r_outer - r_inner) / spacing))
        result.num_rings = num_rings

        # 總角度範圍
        theta_total = (r_outer - r_inner) / b

        # 角度步長 (依空間解析度，約每 5m 一個點)
        step_length = 5.0
        theta_list: List[float] = []
        theta = 0.0

        while theta <= theta_total:
            theta_list.append(theta)
            r_current = r_inner + b * theta if not inward else r_outer - b * theta
            r_current = max(r_current, r_inner)
            # 弧長步長 → 角度步長: ds = r·dθ → dθ = ds/r
            d_theta = step_length / max(r_current, 1.0)
            theta += d_theta

        # 確保包含末尾
        if not theta_list or theta_list[-1] < theta_total:
            theta_list.append(theta_total)

        # 生成座標
        dir_sign = float(params.direction)
        start_angle = deg_to_rad(params.start_angle_deg)
        min_curvature_r = float('inf')
        total_length = 0.0
        prev_x, prev_y = None, None

        for theta in theta_list:
            if inward:
                r = r_outer - b * theta
            else:
                r = r_inner + b * theta

            r = max(r, r_inner)
            r = min(r, r_outer)

            angle = start_angle + dir_sign * theta
            x = cx + r * math.cos(angle)
            y = cy + r * math.sin(angle)

            # 航向 = 切線方向
            # 螺旋切線: dx/dθ, dy/dθ
            dr_dtheta = -b if inward else b
            dx_dtheta = dr_dtheta * math.cos(angle) - r * math.sin(angle) * dir_sign
            dy_dtheta = dr_dtheta * math.sin(angle) + r * math.cos(angle) * dir_sign
            heading_rad = math.atan2(dy_dtheta, dx_dtheta)
            heading_deg = rad_to_deg(heading_rad)

            result.poses.append(Pose3D(x, y, alt, normalize_angle(heading_deg, 0.0)))

            # 曲率半徑追蹤
            if r > 0:
                min_curvature_r = min(min_curvature_r, r)

            # 累計長度
            if prev_x is not None:
                total_length += math.hypot(x - prev_x, y - prev_y)
            prev_x, prev_y = x, y

        result.total_scan_length = total_length
        result.total_length = total_length
        result.min_curvature_radius = min_curvature_r

        if min_curvature_r < R:
            result.is_feasible = False
            result.warnings.append(
                f"最小曲率半徑 {min_curvature_r:.1f}m < R_min {R:.1f}m"
            )

        return result

    # ==========================================
    # 公開: 同心圓掃描
    # ==========================================
    def generate_concentric(
        self,
        params: ConcentricParams,
    ) -> AdvancedScanResult:
        """
        生成同心圓掃描路徑。

        每圈為完整圓弧，圈間用 Dubins 路徑過渡。

        策略:
            由外圈向內圈掃描 (外圈半徑大、曲率小、更安全)。
            每圈結束時，飛行器需從當前圓切線方向過渡到
            內圈的切線方向 → 用 Dubins 連接。

        曲率約束:
            圓的曲率 κ = 1/r，需滿足 r >= R_min。
            因此最內圈半徑不能小於 R_min。

        圈間過渡 (參考 NCIST 計畫 State Lattice 思路):
            相鄰圈半徑差 = spacing，飛行器需從 r_i 圓弧
            過渡到 r_{i-1} 圓弧。若 spacing < 2R_min，
            直接 Dubins 可能失敗 → 允許飛行器在外側
            做一個延伸弧後切入內圈 (類似 Bulb Turn)。

        Args:
            params: 同心圓參數

        Returns:
            AdvancedScanResult
        """
        result = AdvancedScanResult(pattern=ScanPattern.CONCENTRIC)

        R = self._r_min
        cx, cy = params.center
        spacing = max(params.spacing, 1.0)
        alt = self._altitude
        arc_step = max(params.arc_step_deg, 1.0)

        # 半徑序列: 由外向內
        r_inner = max(R * 1.1, spacing)
        r_outer = params.max_radius

        if r_outer <= r_inner:
            result.warnings.append("外圍半徑不足以生成同心圓")
            result.is_feasible = False
            return result

        radii: List[float] = []
        r = r_outer
        while r >= r_inner:
            radii.append(r)
            r -= spacing
        if not radii:
            result.warnings.append("無法生成任何圈")
            return result

        result.num_rings = len(radii)
        min_curvature_r = min(radii)
        result.min_curvature_radius = min_curvature_r

        if min_curvature_r < R:
            result.is_feasible = False
            result.warnings.append(
                f"最內圈半徑 {min_curvature_r:.1f}m < R_min {R:.1f}m"
            )

        total_scan_len = 0.0
        total_trans_len = 0.0
        arc_step_rad = deg_to_rad(arc_step)

        for ring_idx, radius in enumerate(radii):
            # --- 生成一圈圓弧航點 ---
            # 交替方向: 偶數圈逆時針，奇數圈順時針
            direction = 1.0 if ring_idx % 2 == 0 else -1.0
            n_steps = max(4, int(math.ceil(2.0 * math.pi / arc_step_rad)))
            d_angle = direction * 2.0 * math.pi / n_steps

            ring_start_angle = 0.0
            if ring_idx > 0 and result.poses:
                # 從上一圈結束位置的角度開始
                last_pose = result.poses[-1]
                ring_start_angle = math.atan2(
                    last_pose.y - cy, last_pose.x - cx
                )

            ring_poses: List[Pose3D] = []
            ring_length = 0.0
            prev_x, prev_y = None, None

            for step in range(n_steps + 1):  # +1 閉合
                angle = ring_start_angle + step * d_angle
                x = cx + radius * math.cos(angle)
                y = cy + radius * math.sin(angle)

                # 切線方向
                if direction > 0:
                    hdg = rad_to_deg(angle + math.pi / 2)
                else:
                    hdg = rad_to_deg(angle - math.pi / 2)

                ring_poses.append(
                    Pose3D(x, y, alt, normalize_angle(hdg, 0.0))
                )

                if prev_x is not None:
                    ring_length += math.hypot(x - prev_x, y - prev_y)
                prev_x, prev_y = x, y

            total_scan_len += ring_length

            # --- 圈間 Dubins 過渡 ---
            if ring_idx > 0 and result.poses and ring_poses:
                exit_pose = result.poses[-1]
                entry_pose = ring_poses[0]

                dpath = self._dubins_gen.calculate_path(exit_pose, entry_pose)
                if dpath.is_feasible:
                    trans_wps = self._dubins_gen.generate_waypoints(dpath, step_size=10.0)
                    if len(trans_wps) > 2:
                        result.poses.extend(trans_wps[1:-1])
                    total_trans_len += dpath.total_length
                    result.transition_paths.append(dpath)
                else:
                    result.warnings.append(
                        f"圈 {ring_idx-1}→{ring_idx} Dubins 過渡不可行: {dpath.warning}"
                    )

            result.poses.extend(ring_poses)

        result.total_scan_length = total_scan_len
        result.total_transition_length = total_trans_len
        result.total_length = total_scan_len + total_trans_len

        return result

    # ==========================================
    # 公開: 自動模式選擇
    # ==========================================
    def auto_select_pattern(
        self,
        polygon: Sequence[Tuple[float, float]],
        spacing: float,
    ) -> ScanPattern:
        """
        根據多邊形形狀自動推薦掃描模式。

        判斷邏輯:
            - 計算多邊形的等效圓度 (Circularity = 4π·A / P²)
            - 圓度 > 0.7 → 同心圓或螺旋
            - 圓度 ≤ 0.7 → Zigzag (長條形/不規則形)
            - 若間距 < 2R_min 且圓度高 → 螺旋 (避免大量 Bulb Turn)

        Args:
            polygon: 多邊形頂點
            spacing: 掃描線間距

        Returns:
            推薦的 ScanPattern
        """
        if len(polygon) < 3:
            return ScanPattern.ZIGZAG

        # Shapely-free 面積與周長計算
        n = len(polygon)
        area = abs(sum(
            polygon[i][0] * polygon[(i+1) % n][1] -
            polygon[(i+1) % n][0] * polygon[i][1]
            for i in range(n)
        )) / 2.0

        perimeter = sum(
            math.hypot(
                polygon[(i+1) % n][0] - polygon[i][0],
                polygon[(i+1) % n][1] - polygon[i][1]
            )
            for i in range(n)
        )

        if perimeter < 1e-6:
            return ScanPattern.ZIGZAG

        circularity = 4.0 * math.pi * area / (perimeter ** 2)

        if circularity > 0.7:
            if spacing < 2.0 * self._r_min:
                return ScanPattern.SPIRAL_INWARD
            return ScanPattern.CONCENTRIC
        return ScanPattern.ZIGZAG

    # ==========================================
    # 公開: 多邊形 → 螺旋/同心圓參數自動提取
    # ==========================================
    def params_from_polygon(
        self,
        polygon: Sequence[Tuple[float, float]],
        spacing: float,
    ) -> Tuple[SpiralParams, ConcentricParams]:
        """
        從多邊形自動提取螺旋/同心圓的中心與半徑。

        使用最小外接圓 (Minimum Bounding Circle) 的中心與半徑。

        Args:
            polygon: 多邊形頂點
            spacing: 掃描間距

        Returns:
            (SpiralParams, ConcentricParams) 同時回傳兩種參數
        """
        # 質心
        n = len(polygon)
        cx = sum(p[0] for p in polygon) / n
        cy = sum(p[1] for p in polygon) / n

        # 最大距離作為外圍半徑
        max_r = max(math.hypot(p[0] - cx, p[1] - cy) for p in polygon)

        sp = SpiralParams(
            center=(cx, cy),
            max_radius=max_r,
            spacing=spacing,
        )
        cp = ConcentricParams(
            center=(cx, cy),
            max_radius=max_r,
            spacing=spacing,
        )
        return sp, cp

    # ==========================================
    # 資訊
    # ==========================================
    def __repr__(self) -> str:
        return (
            f"AdvancedScanGenerator(R_min={self._r_min:.1f}m, "
            f"alt={self._altitude:.0f}m)"
        )


# ==========================================
# 測試
# ==========================================
if __name__ == "__main__":
    from core.base.fixed_wing_constraints import FixedWingConstraints

    fw = FixedWingConstraints(cruise_airspeed_mps=18.0, max_bank_angle_deg=45.0)
    gen = AdvancedScanGenerator(fw, altitude_m=100.0)
    print(gen)
    print(f"R_min = {gen.r_min:.1f} m\n")

    # --- 螺旋掃描 ---
    print("=== 螺旋掃描 (由外向內) ===")
    sp = SpiralParams(center=(0, 0), max_radius=400, spacing=60)
    spiral_result = gen.generate_spiral(sp, inward=True)
    print(spiral_result.summary())
    if spiral_result.poses:
        p0 = spiral_result.poses[0]
        pn = spiral_result.poses[-1]
        print(f"起點: ({p0.x:.0f}, {p0.y:.0f}) h={p0.heading_deg:.0f}°")
        print(f"終點: ({pn.x:.0f}, {pn.y:.0f}) h={pn.heading_deg:.0f}°")

    # --- 同心圓掃描 ---
    print("\n=== 同心圓掃描 ===")
    cp = ConcentricParams(center=(0, 0), max_radius=400, spacing=60)
    cc_result = gen.generate_concentric(cp)
    print(cc_result.summary())

    # --- 窄間距螺旋 (測試 R_min 截斷) ---
    print("\n=== 窄間距螺旋 (spacing=30m < 2R) ===")
    sp2 = SpiralParams(center=(0, 0), max_radius=300, spacing=30)
    narrow_result = gen.generate_spiral(sp2, inward=True)
    print(narrow_result.summary())
    if narrow_result.warnings:
        for w in narrow_result.warnings:
            print(f"  ⚠ {w}")

    # --- 自動模式選擇 ---
    print("\n=== 自動模式選擇 ===")
    # 近似圓形多邊形 (16邊形)
    circle_poly = [
        (200 * math.cos(i * 2 * math.pi / 16),
         200 * math.sin(i * 2 * math.pi / 16))
        for i in range(16)
    ]
    rec = gen.auto_select_pattern(circle_poly, spacing=60)
    print(f"圓形區域 (spacing=60): 推薦 {rec.name}")

    rec2 = gen.auto_select_pattern(circle_poly, spacing=30)
    print(f"圓形區域 (spacing=30): 推薦 {rec2.name}")

    # 長條形
    rect_poly = [(0, 0), (1000, 0), (1000, 100), (0, 100)]
    rec3 = gen.auto_select_pattern(rect_poly, spacing=60)
    print(f"長條形區域: 推薦 {rec3.name}")