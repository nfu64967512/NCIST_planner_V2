"""
3D Dubins 軌跡生成器
====================
將 2D Dubins Curves 擴展至 3D 空間，為固定翼無人機生成
符合最小轉彎半徑約束的最短平滑路徑。

核心思路 (參考 ntnu-arl/DubinsAirplane):
    1. 水平面: 經典 Dubins 路徑 — CSC (RSR/LSL/RSL/LSR) + CCC (RLR/LRL)
    2. 垂直面: 沿水平弧長線性插值高度，恆定爬升角 γ
    3. 退化處理: 距離不足時插入 loiter 對準圈

航向不連續問題的解決:
    固定翼不能原地轉向，傳統折線路徑在航點處航向不連續。
    Dubins 路徑由圓弧 + 直線組成，航向沿路徑處處連續且
    曲率 ≤ 1/R_min，保證飛行器可追蹤。

作者: NCIST_planner_V1
版本: 1.1.0
"""

from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from enum import Enum, auto

from core.base.fixed_wing_constraints import FixedWingConstraints
from utils.math_utils import deg_to_rad, rad_to_deg, normalize_angle


# ==========================================
# 列舉與資料結構
# ==========================================
class SegmentType(Enum):
    """Dubins 路徑段類型"""
    LEFT_TURN = auto()
    RIGHT_TURN = auto()
    STRAIGHT = auto()


class DubinsPathType(Enum):
    """Dubins 路徑組合類型"""
    RSR = "RSR"
    LSL = "LSL"
    RSL = "RSL"
    LSR = "LSR"
    RLR = "RLR"
    LRL = "LRL"


@dataclass
class Pose3D:
    """
    三維姿態，NED 公尺座標系。

    Attributes:
        x: 北向位移 [m]
        y: 東向位移 [m]
        z: 高度 (向下為正，但本模組以向上為正使用) [m]
        heading_deg: 航向角 [度]，真北=0，順時針為正
    """
    x: float
    y: float
    z: float
    heading_deg: float


@dataclass
class DubinsSegment:
    """
    單一 Dubins 路徑段。

    Attributes:
        seg_type: 段類型 (左轉/右轉/直線)
        center_x: 圓弧圓心 x (直線段為 NaN)
        center_y: 圓弧圓心 y (直線段為 NaN)
        radius: 轉彎半徑 (直線段為 0)
        angle_start_rad: 圓弧起始角 [rad] (從圓心看，x 軸正向=0)
        angle_sweep_rad: 圓弧掃掠角 [rad] (正=逆時針，負=順時針)
        length: 段弧長/線長 [m]
    """
    seg_type: SegmentType
    center_x: float
    center_y: float
    radius: float
    angle_start_rad: float
    angle_sweep_rad: float
    length: float


@dataclass
class DubinsPath3D:
    """
    完整 3D Dubins 路徑結果。

    Attributes:
        segments: 路徑段列表 (通常 3 段)
        total_length: 水平面總路徑長 [m]
        climb_angle_rad: 爬升角 γ [rad]
        path_type: 路徑組合類型
        is_feasible: 是否可行
        warning: 警告訊息
        start: 起始姿態
        end: 終止姿態
    """
    segments: List[DubinsSegment] = field(default_factory=list)
    total_length: float = 0.0
    climb_angle_rad: float = 0.0
    path_type: Optional[DubinsPathType] = None
    is_feasible: bool = True
    warning: str = ""
    start: Optional[Pose3D] = None
    end: Optional[Pose3D] = None


# ==========================================
# 軌跡生成器
# ==========================================
class DubinsTrajectoryGenerator:
    """
    3D Dubins 軌跡生成器。

    透過組合 (Composition) 持有 FixedWingConstraints，
    在水平面計算 Dubins 最短路徑，垂直面線性插值高度。

    Args:
        constraints: 固定翼物理約束實例
        max_climb_angle_deg: 最大爬升角 [度]
        max_descent_angle_deg: 最大下降角 [度]
    """

    __slots__ = (
        '_constraints', '_max_climb_rad', '_max_descent_rad', '_radius'
    )

    def __init__(
        self,
        constraints: FixedWingConstraints,
        max_climb_angle_deg: float = 15.0,
        max_descent_angle_deg: float = 20.0,
    ) -> None:
        self._constraints = constraints
        self._max_climb_rad = deg_to_rad(max_climb_angle_deg)
        self._max_descent_rad = deg_to_rad(max_descent_angle_deg)
        self._radius = constraints.get_min_turn_radius()

    # --- 唯讀屬性 ---
    @property
    def constraints(self) -> FixedWingConstraints:
        return self._constraints

    @property
    def radius(self) -> float:
        """Dubins 轉彎半徑 R_min [m]"""
        return self._radius

    @property
    def max_climb_rad(self) -> float:
        return self._max_climb_rad

    @property
    def max_descent_rad(self) -> float:
        return self._max_descent_rad

    # ==========================================
    # 公開方法: 計算路徑
    # ==========================================
    def calculate_path(self, start: Pose3D, end: Pose3D) -> DubinsPath3D:
        """
        計算兩個 3D 姿態間的 Dubins 最短路徑。

        步驟:
            1. 同點同向 → 空路徑
            2. 距離過近 → 插入 loiter 對準圈
            3. 計算 6 種候選 (CSC×4 + CCC×2) 取最短
            4. 垂直面爬升角檢查

        Args:
            start: 起始姿態
            end: 終止姿態

        Returns:
            DubinsPath3D 結果
        """
        dx = end.x - start.x
        dy = end.y - start.y
        dist_2d = math.hypot(dx, dy)
        dz = end.z - start.z

        # --- 退化: 同點同向 ---
        heading_diff = abs(normalize_angle(end.heading_deg - start.heading_deg))
        if dist_2d < 1e-3 and heading_diff < 1.0 and abs(dz) < 1e-3:
            return DubinsPath3D(
                start=start, end=end, warning="起終點重合，無需路徑"
            )

        # --- 退化: 距離過近，插入 loiter 對準圈 ---
        loiter_segments: List[DubinsSegment] = []
        effective_start = start
        if dist_2d < 2.0 * self._radius:
            loiter_segments, effective_start = self._insert_loiter(start)
            dx = end.x - effective_start.x
            dy = end.y - effective_start.y
            dist_2d = math.hypot(dx, dy)

        # --- 水平面: 4 種 CSC + 2 種 CCC ---
        best_path: Optional[DubinsPath3D] = None
        best_len = float('inf')

        h_s = deg_to_rad(effective_start.heading_deg)
        h_e = deg_to_rad(end.heading_deg)

        # CSC 候選
        for ptype in (DubinsPathType.RSR, DubinsPathType.LSL,
                      DubinsPathType.RSL, DubinsPathType.LSR):
            segs = self._compute_csc(
                effective_start.x, effective_start.y, h_s,
                end.x, end.y, h_e,
                ptype,
            )
            if segs is None:
                continue
            total_l = sum(s.length for s in segs)
            if total_l < best_len:
                best_len = total_l
                best_path = DubinsPath3D(
                    segments=loiter_segments + segs,
                    total_length=sum(s.length for s in loiter_segments) + total_l,
                    path_type=ptype,
                    start=start,
                    end=end,
                )

        # CCC 候選 (RLR, LRL)
        for ptype in (DubinsPathType.RLR, DubinsPathType.LRL):
            segs = self._compute_ccc(
                effective_start.x, effective_start.y, h_s,
                end.x, end.y, h_e,
                ptype,
            )
            if segs is None:
                continue
            total_l = sum(s.length for s in segs)
            if total_l < best_len:
                best_len = total_l
                best_path = DubinsPath3D(
                    segments=loiter_segments + segs,
                    total_length=sum(s.length for s in loiter_segments) + total_l,
                    path_type=ptype,
                    start=start,
                    end=end,
                )

        if best_path is None:
            return DubinsPath3D(
                start=start, end=end,
                is_feasible=False,
                warning="無法計算任何 CSC 路徑",
            )

        # --- 垂直面: 爬升角 ---
        if best_path.total_length > 1e-3:
            gamma = math.atan2(dz, best_path.total_length)
        else:
            gamma = 0.0

        best_path.climb_angle_rad = gamma

        if gamma > self._max_climb_rad:
            best_path.is_feasible = False
            best_path.warning = (
                f"爬升角 {rad_to_deg(gamma):.1f}° 超出限制 "
                f"{rad_to_deg(self._max_climb_rad):.1f}°"
            )
        elif gamma < -self._max_descent_rad:
            best_path.is_feasible = False
            best_path.warning = (
                f"下降角 {rad_to_deg(abs(gamma)):.1f}° 超出限制 "
                f"{rad_to_deg(self._max_descent_rad):.1f}°"
            )

        return best_path

    # ==========================================
    # 公開方法: 離散化
    # ==========================================
    def generate_waypoints(
        self,
        path: DubinsPath3D,
        step_size: float = 5.0,
    ) -> List[Pose3D]:
        """
        將 Dubins 路徑離散化為 3D 航點序列。

        沿水平弧長 s 以 step_size 採樣，高度線性插值:
            z(s) = z_start + s · tan(γ)

        Args:
            path: calculate_path() 的輸出
            step_size: 採樣間距 [m]

        Returns:
            航點列表，每點含 (x, y, z, heading_deg)
        """
        if not path.segments or path.start is None:
            return []

        waypoints: List[Pose3D] = []
        z_start = path.start.z
        tan_gamma = math.tan(path.climb_angle_rad) if path.total_length > 1e-3 else 0.0
        cumulative_s = 0.0

        for seg in path.segments:
            if seg.length < 1e-6:
                continue

            n_steps = max(1, int(math.ceil(seg.length / step_size)))

            for i in range(n_steps):
                frac = i / n_steps
                s_local = frac * seg.length

                if seg.seg_type == SegmentType.STRAIGHT:
                    # 直線段: 沿起始方向等距前進
                    heading_rad = seg.angle_start_rad
                    x = seg.center_x + s_local * math.cos(heading_rad)
                    y = seg.center_y + s_local * math.sin(heading_rad)
                    hdg_deg = rad_to_deg(heading_rad)
                else:
                    # 圓弧段: 角度步進
                    swept = frac * seg.angle_sweep_rad
                    angle = seg.angle_start_rad + swept
                    x = seg.center_x + seg.radius * math.cos(angle)
                    y = seg.center_y + seg.radius * math.sin(angle)
                    # 航向 = 切線方向
                    if seg.seg_type == SegmentType.LEFT_TURN:
                        hdg_deg = rad_to_deg(angle + math.pi / 2)
                    else:
                        hdg_deg = rad_to_deg(angle - math.pi / 2)

                z = z_start + (cumulative_s + s_local) * tan_gamma
                waypoints.append(Pose3D(x, y, z, normalize_angle(hdg_deg, 0.0)))

            cumulative_s += seg.length

        # 附加終點
        if path.end is not None:
            waypoints.append(Pose3D(
                path.end.x, path.end.y, path.end.z,
                normalize_angle(path.end.heading_deg, 0.0)
            ))

        return waypoints

    # ==========================================
    # 風場修正預留
    # ==========================================
    def set_wind_correction(self, speed_mps: float = 0.0, direction_deg: float = 0.0) -> None:
        """
        設定風場修正（預留介面）。
        聯動 FixedWingConstraints.set_wind 後重新計算 R_min。
        """
        self._constraints.set_wind(speed_mps, direction_deg)
        self._radius = self._constraints.get_min_turn_radius()

    # ==========================================
    # 內部: CSC 路徑計算
    # ==========================================
    def _compute_csc(
        self,
        x1: float, y1: float, h1: float,
        x2: float, y2: float, h2: float,
        ptype: DubinsPathType,
    ) -> Optional[List[DubinsSegment]]:
        """
        計算指定類型的 CSC Dubins 路徑。

        幾何推導:
        ─────────
        1. 圓心計算:
           右轉圓心 = 起點 + R * (sin(h), -cos(h))  (航向右側偏移)
           左轉圓心 = 起點 + R * (-sin(h), cos(h))  (航向左側偏移)
           此處 h 為航向角(數學座標系，x 軸正向=0，逆時針為正)

        2. 外切線 (RSR, LSL):
           兩圓同側切線，切線方向角 = atan2(cy2-cy1, cx2-cx1) ± acos(0)
           (同半徑時切線平行於圓心連線)

        3. 內切線 (RSL, LSR):
           兩圓異側切線，切線方向角涉及圓心距與半徑的三角關係:
           β = atan2(cy2-cy1, cx2-cx1)
           α = acos(2R / D)  其中 D = 圓心距
           切線角 = β ± α

        Args:
            x1,y1,h1: 起點座標與航向 [rad]
            x2,y2,h2: 終點座標與航向 [rad]
            ptype: CSC 組合類型

        Returns:
            3 個 DubinsSegment 的列表，或 None (幾何不可行)
        """
        R = self._radius

        # --- 起點圓心 ---
        if ptype in (DubinsPathType.RSR, DubinsPathType.RSL):
            # 起點右轉圓: 航向右側偏移 R
            cx1 = x1 + R * math.sin(h1)
            cy1 = y1 - R * math.cos(h1)
            start_turn = SegmentType.RIGHT_TURN
        else:
            # 起點左轉圓: 航向左側偏移 R
            cx1 = x1 - R * math.sin(h1)
            cy1 = y1 + R * math.cos(h1)
            start_turn = SegmentType.LEFT_TURN

        # --- 終點圓心 ---
        if ptype in (DubinsPathType.RSR, DubinsPathType.LSR):
            # 終點右轉圓
            cx2 = x2 + R * math.sin(h2)
            cy2 = y2 - R * math.cos(h2)
            end_turn = SegmentType.RIGHT_TURN
        else:
            # 終點左轉圓
            cx2 = x2 - R * math.sin(h2)
            cy2 = y2 + R * math.cos(h2)
            end_turn = SegmentType.LEFT_TURN

        # --- 圓心距 ---
        dx = cx2 - cx1
        dy = cy2 - cy1
        D = math.hypot(dx, dy)

        if D < 1e-6:
            # 圓心重合: 只有同類型(RSR/LSL)才可能，直接轉向
            return self._coincident_centers(
                cx1, cy1, R, h1, h2, start_turn, end_turn, ptype
            )

        beta = math.atan2(dy, dx)

        # --- 切線角計算 ---
        if ptype == DubinsPathType.RSR:
            # 外切線 (同為右轉，同半徑)
            tangent_angle = beta
            # 起點圓弧: 從起點在圓上的角度 → 切點角度 (順時針)
            angle_s = math.atan2(y1 - cy1, x1 - cx1)
            angle_s_end = tangent_angle - math.pi / 2
            sweep_s = self._wrap_cw(angle_s, angle_s_end)
            # 終點圓弧
            angle_e = tangent_angle - math.pi / 2
            angle_e_end = math.atan2(y2 - cy2, x2 - cx2)
            sweep_e = self._wrap_cw(angle_e, angle_e_end)

        elif ptype == DubinsPathType.LSL:
            tangent_angle = beta
            angle_s = math.atan2(y1 - cy1, x1 - cx1)
            angle_s_end = tangent_angle + math.pi / 2
            sweep_s = self._wrap_ccw(angle_s, angle_s_end)
            angle_e = tangent_angle + math.pi / 2
            angle_e_end = math.atan2(y2 - cy2, x2 - cx2)
            sweep_e = self._wrap_ccw(angle_e, angle_e_end)

        elif ptype == DubinsPathType.RSL:
            # 內切線: 右轉 → 直線 → 左轉
            ratio = 2.0 * R / D
            if ratio > 1.0:
                return None  # 幾何不可行: 圓心距太近
            alpha = math.acos(ratio)
            tangent_angle = beta + alpha
            angle_s = math.atan2(y1 - cy1, x1 - cx1)
            angle_s_end = tangent_angle - math.pi / 2
            sweep_s = self._wrap_cw(angle_s, angle_s_end)
            angle_e = tangent_angle + math.pi / 2 + math.pi
            angle_e_end = math.atan2(y2 - cy2, x2 - cx2)
            sweep_e = self._wrap_ccw(angle_e, angle_e_end)

        elif ptype == DubinsPathType.LSR:
            ratio = 2.0 * R / D
            if ratio > 1.0:
                return None
            alpha = math.acos(ratio)
            tangent_angle = beta - alpha
            angle_s = math.atan2(y1 - cy1, x1 - cx1)
            angle_s_end = tangent_angle + math.pi / 2
            sweep_s = self._wrap_ccw(angle_s, angle_s_end)
            angle_e = tangent_angle - math.pi / 2 + math.pi
            angle_e_end = math.atan2(y2 - cy2, x2 - cx2)
            sweep_e = self._wrap_cw(angle_e, angle_e_end)
        else:
            return None

        # --- 切點座標 & 直線段 ---
        tp1_x = cx1 + R * math.cos(angle_s + sweep_s)
        tp1_y = cy1 + R * math.sin(angle_s + sweep_s)
        tp2_x = cx2 + R * math.cos(angle_e)
        tp2_y = cy2 + R * math.sin(angle_e)

        straight_len = math.hypot(tp2_x - tp1_x, tp2_y - tp1_y)
        straight_heading = math.atan2(tp2_y - tp1_y, tp2_x - tp1_x)

        # --- 組裝三段 ---
        seg1 = DubinsSegment(
            seg_type=start_turn,
            center_x=cx1, center_y=cy1,
            radius=R,
            angle_start_rad=angle_s,
            angle_sweep_rad=sweep_s,
            length=abs(sweep_s) * R,
        )
        seg2 = DubinsSegment(
            seg_type=SegmentType.STRAIGHT,
            center_x=tp1_x, center_y=tp1_y,  # 直線段起點
            radius=0.0,
            angle_start_rad=straight_heading,
            angle_sweep_rad=0.0,
            length=straight_len,
        )
        seg3 = DubinsSegment(
            seg_type=end_turn,
            center_x=cx2, center_y=cy2,
            radius=R,
            angle_start_rad=angle_e,
            angle_sweep_rad=sweep_e,
            length=abs(sweep_e) * R,
        )

        return [seg1, seg2, seg3]

    # ==========================================
    # 內部: CCC 路徑計算 (RLR / LRL)
    # ==========================================
    def _compute_ccc(
        self,
        x1: float, y1: float, h1: float,
        x2: float, y2: float, h2: float,
        ptype: DubinsPathType,
    ) -> Optional[List[DubinsSegment]]:
        """
        計算 CCC (三段圓弧) Dubins 路徑。

        幾何推導 (以 RLR 為例):
        ─────────────────────────
        三個圓: C1(右轉, R), C_m(左轉, R), C2(右轉, R)
        相鄰圓外切 → 圓心距 = 2R

        1. C1 圓心 = 起點右側偏移 R
           C2 圓心 = 終點右側偏移 R
        2. D = |C1C2|，需滿足 D ≤ 4R (否則三圓無法首尾相接)
        3. 中間圓 C_m 圓心由餘弦定理求解:
           三角形 C1-C_m-C2，三邊為 2R, 2R, D
           cos(α) = D / (4R)
           其中 α = C1→C_m 相對 C1→C2 的偏轉角

        LRL 同理，方向全部反轉。

        Args:
            x1,y1,h1: 起點座標與航向 [rad]
            x2,y2,h2: 終點座標與航向 [rad]
            ptype: RLR 或 LRL

        Returns:
            3 個 DubinsSegment 的列表，或 None (幾何不可行)
        """
        R = self._radius

        if ptype == DubinsPathType.RLR:
            # C1: 右轉圓心, C2: 右轉圓心, C_m: 左轉圓心
            cx1 = x1 + R * math.sin(h1)
            cy1 = y1 - R * math.cos(h1)
            cx2 = x2 + R * math.sin(h2)
            cy2 = y2 - R * math.cos(h2)
            outer_turn = SegmentType.RIGHT_TURN
            mid_turn = SegmentType.LEFT_TURN
            wrap_outer = self._wrap_cw
            wrap_mid = self._wrap_ccw
        else:  # LRL
            cx1 = x1 - R * math.sin(h1)
            cy1 = y1 + R * math.cos(h1)
            cx2 = x2 - R * math.sin(h2)
            cy2 = y2 + R * math.cos(h2)
            outer_turn = SegmentType.LEFT_TURN
            mid_turn = SegmentType.RIGHT_TURN
            wrap_outer = self._wrap_ccw
            wrap_mid = self._wrap_cw

        # --- 圓心距 ---
        dx = cx2 - cx1
        dy = cy2 - cy1
        D = math.hypot(dx, dy)

        # 可行性: D ≤ 4R (兩個外圓各與中間圓外切，圓心距 = 2R)
        if D > 4.0 * R or D < 1e-6:
            return None

        # --- 中間圓圓心 (餘弦定理) ---
        # 三角形 C1-C_m-C2: C1C_m = C2C_m = 2R, C1C2 = D
        # cos(α) = D / (4R)，α 為 C1→C_m 相對 C1→C2 的偏轉角
        cos_alpha = D / (4.0 * R)
        cos_alpha = max(-1.0, min(1.0, cos_alpha))  # 數值保護
        alpha = math.acos(cos_alpha)

        beta = math.atan2(dy, dx)

        # RLR: 中間圓在 C1C2 連線的左側 (逆時針偏轉)
        # LRL: 中間圓在 C1C2 連線的右側 (順時針偏轉)
        if ptype == DubinsPathType.RLR:
            cm_angle = beta + alpha
        else:
            cm_angle = beta - alpha

        cm_x = cx1 + 2.0 * R * math.cos(cm_angle)
        cm_y = cy1 + 2.0 * R * math.sin(cm_angle)

        # --- 切點角度 ---
        # 切點1: C1 圓與 C_m 圓的切點 (在 C1 圓上的角度)
        tp1_angle_on_c1 = math.atan2(cm_y - cy1, cm_x - cx1)
        # 切點2: C_m 圓與 C2 圓的切點 (在 C_m 圓上的角度)
        tp2_angle_on_cm = math.atan2(cy2 - cm_y, cx2 - cm_x)

        # --- 段1: 起點 → 切點1 (外圓弧) ---
        angle_s = math.atan2(y1 - cy1, x1 - cx1)
        sweep_1 = wrap_outer(angle_s, tp1_angle_on_c1)

        # --- 段2: 切點1 → 切點2 (中間圓弧) ---
        # 在中間圓上: 切點1 的角度 = 從 C_m 看回 C1 的方向
        tp1_angle_on_cm = math.atan2(cy1 - cm_y, cx1 - cm_x)
        sweep_2 = wrap_mid(tp1_angle_on_cm, tp2_angle_on_cm)

        # --- 段3: 切點2 → 終點 (外圓弧) ---
        # 在 C2 圓上: 切點2 的角度 = 從 C2 看回 C_m 的方向
        tp2_angle_on_c2 = math.atan2(cm_y - cy2, cm_x - cx2)
        angle_e_end = math.atan2(y2 - cy2, x2 - cx2)
        sweep_3 = wrap_outer(tp2_angle_on_c2, angle_e_end)

        # --- 組裝 ---
        seg1 = DubinsSegment(
            seg_type=outer_turn,
            center_x=cx1, center_y=cy1, radius=R,
            angle_start_rad=angle_s, angle_sweep_rad=sweep_1,
            length=abs(sweep_1) * R,
        )
        seg2 = DubinsSegment(
            seg_type=mid_turn,
            center_x=cm_x, center_y=cm_y, radius=R,
            angle_start_rad=tp1_angle_on_cm, angle_sweep_rad=sweep_2,
            length=abs(sweep_2) * R,
        )
        seg3 = DubinsSegment(
            seg_type=outer_turn,
            center_x=cx2, center_y=cy2, radius=R,
            angle_start_rad=tp2_angle_on_c2, angle_sweep_rad=sweep_3,
            length=abs(sweep_3) * R,
        )

        return [seg1, seg2, seg3]

    # ==========================================
    # 內部: Loiter 對準圈
    # ==========================================
    def _insert_loiter(self, start: Pose3D) -> Tuple[List[DubinsSegment], Pose3D]:
        """
        距離不足時插入一圈右轉 loiter，使飛行器繞行後
        累積足夠的水平位移再進入 Dubins 路徑。

        Returns:
            (loiter_segments, new_effective_start)
        """
        R = self._radius
        h = deg_to_rad(start.heading_deg)

        # 右轉圓心
        cx = start.x + R * math.sin(h)
        cy = start.y - R * math.cos(h)

        angle_start = math.atan2(start.y - cy, start.x - cx)
        sweep = -2.0 * math.pi  # 順時針一整圈

        seg = DubinsSegment(
            seg_type=SegmentType.RIGHT_TURN,
            center_x=cx, center_y=cy,
            radius=R,
            angle_start_rad=angle_start,
            angle_sweep_rad=sweep,
            length=2.0 * math.pi * R,
        )

        # Loiter 結束後回到原位、原航向
        new_start = Pose3D(start.x, start.y, start.z, start.heading_deg)
        return [seg], new_start

    # ==========================================
    # 內部: 圓心重合特例
    # ==========================================
    def _coincident_centers(
        self,
        cx: float, cy: float, R: float,
        h1: float, h2: float,
        start_turn: SegmentType, end_turn: SegmentType,
        ptype: DubinsPathType,
    ) -> Optional[List[DubinsSegment]]:
        """圓心重合時的退化路徑（純轉向）"""
        angle_s = h1 - math.pi / 2 if start_turn == SegmentType.RIGHT_TURN else h1 + math.pi / 2
        angle_e = h2 - math.pi / 2 if end_turn == SegmentType.RIGHT_TURN else h2 + math.pi / 2

        if start_turn == SegmentType.RIGHT_TURN:
            sweep = self._wrap_cw(angle_s, angle_e)
        else:
            sweep = self._wrap_ccw(angle_s, angle_e)

        seg = DubinsSegment(
            seg_type=start_turn,
            center_x=cx, center_y=cy,
            radius=R,
            angle_start_rad=angle_s,
            angle_sweep_rad=sweep,
            length=abs(sweep) * R,
        )
        # 直線段長度為 0
        seg_s = DubinsSegment(
            seg_type=SegmentType.STRAIGHT,
            center_x=cx + R * math.cos(angle_s + sweep),
            center_y=cy + R * math.sin(angle_s + sweep),
            radius=0.0, angle_start_rad=0.0, angle_sweep_rad=0.0, length=0.0,
        )
        return [seg, seg_s, seg]

    # ==========================================
    # 工具: 弧度正規化
    # ==========================================
    @staticmethod
    def _wrap_cw(from_rad: float, to_rad: float) -> float:
        """計算順時針 (負方向) 掃掠角，結果 ∈ (-2π, 0]"""
        diff = to_rad - from_rad
        diff = diff % (2 * math.pi)
        if diff > 0:
            diff -= 2 * math.pi
        if abs(diff) < 1e-10:
            diff = 0.0
        return diff

    @staticmethod
    def _wrap_ccw(from_rad: float, to_rad: float) -> float:
        """計算逆時針 (正方向) 掃掠角，結果 ∈ [0, 2π)"""
        diff = to_rad - from_rad
        diff = diff % (2 * math.pi)
        if diff < 0:
            diff += 2 * math.pi
        if abs(diff) < 1e-10:
            diff = 0.0
        return diff

    # ==========================================
    # 資訊
    # ==========================================
    def __repr__(self) -> str:
        return (
            f"DubinsTrajectoryGenerator(R={self._radius:.1f}m, "
            f"climb={rad_to_deg(self._max_climb_rad):.0f}°, "
            f"descent={rad_to_deg(self._max_descent_rad):.0f}°)"
        )


# ==========================================
# 測試
# ==========================================
if __name__ == "__main__":
    import sys
    import os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

    from core.base.fixed_wing_constraints import FixedWingConstraints
    from utils.math_utils import deg_to_rad, rad_to_deg, normalize_angle

    fw = FixedWingConstraints(cruise_airspeed_mps=18.0, max_bank_angle_deg=45.0)
    gen = DubinsTrajectoryGenerator(fw, max_climb_angle_deg=15.0)
    print(gen)
    print(fw.info())
    print()

    start = Pose3D(0, 0, 100, 0)
    end = Pose3D(500, 300, 150, 90)
    path = gen.calculate_path(start, end)

    print(f"路徑類型: {path.path_type}")
    print(f"水平總長: {path.total_length:.1f} m")
    print(f"爬升角:   {rad_to_deg(path.climb_angle_rad):.2f}°")
    print(f"可行性:   {path.is_feasible}")
    if path.warning:
        print(f"警告:     {path.warning}")
    print()

    wps = gen.generate_waypoints(path, step_size=5.0)
    print(f"生成 {len(wps)} 個航點")
    if wps:
        print(f"起點: ({wps[0].x:.1f}, {wps[0].y:.1f}, {wps[0].z:.1f}) h={wps[0].heading_deg:.1f}°")
        print(f"終點: ({wps[-1].x:.1f}, {wps[-1].y:.1f}, {wps[-1].z:.1f}) h={wps[-1].heading_deg:.1f}°")

    # --- 測試退化: 距離過近 ---
    print("\n--- 近距離測試 ---")
    near_end = Pose3D(10, 5, 100, 180)
    near_path = gen.calculate_path(start, near_end)
    print(f"類型: {near_path.path_type}, 長度: {near_path.total_length:.1f} m")
    print(f"含 loiter: {len(near_path.segments)} 段")

    # --- 測試 CCC: 近距離大角度轉向 ---
    print("\n--- CCC 路徑測試 (近距離急轉) ---")
    ccc_end = Pose3D(30, 0, 100, 270)
    ccc_path = gen.calculate_path(start, ccc_end)
    print(f"類型: {ccc_path.path_type}, 長度: {ccc_path.total_length:.1f} m")
    print(f"段數: {len(ccc_path.segments)}, 可行: {ccc_path.is_feasible}")
