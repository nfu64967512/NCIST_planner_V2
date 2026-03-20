"""
固定翼覆蓋路徑規劃器
====================
為固定翼無人機生成符合最小轉彎半徑約束的區域覆蓋路徑。

為什麼固定翼不能用簡單 U-turn:
    多旋翼可以原地懸停轉向，U-turn 半徑趨近 0。
    固定翼受限於 R_min = V²/(g·tanφ)，典型值 30~60m。
    當掃描線間距 S < 2·R_min 時，標準半圓 U-turn 的直徑 = 2R > S，
    飛行器會侵入下一條掃描線的覆蓋區域或飛出邊界。

    解法: 使用燈泡型轉彎 (Bulb Turn) — 飛行器先向外延伸超出掃描區域，
    繞一個完整或部分圓弧後對齊下一條掃描線。這需要額外的路徑長度，
    但保證曲率始終 ≤ 1/R_min。

依賴:
    - core/base/fixed_wing_constraints.py → FixedWingConstraints
    - core/trajectory/dubins_trajectory.py → DubinsTrajectoryGenerator, Pose3D
    - sensors/camera_model.py → CameraInfo (CameraSpec), CameraCalculator, CameraDatabase
    - utils/math_utils.py → 幾何工具

作者: NCIST_planner_V1
版本: 1.0.0
"""

from __future__ import annotations

import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Sequence

from core.base.fixed_wing_constraints import FixedWingConstraints
from core.trajectory.dubins_trajectory import DubinsTrajectoryGenerator, Pose3D, DubinsPath3D
from sensors.camera_model import CameraInfo as CameraSpec, CameraCalculator, CameraDatabase
from utils.math_utils import (
    deg_to_rad, rad_to_deg, normalize_angle,
    latlon_to_meters, meters_to_latlon, polygon_area, polygon_centroid
)


# ==========================================
# 資料結構
# ==========================================
@dataclass
class ScanLine:
    """
    單一掃描線。

    Attributes:
        start: 掃描線起點 (x, y) 公尺座標
        end: 掃描線終點 (x, y) 公尺座標
        heading_rad: 掃描方向 [rad]
        index: 掃描線序號
    """
    start: Tuple[float, float]
    end: Tuple[float, float]
    heading_rad: float
    index: int

    @property
    def length(self) -> float:
        return math.hypot(self.end[0] - self.start[0], self.end[1] - self.start[1])

    @property
    def reverse_heading_rad(self) -> float:
        return self.heading_rad + math.pi


@dataclass
class CoverageResult:
    """
    覆蓋路徑規劃結果。

    Attributes:
        scan_lines: 掃描線列表
        waypoints_2d: 含轉彎的完整 2D 航點序列 [(x,y), ...]
        poses_3d: 含航向的 3D 姿態序列
        dubins_paths: 各轉彎段的 Dubins 路徑
        total_scan_length: 掃描線總長 [m]
        total_turn_length: 轉彎路段總長 [m]
        total_length: 總路徑長 [m]
        estimated_time_s: 預估飛行時間 [s]
        coverage_area_m2: 覆蓋面積 [m²]
        spacing_m: 掃描線間距 [m]
        num_scan_lines: 掃描線數量
        warnings: 警告列表
    """
    scan_lines: List[ScanLine] = field(default_factory=list)
    waypoints_2d: List[Tuple[float, float]] = field(default_factory=list)
    poses_3d: List[Pose3D] = field(default_factory=list)
    dubins_paths: List[DubinsPath3D] = field(default_factory=list)
    total_scan_length: float = 0.0
    total_turn_length: float = 0.0
    total_length: float = 0.0
    estimated_time_s: float = 0.0
    coverage_area_m2: float = 0.0
    spacing_m: float = 0.0
    num_scan_lines: int = 0
    warnings: List[str] = field(default_factory=list)

    def summary(self) -> str:
        mins = self.estimated_time_s / 60.0
        return (
            f"═══ 覆蓋路徑規劃結果 ═══\n"
            f"  掃描線數:     {self.num_scan_lines}\n"
            f"  掃描線間距:   {self.spacing_m:.1f} m\n"
            f"  掃描總長:     {self.total_scan_length:.0f} m\n"
            f"  轉彎總長:     {self.total_turn_length:.0f} m\n"
            f"  路徑總長:     {self.total_length:.0f} m\n"
            f"  預估時間:     {mins:.1f} min\n"
            f"  覆蓋面積:     {self.coverage_area_m2:.0f} m²\n"
            f"  航點數:       {len(self.poses_3d)}\n"
            f"  警告:         {len(self.warnings)} 條"
        )


# ==========================================
# 覆蓋路徑規劃器
# ==========================================
class CoveragePathPlanner:
    """
    固定翼區域覆蓋路徑規劃器。

    整合掃描線生成、燈泡型轉彎合成、Dubins 軌跡串接。

    Args:
        constraints: 固定翼物理約束
        dubins_gen: Dubins 軌跡生成器 (可選，內部會自動建立)
        altitude_m: 飛行高度 [m]
        airspeed_mps: 巡航空速 [m/s]
    """

    __slots__ = (
        '_constraints', '_dubins_gen', '_altitude', '_airspeed', '_r_min'
    )

    def __init__(
        self,
        constraints: FixedWingConstraints,
        dubins_gen: Optional[DubinsTrajectoryGenerator] = None,
        altitude_m: float = 100.0,
        airspeed_mps: Optional[float] = None,
    ) -> None:
        self._constraints = constraints
        self._dubins_gen = dubins_gen or DubinsTrajectoryGenerator(constraints)
        self._altitude = altitude_m
        self._airspeed = airspeed_mps or constraints.cruise_airspeed
        self._r_min = constraints.get_min_turn_radius()

    # --- 唯讀屬性 ---
    @property
    def constraints(self) -> FixedWingConstraints:
        return self._constraints

    @property
    def dubins_gen(self) -> DubinsTrajectoryGenerator:
        return self._dubins_gen

    @property
    def altitude(self) -> float:
        return self._altitude

    @property
    def airspeed(self) -> float:
        return self._airspeed

    @property
    def r_min(self) -> float:
        return self._r_min

    # ==========================================
    # 公開: 主規劃流程
    # ==========================================
    def plan_coverage(
        self,
        polygon: Sequence[Tuple[float, float]],
        scan_angle_deg: float = 0.0,
        spacing_m: Optional[float] = None,
        camera: Optional[CameraSpec] = None,
        side_overlap_pct: float = 60.0,
        coord_type: str = "metric",
    ) -> CoverageResult:
        """
        規劃區域覆蓋路徑。

        步驟:
            1. 計算掃描線間距 (相機自動 or 手動)
            2. 生成掃描線 (旋轉裁切法)
            3. 排序掃描線 (Boustrophedon: 蛇行)
            4. 在相鄰掃描線間插入 Dubins 轉彎
            5. 組裝完整路徑 + 效率評估

        Args:
            polygon: 凸多邊形頂點 [(x,y),...] 或 [(lat,lon),...]
            scan_angle_deg: 掃描線方向角 [度]，0=北
            spacing_m: 手動指定掃描線間距 [m] (優先於相機計算)
            camera: 相機規格 (用於自動計算間距)
            side_overlap_pct: 側向重疊率 [%]
            coord_type: "metric" 或 "latlon"

        Returns:
            CoverageResult
        """
        result = CoverageResult()

        # --- 座標轉換 ---
        if coord_type == "latlon" and polygon:
            ref_lat, ref_lon = polygon[0]
            metric_poly = [
                latlon_to_meters(lat, lon, ref_lat, ref_lon)
                for lat, lon in polygon
            ]
        else:
            metric_poly = list(polygon)

        if len(metric_poly) < 3:
            result.warnings.append("多邊形頂點不足 3 個")
            return result

        # --- 間距計算 ---
        if spacing_m is not None and spacing_m > 0:
            effective_spacing = spacing_m
        elif camera is not None:
            gw, _ = CameraCalculator.calculate_ground_coverage(
                self._altitude, camera.focal_length,
                camera.sensor_width, camera.sensor_height
            )
            effective_spacing = gw * (1.0 - side_overlap_pct / 100.0)
            effective_spacing = max(effective_spacing, 1.0)
        else:
            effective_spacing = self._r_min * 2.5  # 預設: 2.5 倍 R_min
            result.warnings.append(
                f"未提供間距或相機，預設 {effective_spacing:.1f} m"
            )
        result.spacing_m = effective_spacing

        # --- 生成掃描線 ---
        scan_lines = self.generate_scan_lines(
            metric_poly, scan_angle_deg, effective_spacing
        )
        if not scan_lines:
            result.warnings.append("無法生成掃描線")
            return result

        result.scan_lines = scan_lines
        result.num_scan_lines = len(scan_lines)
        result.total_scan_length = sum(sl.length for sl in scan_lines)

        # --- 蛇行排序 + 串接轉彎 ---
        poses, dubins_paths, turn_length, warnings = self._connect_scan_lines(
            scan_lines, effective_spacing
        )
        result.poses_3d = poses
        result.dubins_paths = dubins_paths
        result.total_turn_length = turn_length
        result.warnings.extend(warnings)

        # --- 統計 ---
        result.waypoints_2d = [(p.x, p.y) for p in poses]
        result.total_length = result.total_scan_length + result.total_turn_length
        result.coverage_area_m2 = polygon_area(metric_poly)
        result.estimated_time_s = self.estimate_task_time(result)

        return result

    # ==========================================
    # 公開: 掃描線生成
    # ==========================================
    def generate_scan_lines(
        self,
        polygon: Sequence[Tuple[float, float]],
        angle_deg: float,
        spacing: float,
    ) -> List[ScanLine]:
        """
        在多邊形內生成等間距平行掃描線。

        方法: 旋轉裁切法
            1. 將多邊形旋轉 -angle 使掃描方向對齊 Y 軸
            2. 沿 X 軸等間距放平行線
            3. 與旋轉後多邊形求交 → 掃描線段
            4. 反旋轉回原座標系

        Args:
            polygon: 凸多邊形頂點 [(x,y),...]
            angle_deg: 掃描方向角 [度]
            spacing: 掃描線間距 [m]

        Returns:
            ScanLine 列表
        """
        if len(polygon) < 3 or spacing <= 0:
            return []

        angle_rad = deg_to_rad(angle_deg)
        cos_a = math.cos(-angle_rad)
        sin_a = math.sin(-angle_rad)

        # --- 旋轉多邊形 ---
        rot_poly = [
            (x * cos_a - y * sin_a, x * sin_a + y * cos_a)
            for x, y in polygon
        ]

        xs = [p[0] for p in rot_poly]
        x_min, x_max = min(xs), max(xs)

        margin = spacing * 0.1
        x_min -= margin
        x_max += margin

        # --- 等間距掃描 ---
        scan_lines: List[ScanLine] = []
        n_lines = max(1, int(math.ceil((x_max - x_min) / spacing)) + 1)

        cos_back = math.cos(angle_rad)
        sin_back = math.sin(angle_rad)
        heading_rad = angle_rad  # 掃描方向

        for i in range(n_lines):
            sweep_x = x_min + i * spacing

            # 與旋轉多邊形的邊求交
            intersections = self._sweep_intersect(rot_poly, sweep_x)
            if len(intersections) < 2:
                continue

            intersections.sort()
            y_lo, y_hi = intersections[0], intersections[-1]

            # 反旋轉回原座標
            sx = sweep_x * cos_back - y_lo * sin_back
            sy = sweep_x * sin_back + y_lo * cos_back
            ex = sweep_x * cos_back - y_hi * sin_back
            ey = sweep_x * sin_back + y_hi * cos_back

            scan_lines.append(ScanLine(
                start=(sx, sy), end=(ex, ey),
                heading_rad=heading_rad, index=i,
            ))

        return scan_lines

    # ==========================================
    # 公開: 任務時間評估
    # ==========================================
    def estimate_task_time(self, result: CoverageResult) -> float:
        """
        評估總飛行時間。

        包含:
            - 掃描線直線段飛行時間
            - 轉彎路段飛行時間 (假設轉彎速度 = 0.8 × 巡航速度)
            - 起降緩衝 (固定 30s)

        Args:
            result: 覆蓋規劃結果

        Returns:
            預估飛行時間 [s]
        """
        if self._airspeed <= 0:
            return 0.0

        scan_time = result.total_scan_length / self._airspeed
        turn_speed = self._airspeed * 0.8
        turn_time = result.total_turn_length / turn_speed if turn_speed > 0 else 0.0
        buffer = 30.0  # 起降緩衝

        return scan_time + turn_time + buffer

    # ==========================================
    # 內部: 串接掃描線 + 轉彎合成
    # ==========================================
    def _connect_scan_lines(
        self,
        scan_lines: List[ScanLine],
        spacing: float,
    ) -> Tuple[List[Pose3D], List[DubinsPath3D], float, List[str]]:
        """
        以蛇行 (Boustrophedon) 模式串接掃描線，
        在相鄰線間插入 Dubins 轉彎。

        轉彎策略:
            - spacing >= 2·R_min → 標準 Dubins (可能是半圓 U-turn)
            - spacing < 2·R_min  → 燈泡型轉彎 (Bulb Turn):
              先沿掃描方向延伸 overshoot 距離，再用 Dubins 回接

        Returns:
            (poses, dubins_paths, total_turn_length, warnings)
        """
        poses: List[Pose3D] = []
        dubins_paths: List[DubinsPath3D] = []
        total_turn_len = 0.0
        warnings: List[str] = []
        alt = self._altitude

        for i, sl in enumerate(scan_lines):
            # 蛇行: 偶數線正向，奇數線反向
            if i % 2 == 0:
                p_start, p_end = sl.start, sl.end
                hdg = sl.heading_rad
            else:
                p_start, p_end = sl.end, sl.start
                hdg = sl.reverse_heading_rad

            # 掃描線起點
            poses.append(Pose3D(p_start[0], p_start[1], alt,
                                rad_to_deg(normalize_angle(hdg, -math.pi))))
            # 掃描線終點
            poses.append(Pose3D(p_end[0], p_end[1], alt,
                                rad_to_deg(normalize_angle(hdg, -math.pi))))

            # --- 轉彎到下一條線 ---
            if i < len(scan_lines) - 1:
                next_sl = scan_lines[i + 1]
                if (i + 1) % 2 == 0:
                    next_start = next_sl.start
                    next_hdg = next_sl.heading_rad
                else:
                    next_start = next_sl.end
                    next_hdg = next_sl.reverse_heading_rad

                # 判斷是否需要燈泡型轉彎
                turn_poses, turn_path = self._synthesize_turn(
                    exit_pos=p_end,
                    exit_heading_rad=hdg,
                    entry_pos=next_start,
                    entry_heading_rad=next_hdg,
                    spacing=spacing,
                )

                if turn_path is not None:
                    dubins_paths.append(turn_path)
                    total_turn_len += turn_path.total_length
                    if not turn_path.is_feasible:
                        warnings.append(
                            f"線 {i}→{i+1} 轉彎不可行: {turn_path.warning}"
                        )

                # 插入轉彎航點 (跳過首尾避免與掃描線重複)
                if len(turn_poses) > 2:
                    poses.extend(turn_poses[1:-1])

        return poses, dubins_paths, total_turn_len, warnings

    def _synthesize_turn(
        self,
        exit_pos: Tuple[float, float],
        exit_heading_rad: float,
        entry_pos: Tuple[float, float],
        entry_heading_rad: float,
        spacing: float,
    ) -> Tuple[List[Pose3D], Optional[DubinsPath3D]]:
        """
        合成轉彎路徑。

        燈泡型轉彎 (Bulb Turn) 幾何:
            當 spacing < 2·R_min 時，飛行器無法在掃描線間直接 U-turn。
            解法: 從掃描線終點沿原航向繼續前進 overshoot 距離，
            使得延伸後的起點與下一條線的入口距離 >= 2·R_min，
            再用標準 Dubins 連接。

            overshoot = sqrt((2R)² - spacing²)

            這形成一個「燈泡」形狀: 直線延伸 + 圓弧折返。

        Args:
            exit_pos: 掃描線終點
            exit_heading_rad: 離開航向
            entry_pos: 下一掃描線起點
            entry_heading_rad: 進入航向
            spacing: 掃描線間距

        Returns:
            (轉彎航點列表, DubinsPath3D)
        """
        alt = self._altitude
        R = self._r_min
        two_r = 2.0 * R

        # 構造 Dubins 起終點
        exit_hdg_deg = rad_to_deg(exit_heading_rad)
        entry_hdg_deg = rad_to_deg(entry_heading_rad)

        if spacing >= two_r:
            # 標準 Dubins 轉彎 (間距充足)
            start_pose = Pose3D(exit_pos[0], exit_pos[1], alt, exit_hdg_deg)
            end_pose = Pose3D(entry_pos[0], entry_pos[1], alt, entry_hdg_deg)
        else:
            # 燈泡型轉彎: 沿原航向延伸 overshoot
            if spacing > 0:
                overshoot = math.sqrt(max(0, two_r ** 2 - spacing ** 2))
            else:
                overshoot = two_r

            # 安全餘量
            overshoot *= 1.15

            # 延伸起點
            ext_x = exit_pos[0] + overshoot * math.cos(exit_heading_rad)
            ext_y = exit_pos[1] + overshoot * math.sin(exit_heading_rad)

            start_pose = Pose3D(ext_x, ext_y, alt, exit_hdg_deg)
            end_pose = Pose3D(entry_pos[0], entry_pos[1], alt, entry_hdg_deg)

            # 也需要把延伸段加為航點 (直線段)

        # 計算 Dubins 路徑
        dpath = self._dubins_gen.calculate_path(start_pose, end_pose)

        # 離散化
        turn_wps = self._dubins_gen.generate_waypoints(dpath, step_size=10.0)

        # 如果有 overshoot，在前面補上延伸直線的航點
        if spacing < two_r:
            overshoot_poses = [
                Pose3D(exit_pos[0], exit_pos[1], alt, exit_hdg_deg),
                Pose3D(start_pose.x, start_pose.y, alt, exit_hdg_deg),
            ]
            turn_wps = overshoot_poses + turn_wps
            # 更新路徑長度 (加上 overshoot 直線段)
            if dpath is not None:
                ext_len = math.hypot(
                    start_pose.x - exit_pos[0],
                    start_pose.y - exit_pos[1]
                )
                dpath.total_length += ext_len

        return turn_wps, dpath

    # ==========================================
    # 內部: 掃描線與多邊形求交
    # ==========================================
    @staticmethod
    def _sweep_intersect(
        rot_poly: List[Tuple[float, float]],
        sweep_x: float,
    ) -> List[float]:
        """
        計算垂直掃描線 x=sweep_x 與旋轉後多邊形邊的所有交點 y 座標。

        Args:
            rot_poly: 旋轉後的多邊形頂點
            sweep_x: 掃描線 x 座標

        Returns:
            交點的 y 座標列表
        """
        intersections: List[float] = []
        n = len(rot_poly)
        for i in range(n):
            x1, y1 = rot_poly[i]
            x2, y2 = rot_poly[(i + 1) % n]

            if x1 == x2:
                continue  # 平行邊

            if (sweep_x - x1) * (sweep_x - x2) > 0:
                continue  # 掃描線不在邊的 x 範圍內

            t = (sweep_x - x1) / (x2 - x1)
            if 0.0 <= t <= 1.0:
                y_intersect = y1 + t * (y2 - y1)
                intersections.append(y_intersect)

        return intersections

    # ==========================================
    # 資訊
    # ==========================================
    def __repr__(self) -> str:
        return (
            f"CoveragePathPlanner(R_min={self._r_min:.1f}m, "
            f"alt={self._altitude:.0f}m, Va={self._airspeed:.1f}m/s)"
        )


# ==========================================
# 測試
# ==========================================
if __name__ == "__main__":
    import sys
    import os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

    # --- 建立約束與生成器 ---
    fw = FixedWingConstraints(cruise_airspeed_mps=18.0, max_bank_angle_deg=45.0)
    dubins = DubinsTrajectoryGenerator(fw)
    planner = CoveragePathPlanner(fw, dubins, altitude_m=100.0)
    print(planner)
    print(f"R_min = {planner.r_min:.1f} m\n")

    # --- 定義凸多邊形 (500m × 400m 矩形) ---
    poly = [(0, 0), (500, 0), (500, 400), (0, 400)]

    # --- 規劃 (手動間距) ---
    result = planner.plan_coverage(
        polygon=poly,
        scan_angle_deg=0.0,
        spacing_m=80.0,
        coord_type="metric",
    )
    print(result.summary())

    # --- 規劃 (間距 < 2R，觸發燈泡轉彎) ---
    print("\n--- 窄間距測試 (觸發 Bulb Turn) ---")
    result2 = planner.plan_coverage(
        polygon=poly,
        scan_angle_deg=0.0,
        spacing_m=30.0,  # < 2*R_min ≈ 66m
        coord_type="metric",
    )
    print(result2.summary())
    if result2.warnings:
        print("警告:")
        for w in result2.warnings:
            print(f"  - {w}")

    # --- 規劃 (相機自動間距) ---
    print("\n--- 相機自動間距測試 ---")
    cam = CameraDatabase.CAMERAS.get("DJI Mavic 3")
    if cam:
        result3 = planner.plan_coverage(
            polygon=poly,
            scan_angle_deg=45.0,
            camera=cam,
            side_overlap_pct=60.0,
            coord_type="metric",
        )
        print(result3.summary())
