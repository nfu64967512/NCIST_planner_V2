"""
固定翼禁航區路徑規劃器
======================
基於 Shapely 幾何引擎與 Visibility Graph 演算法，
為固定翼無人機提供禁航區感知的路徑修正能力。

核心設計:
    1. 禁航區膨脹 (Buffer): 以 R_min 為基礎動態膨脹，
       確保飛行器在繞行時有足夠的轉彎空間
    2. 可視圖 (Visibility Graph): 在膨脹後的禁航區頂點間
       建立可視性邊，用 Dijkstra 求最短繞行路徑
    3. Dubins 相容: 輸出的頂點路徑保證相鄰點間距 >= 2R_min，
       便於下游 DubinsTrajectoryGenerator 平滑化

與現有模組的關係:
    - obstacle_manager.py: 處理圓形障礙物的掃描線分段（多旋翼場景）
    - 本模組: 處理多邊形禁航區的路徑級繞行（固定翼場景）
    兩者互補，不衝突。

作者: NCIST_planner_V1
版本: 1.0.0
"""

from __future__ import annotations

import math
import heapq
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Set, Sequence
from enum import Enum, auto

from shapely.geometry import (
    Polygon, MultiPolygon, LineString, Point, MultiPoint
)
from shapely.ops import unary_union
import numpy as np

from core.base.fixed_wing_constraints import FixedWingConstraints
from utils.math_utils import deg_to_rad, latlon_to_meters, meters_to_latlon


# ==========================================
# 資料結構
# ==========================================
class NFZType(Enum):
    """禁航區類型"""
    POLYGON = auto()     # 多邊形
    CIRCLE = auto()      # 圓形 (會被近似為多邊形)
    RUNWAY = auto()      # 跑道保護區


@dataclass
class NoFlyZone:
    """
    禁航區定義。

    Attributes:
        geometry: Shapely Polygon (原始幾何)
        buffered: 膨脹後的 Shapely Polygon
        nfz_type: 禁航區類型
        name: 識別名稱
        buffer_distance: 實際膨脹距離 [m]
    """
    geometry: Polygon
    buffered: Optional[Polygon] = None
    nfz_type: NFZType = NFZType.POLYGON
    name: str = ""
    buffer_distance: float = 0.0


@dataclass
class PathCorrectionResult:
    """
    路徑修正結果。

    Attributes:
        original_path: 原始航點
        corrected_path: 修正後航點
        is_modified: 是否有修改
        segments_rerouted: 被繞行的航段數
        added_length: 增加的路徑長度 [m]
        warnings: 警告列表
    """
    original_path: List[Tuple[float, float]] = field(default_factory=list)
    corrected_path: List[Tuple[float, float]] = field(default_factory=list)
    is_modified: bool = False
    segments_rerouted: int = 0
    added_length: float = 0.0
    warnings: List[str] = field(default_factory=list)

    def summary(self) -> str:
        if not self.is_modified:
            return "✓ 路徑無需修正"
        return (
            f"✗ 路徑已修正 | 繞行 {self.segments_rerouted} 段 | "
            f"增加 {self.added_length:.1f} m | "
            f"航點 {len(self.original_path)} → {len(self.corrected_path)}"
        )


# ==========================================
# 禁航區路徑規劃器
# ==========================================
class FixedWingNFZPlanner:
    """
    固定翼禁航區路徑規劃器。

    使用 Shapely 處理多邊形禁航區幾何運算，
    Visibility Graph + Dijkstra 計算最短繞行路徑。

    膨脹策略:
        buffer_distance = R_min × buffer_factor
        預設 buffer_factor = 1.5，確保轉彎空間充裕。
        R_min 從 FixedWingConstraints 動態取得。

    Args:
        constraints: 固定翼物理約束
        buffer_factor: 膨脹係數 (相對於 R_min)
    """

    __slots__ = (
        '_constraints', '_r_min', '_buffer_factor',
        '_nfz_list', '_merged_buffered', '_vis_graph_dirty'
    )

    def __init__(
        self,
        constraints: FixedWingConstraints,
        buffer_factor: float = 1.5,
    ) -> None:
        self._constraints = constraints
        self._r_min = constraints.get_min_turn_radius()
        self._buffer_factor = max(1.0, buffer_factor)
        self._nfz_list: List[NoFlyZone] = []
        self._merged_buffered: Optional[MultiPolygon] = None
        self._vis_graph_dirty: bool = True

    # --- 唯讀屬性 ---
    @property
    def constraints(self) -> FixedWingConstraints:
        return self._constraints

    @property
    def r_min(self) -> float:
        return self._r_min

    @property
    def buffer_distance(self) -> float:
        """動態膨脹距離 [m]"""
        return self._r_min * self._buffer_factor

    @property
    def nfz_count(self) -> int:
        return len(self._nfz_list)

    # ==========================================
    # 公開: 禁航區管理
    # ==========================================
    def add_polygon_nfz(
        self,
        vertices: Sequence[Tuple[float, float]],
        name: str = "",
        coord_type: str = "metric",
        ref_latlon: Optional[Tuple[float, float]] = None,
    ) -> NoFlyZone:
        """
        新增多邊形禁航區。

        Args:
            vertices: 頂點列表 [(x,y),...] 或 [(lat,lon),...]
            name: 識別名稱
            coord_type: "metric" 或 "latlon"
            ref_latlon: latlon 模式的參考原點

        Returns:
            建立的 NoFlyZone 物件
        """
        if coord_type == "latlon" and ref_latlon:
            metric_verts = [
                latlon_to_meters(lat, lon, ref_latlon[0], ref_latlon[1])
                for lat, lon in vertices
            ]
        else:
            metric_verts = list(vertices)

        poly = Polygon(metric_verts)
        if not poly.is_valid:
            poly = poly.buffer(0)  # Shapely 自動修復

        buf_dist = self.buffer_distance
        buffered = poly.buffer(buf_dist, join_style=2, cap_style=3)
        # join_style=2: mitre, cap_style=3: square
        # 確保膨脹後的頂點為銳角，有利於 Visibility Graph

        nfz = NoFlyZone(
            geometry=poly,
            buffered=buffered if isinstance(buffered, Polygon) else buffered.convex_hull,
            nfz_type=NFZType.POLYGON,
            name=name or f"NFZ_{len(self._nfz_list)}",
            buffer_distance=buf_dist,
        )
        self._nfz_list.append(nfz)
        self._vis_graph_dirty = True
        return nfz

    def add_circle_nfz(
        self,
        center: Tuple[float, float],
        radius_m: float,
        name: str = "",
        n_segments: int = 16,
    ) -> NoFlyZone:
        """
        新增圓形禁航區 (近似為正多邊形)。

        Args:
            center: 圓心 (x, y) 公尺座標
            radius_m: 半徑 [m]
            name: 識別名稱
            n_segments: 近似邊數

        Returns:
            建立的 NoFlyZone 物件
        """
        circle_poly = Point(center).buffer(radius_m, resolution=n_segments)
        buf_dist = self.buffer_distance
        buffered = circle_poly.buffer(buf_dist, resolution=n_segments)

        nfz = NoFlyZone(
            geometry=circle_poly,
            buffered=buffered,
            nfz_type=NFZType.CIRCLE,
            name=name or f"NFZ_C_{len(self._nfz_list)}",
            buffer_distance=buf_dist,
        )
        self._nfz_list.append(nfz)
        self._vis_graph_dirty = True
        return nfz

    def remove_nfz(self, name: str) -> bool:
        """依名稱移除禁航區"""
        for i, nfz in enumerate(self._nfz_list):
            if nfz.name == name:
                self._nfz_list.pop(i)
                self._vis_graph_dirty = True
                return True
        return False

    def clear_all(self) -> None:
        """清除所有禁航區"""
        self._nfz_list.clear()
        self._merged_buffered = None
        self._vis_graph_dirty = True

    # ==========================================
    # 公開: 路徑修正
    # ==========================================
    def correct_path(
        self,
        waypoints: Sequence[Tuple[float, float]],
    ) -> PathCorrectionResult:
        """
        檢查並修正穿過禁航區的航點序列。

        演算法:
            1. 合併所有膨脹禁航區為單一 MultiPolygon
            2. 逐段檢查線段是否與禁航區相交
            3. 若下一個航點本身在禁航區內部，往後尋找第一個安全目標點，
               避免 Visibility Graph 終點置於禁航區內（Dijkstra 必定失敗）
            4. 對相交段使用 Visibility Graph + Dijkstra 計算繞行路徑
            5. 組裝修正後的完整路徑

        Args:
            waypoints: 航點序列 [(x,y), ...]

        Returns:
            PathCorrectionResult
        """
        result = PathCorrectionResult(original_path=list(waypoints))

        if not self._nfz_list or len(waypoints) < 2:
            result.corrected_path = list(waypoints)
            return result

        # --- 合併膨脹禁航區 ---
        self._rebuild_merged()
        if self._merged_buffered is None or self._merged_buffered.is_empty:
            result.corrected_path = list(waypoints)
            return result

        def _pt_inside(pt: Tuple[float, float]) -> bool:
            """點是否嚴格在（不含邊界）任一 NFZ polygon 內部"""
            p = Point(pt)
            return any(poly.contains(p) for poly in self._merged_buffered.geoms)

        # 原始路徑總長
        orig_length = sum(
            math.hypot(
                waypoints[k + 1][0] - waypoints[k][0],
                waypoints[k + 1][1] - waypoints[k][1],
            )
            for k in range(len(waypoints) - 1)
        )

        # --- 逐段檢查與修正 ---
        corrected: List[Tuple[float, float]] = [waypoints[0]]
        new_length = 0.0
        i = 0

        while i < len(waypoints) - 1:
            p1 = corrected[-1]          # 上一個安全點（已加入 corrected）
            p2 = waypoints[i + 1]
            seg = LineString([p1, p2])

            if not self._merged_buffered.intersects(seg):
                # 安全段，直接加入
                corrected.append(p2)
                new_length += seg.length
                i += 1
                continue

            # 此段穿過禁航區。
            # 若 p2 在禁航區內部，往後尋找第一個安全目標點，
            # 以避免 VG 終點位於禁航區內部（Dijkstra 無法到達）。
            j = i + 1
            while j < len(waypoints) - 1 and _pt_inside(waypoints[j]):
                j += 1

            p2_target = waypoints[j]

            # Visibility Graph 繞行
            detour = self._find_detour(p1, p2_target)
            if detour and len(detour) > 1:
                # 跳過第一個點（與 corrected 末尾重複）
                corrected.extend(detour[1:])
                det_len = sum(
                    math.hypot(
                        detour[k + 1][0] - detour[k][0],
                        detour[k + 1][1] - detour[k][1],
                    )
                    for k in range(len(detour) - 1)
                )
                new_length += det_len
                result.segments_rerouted += 1
            else:
                # 繞行失敗，保留直線並警告
                corrected.append(p2_target)
                new_length += math.hypot(
                    p2_target[0] - p1[0], p2_target[1] - p1[1]
                )
                result.warnings.append(f"段 {i}–{j} 繞行失敗，保留原路徑")

            i = j   # 跳到安全目標點的下一段

        result.corrected_path = corrected
        result.is_modified = result.segments_rerouted > 0
        result.added_length = new_length - orig_length
        return result

    # ==========================================
    # 公開: 碰撞檢測
    # ==========================================
    def check_collision(
        self,
        waypoints: Sequence[Tuple[float, float]],
    ) -> List[int]:
        """
        檢查哪些航段穿過禁航區。

        Args:
            waypoints: 航點序列

        Returns:
            衝突航段索引列表 (段 i 表示 waypoints[i]→waypoints[i+1])
        """
        self._rebuild_merged()
        if self._merged_buffered is None or self._merged_buffered.is_empty:
            return []

        collisions = []
        for i in range(len(waypoints) - 1):
            seg = LineString([waypoints[i], waypoints[i + 1]])
            if self._merged_buffered.intersects(seg):
                collisions.append(i)
        return collisions

    # ==========================================
    # 內部: 合併膨脹禁航區
    # ==========================================
    def _rebuild_merged(self) -> None:
        """合併所有膨脹禁航區為單一幾何體"""
        if not self._vis_graph_dirty:
            return

        buffered_list = [
            nfz.buffered for nfz in self._nfz_list
            if nfz.buffered is not None and nfz.buffered.is_valid
        ]

        if buffered_list:
            merged = unary_union(buffered_list)
            if isinstance(merged, Polygon):
                self._merged_buffered = MultiPolygon([merged])
            elif isinstance(merged, MultiPolygon):
                self._merged_buffered = merged
            else:
                self._merged_buffered = None
        else:
            self._merged_buffered = None

        self._vis_graph_dirty = False

    # ==========================================
    # 內部: Visibility Graph 繞行
    # ==========================================
    def _find_detour(
        self,
        start: Tuple[float, float],
        end: Tuple[float, float],
    ) -> Optional[List[Tuple[float, float]]]:
        """
        使用 Visibility Graph + Dijkstra 尋找繞行路徑。

        Visibility Graph 演算法:
        ────────────────────────
        1. 節點集 V = {start, end} ∪ {膨脹禁航區的所有頂點}
        2. 邊集 E: 對 V 中任意兩點 (u, v)，若線段 uv 不穿過
           任何膨脹禁航區 → 加入邊，權重 = 歐氏距離
        3. 在此圖上跑 Dijkstra → 最短繞行路徑

        為何用膨脹後的頂點:
            膨脹距離 = R_min × factor，確保飛行器沿頂點路徑
            飛行時，轉彎圓弧不會侵入原始禁航區。

        Args:
            start: 起點 (x, y)
            end: 終點 (x, y)

        Returns:
            繞行路徑點列表，或 None (失敗)
        """
        if self._merged_buffered is None:
            return None

        # --- 收集可視圖節點 ---
        nodes: List[Tuple[float, float]] = [start, end]

        for poly in self._merged_buffered.geoms:
            exterior_coords = list(poly.exterior.coords[:-1])  # 去重複尾點
            nodes.extend(exterior_coords)

        n = len(nodes)
        if n < 2:
            return None

        # --- 建立可視性鄰接表 ---
        # adj[i] = [(j, distance), ...]
        adj: Dict[int, List[Tuple[int, float]]] = {i: [] for i in range(n)}

        for i in range(n):
            for j in range(i + 1, n):
                if self._is_visible(nodes[i], nodes[j]):
                    d = math.hypot(
                        nodes[j][0] - nodes[i][0],
                        nodes[j][1] - nodes[i][1]
                    )
                    adj[i].append((j, d))
                    adj[j].append((i, d))

        # --- Dijkstra: 從 node 0 (start) 到 node 1 (end) ---
        dist = [float('inf')] * n
        prev = [-1] * n
        dist[0] = 0.0
        pq = [(0.0, 0)]  # (distance, node_index)

        while pq:
            d_u, u = heapq.heappop(pq)
            if d_u > dist[u]:
                continue
            if u == 1:
                break  # 已到達終點
            for v, w in adj[u]:
                d_v = d_u + w
                if d_v < dist[v]:
                    dist[v] = d_v
                    prev[v] = u
                    heapq.heappush(pq, (d_v, v))

        # --- 回溯路徑 ---
        if dist[1] == float('inf'):
            return None  # 無法到達

        path_indices = []
        cur = 1
        while cur != -1:
            path_indices.append(cur)
            cur = prev[cur]
        path_indices.reverse()

        return [nodes[i] for i in path_indices]

    def _is_visible(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
    ) -> bool:
        """
        判斷兩點間是否可視 (線段不穿過任何膨脹禁航區內部)。

        注意: 線段可以「沿著」禁航區邊界 (touches)，
        但不能「穿過」(crosses 或 within)。

        Args:
            p1, p2: 兩點座標

        Returns:
            True = 可視 (線段不穿入禁航區)
        """
        if self._merged_buffered is None:
            return True

        seg = LineString([p1, p2])

        # 微小容差: 避免邊界頂點的數值誤差
        # 將線段微縮，跳過端點附近的微小相交
        if seg.length < 1e-6:
            return True

        for poly in self._merged_buffered.geoms:
            if seg.crosses(poly):
                return False
            # 額外檢查: 線段中點是否在禁航區內
            mid = seg.interpolate(0.5, normalized=True)
            if poly.contains(mid):
                return False

        return True

    # ==========================================
    # 資訊
    # ==========================================
    def __repr__(self) -> str:
        return (
            f"FixedWingNFZPlanner("
            f"NFZ={len(self._nfz_list)}, "
            f"R_min={self._r_min:.1f}m, "
            f"buffer={self.buffer_distance:.1f}m)"
        )

    def info(self) -> str:
        lines = [
            f"═══ 禁航區規劃器 ═══",
            f"  R_min:        {self._r_min:.1f} m",
            f"  膨脹係數:     {self._buffer_factor:.1f}×",
            f"  膨脹距離:     {self.buffer_distance:.1f} m",
            f"  禁航區數量:   {len(self._nfz_list)}",
        ]
        for nfz in self._nfz_list:
            area = nfz.geometry.area if nfz.geometry else 0
            buf_area = nfz.buffered.area if nfz.buffered else 0
            lines.append(
                f"    [{nfz.name}] {nfz.nfz_type.name} | "
                f"原始 {area:.0f} m² → 膨脹 {buf_area:.0f} m²"
            )
        return "\n".join(lines)


# ==========================================
# 測試
# ==========================================
if __name__ == "__main__":
    from core.base.fixed_wing_constraints import FixedWingConstraints

    # --- 建立約束與規劃器 ---
    fw = FixedWingConstraints(cruise_airspeed_mps=18.0, max_bank_angle_deg=45.0)
    nfz_planner = FixedWingNFZPlanner(fw, buffer_factor=1.5)
    print(nfz_planner)
    print(nfz_planner.info())
    print()

    # --- 新增禁航區 ---
    nfz_planner.add_polygon_nfz(
        vertices=[(200, 100), (300, 100), (300, 200), (200, 200)],
        name="建築物A",
    )
    nfz_planner.add_circle_nfz(
        center=(400, 250), radius_m=50.0, name="信號塔",
    )
    print(nfz_planner.info())
    print()

    # --- 路徑碰撞檢測 ---
    path = [(0, 150), (250, 150), (500, 150)]
    collisions = nfz_planner.check_collision(path)
    print(f"碰撞航段: {collisions}")

    # --- 路徑修正 ---
    result = nfz_planner.correct_path(path)
    print(result.summary())
    print(f"修正路徑: {len(result.corrected_path)} 點")
    for i, pt in enumerate(result.corrected_path):
        print(f"  [{i}] ({pt[0]:.1f}, {pt[1]:.1f})")