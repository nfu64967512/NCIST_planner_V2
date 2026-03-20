"""
區域分割器模組
將多邊形區域分割為子區域，支援最多 6 台無人機

分割策略：
  n=1        : 不分割，直接返回原區域
  n=2,3,5    : 水平條帶（均等寬度，可設間隔）
  n=4        : 2×2 網格
  n=6        : 3×2 網格（3 列 × 2 行）

參考 RegionDivider (region_divider.py) 並擴充至 6 台
"""

import math
from typing import List, Tuple, Optional


class RegionDivider:
    """區域分割器 — 支援四邊形雙線性插值分割與任意多邊形水平條帶分割"""

    # ------------------------------------------------------------------
    # 雙線性插值（四邊形專用）
    # ------------------------------------------------------------------
    @staticmethod
    def bilinear_interpolation(
        corners: List[Tuple[float, float]], u: float, v: float
    ) -> Tuple[float, float]:
        """
        四邊形雙線性插值

        corners 順序：左下(p0), 右下(p1), 右上(p2), 左上(p3)
        u ∈ [0,1]：水平方向（左→右）
        v ∈ [0,1]：垂直方向（下→上）
        """
        if len(corners) != 4:
            raise ValueError("雙線性插值需要 4 個角點")

        p0, p1, p2, p3 = corners
        x = (1-u)*(1-v)*p0[0] + u*(1-v)*p1[0] + u*v*p2[0] + (1-u)*v*p3[0]
        y = (1-u)*(1-v)*p0[1] + u*(1-v)*p1[1] + u*v*p2[1] + (1-u)*v*p3[1]
        return (x, y)

    # ------------------------------------------------------------------
    # 四邊形分割（雙線性插值）
    # ------------------------------------------------------------------
    @staticmethod
    def subdivide_rectangle(
        corners: List[Tuple[float, float]],
        n: int,
        spacing_m: float = 0.0,
        gap_spacings_m: Optional[List[float]] = None,
        v_spacing_m: Optional[float] = None,
    ) -> List[List[Tuple[float, float]]]:
        """
        分割四邊形區域為 n 個子區域

        n=1        : 原樣返回
        n=2,3,5    : 水平條帶（沿 u 軸等分）
        n=4        : 2×2 網格
        n=6        : 3×2 網格（3 列 × 2 行）

        參數:
            corners          : 四個角點 [(lat,lon)×4]，順序：左下、右下、右上、左上
            n                : 分割數量（1–6）
            spacing_m        : 預設間隔（公尺），作為所有間隔的回退值
            gap_spacings_m   : 條帶模式（n=2,3,5）各邊界間隔列表，長度 = n-1
            v_spacing_m      : 網格模式（n=4,6）垂直方向間隔（公尺）；
                               水平方向使用 spacing_m

        返回:
            子區域列表，每個子區域為 4 個角點
        """
        if len(corners) != 4:
            raise ValueError("矩形分割需要 4 個角點")
        if not 1 <= n <= 6:
            raise ValueError(f"n 必須在 1–6 之間，收到 {n}")

        if n == 1:
            return [corners]

        # 計算參考寬度與高度
        w1 = RegionDivider._distance(corners[0], corners[1])
        w2 = RegionDivider._distance(corners[3], corners[2])
        avg_w = (w1 + w2) / 2 if (w1 + w2) > 0 else 1.0

        h1 = RegionDivider._distance(corners[0], corners[3])
        h2 = RegionDivider._distance(corners[1], corners[2])
        avg_h = (h1 + h2) / 2 if (h1 + h2) > 0 else 1.0

        # 預設水平間隔比例（最多 15%）
        default_h_ratio = min(0.15, spacing_m / avg_w) if spacing_m > 0 else 0.0
        # 垂直間隔比例（網格模式）
        v_ratio = min(0.15, v_spacing_m / avg_h) if v_spacing_m and v_spacing_m > 0 else default_h_ratio

        bi = RegionDivider.bilinear_interpolation
        regions: List[List[Tuple[float, float]]] = []

        if n in (2, 3, 5):
            # ── 水平條帶：支援每條邊界各自設定間隔 ─────────────
            n_gaps = n - 1
            if gap_spacings_m and len(gap_spacings_m) >= n_gaps:
                gap_ratios = [min(0.15, g / avg_w) if avg_w > 0 else 0.0
                              for g in gap_spacings_m[:n_gaps]]
            else:
                gap_ratios = [default_h_ratio] * n_gaps

            total_gap = sum(gap_ratios)
            seg = max(0.01, (1.0 - total_gap) / n)

            u_pos = 0.0
            for i in range(n):
                u0 = max(0.0, min(1.0, u_pos))
                u1 = max(0.0, min(1.0, u_pos + seg))
                if u0 < u1:
                    regions.append([
                        bi(corners, u0, 0),
                        bi(corners, u1, 0),
                        bi(corners, u1, 1),
                        bi(corners, u0, 1),
                    ])
                u_pos = u1
                if i < n_gaps:
                    u_pos += gap_ratios[i]

        elif n == 4:
            # ── 2×2 網格：水平用 spacing_m，垂直用 v_spacing_m ──
            regions = RegionDivider._grid(corners, cols=2, rows=2,
                                          h_spacing_ratio=default_h_ratio,
                                          v_spacing_ratio=v_ratio)

        elif n == 6:
            # ── 3×2 網格（3 列 × 2 行）───────────────────────────
            regions = RegionDivider._grid(corners, cols=3, rows=2,
                                          h_spacing_ratio=default_h_ratio,
                                          v_spacing_ratio=v_ratio)

        return regions

    @staticmethod
    def _grid(
        corners: List[Tuple[float, float]],
        cols: int,
        rows: int,
        spacing_ratio: float = 0.0,
        h_spacing_ratio: Optional[float] = None,
        v_spacing_ratio: Optional[float] = None,
    ) -> List[List[Tuple[float, float]]]:
        """
        通用矩形網格分割輔助函式，支援水平/垂直獨立間隔

        分割順序：row 0 col 0, row 0 col 1, ..., row 1 col 0, ...
        """
        bi = RegionDivider.bilinear_interpolation
        h_sr = h_spacing_ratio if h_spacing_ratio is not None else spacing_ratio
        v_sr = v_spacing_ratio if v_spacing_ratio is not None else spacing_ratio

        total_u_gap = h_sr * (cols - 1)
        total_v_gap = v_sr * (rows - 1)
        cell_u = max(0.01, (1.0 - total_u_gap) / cols)
        cell_v = max(0.01, (1.0 - total_v_gap) / rows)

        regions: List[List[Tuple[float, float]]] = []
        for row in range(rows):
            for col in range(cols):
                u0 = max(0.0, min(1.0, col * (cell_u + h_sr)))
                u1 = max(0.0, min(1.0, u0 + cell_u))
                v0 = max(0.0, min(1.0, row * (cell_v + v_sr)))
                v1 = max(0.0, min(1.0, v0 + cell_v))
                if u0 >= u1 or v0 >= v1:
                    continue
                regions.append([
                    bi(corners, u0, v0),
                    bi(corners, u1, v0),
                    bi(corners, u1, v1),
                    bi(corners, u0, v1),
                ])
        return regions

    # ------------------------------------------------------------------
    # 任意多邊形分割（水平條帶）
    # ------------------------------------------------------------------
    @staticmethod
    def subdivide_polygon(
        corners: List[Tuple[float, float]],
        n: int,
        spacing_m: float = 0.0
    ) -> List[List[Tuple[float, float]]]:
        """
        分割任意多邊形為 n 條水平條帶

        適用於非四邊形多邊形；座標單位為經緯度，
        spacing_m 以公尺計並自動換算為緯度度數。

        參數:
            corners   : 多邊形頂點列表（經緯度）
            n         : 分割數量（1–6）
            spacing_m : 子區域間隔（公尺）

        返回:
            子區域頂點列表
        """
        if not 1 <= n <= 6:
            raise ValueError(f"n 必須在 1–6 之間，收到 {n}")

        if n == 1:
            return [corners]

        min_y = min(p[1] for p in corners)
        max_y = max(p[1] for p in corners)
        total_h = max_y - min_y

        # 間隔換算（公尺 → 度，1° ≈ 111111 m）
        if spacing_m > 0:
            sp_deg = spacing_m / 111111.0
            total_sp = sp_deg * (n - 1)
            if total_sp >= total_h * 0.5:
                sp_deg = total_h * 0.1 / (n - 1)
                total_sp = sp_deg * (n - 1)
        else:
            sp_deg = 0.0
            total_sp = 0.0

        strip_h = (total_h - total_sp) / n
        regions: List[List[Tuple[float, float]]] = []

        for i in range(n):
            y0 = max(min_y, min_y + i * (strip_h + sp_deg))
            y1 = min(max_y, y0 + strip_h)
            if y0 >= y1:
                continue

            strip_pts: List[Tuple[float, float]] = []
            for j in range(len(corners)):
                x1, y1_ = corners[j]
                x2, y2 = corners[(j + 1) % len(corners)]

                for y_cut in (y0, y1):
                    if (y1_ <= y_cut <= y2) or (y2 <= y_cut <= y1_):
                        if abs(y2 - y1_) > 1e-10:
                            xi = x1 + (y_cut - y1_) * (x2 - x1) / (y2 - y1_)
                            strip_pts.append((xi, y_cut))

                if y0 <= y1_ <= y1:
                    strip_pts.append((x1, y1_))

            if len(strip_pts) >= 3:
                cx = sum(p[0] for p in strip_pts) / len(strip_pts)
                cy = sum(p[1] for p in strip_pts) / len(strip_pts)
                strip_pts.sort(key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
                regions.append(strip_pts)
            else:
                regions.append(corners)

        return regions if regions else [corners]

    # ------------------------------------------------------------------
    # 輔助
    # ------------------------------------------------------------------
    @staticmethod
    def _distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """兩點近似距離（公尺）"""
        dlat = p2[0] - p1[0]
        dlon = p2[1] - p1[1]
        return math.sqrt(dlat**2 + dlon**2) * 111111.0
