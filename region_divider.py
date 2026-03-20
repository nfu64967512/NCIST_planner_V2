"""
根目錄 region_divider — 向後相容重新匯出

survey_mission.py 使用 `from region_divider import RegionDivider`，
實際實作位於 core/geometry/region_divider.py。
"""

from core.geometry.region_divider import RegionDivider

__all__ = ['RegionDivider']
