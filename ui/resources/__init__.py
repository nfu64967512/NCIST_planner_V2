"""
圖示資源模組
提供統一的圖示訪問介面
"""

from ui.resources.icons.icon_manager import (
    IconManager,
    Icons,
    get_icon,
    get_colored_icon
)

__all__ = [
    'IconManager',
    'Icons',
    'get_icon',
    'get_colored_icon',
]
