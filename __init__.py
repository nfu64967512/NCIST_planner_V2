"""
UAV Path Planner - 無人機路徑規劃系統
版本: 2.0.0
"""

__version__ = "2.0.0"
__author__ = "UAV Path Planner Team"
__license__ = "MIT"

from .config.settings import get_settings, init_settings
from .utils.logger import get_logger

__all__ = [
    'get_settings',
    'init_settings',
    'get_logger',
]
