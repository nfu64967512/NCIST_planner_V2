"""
全局配置管理模組
提供系統級配置、路徑配置、地圖配置等
"""

import os
import json
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Dict, Any, Optional


@dataclass
class PathSettings:
    """路徑配置"""
    # 專案根目錄
    project_root: str = str(Path(__file__).parent.parent)
    
    # 資料目錄
    data_dir: str = None
    config_dir: str = None
    cache_dir: str = None
    export_dir: str = None
    log_dir: str = None
    
    # 資源目錄
    icons_dir: str = None
    styles_dir: str = None
    
    def __post_init__(self):
        """初始化後自動設定路徑"""
        if self.data_dir is None:
            self.data_dir = os.path.join(self.project_root, "data")
        if self.config_dir is None:
            self.config_dir = os.path.join(self.project_root, "config")
        if self.cache_dir is None:
            self.cache_dir = os.path.join(self.data_dir, "cache")
        if self.export_dir is None:
            self.export_dir = os.path.join(self.data_dir, "exports")
        if self.log_dir is None:
            self.log_dir = os.path.join(self.data_dir, "logs")
        if self.icons_dir is None:
            self.icons_dir = os.path.join(self.project_root, "ui", "resources", "icons")
        if self.styles_dir is None:
            self.styles_dir = os.path.join(self.project_root, "ui", "resources", "styles")
        
        # 確保目錄存在
        self._ensure_dirs()
    
    def _ensure_dirs(self):
        """確保所有目錄存在"""
        for dir_path in [self.data_dir, self.config_dir, self.cache_dir, 
                        self.export_dir, self.log_dir]:
            os.makedirs(dir_path, exist_ok=True)


@dataclass
class MapSettings:
    """地圖配置"""
    # 預設位置
    default_lat: float = 23.702732
    default_lon: float = 120.419333
    default_zoom: int = 20  # 預設縮放 20 倍
    max_zoom: int = 22      # 最大縮放
    
    # 地圖伺服器
    map_servers: list = None
    
    # 快取設定
    enable_cache: bool = True
    cache_size_mb: int = 500
    
    def __post_init__(self):
        """初始化地圖伺服器列表"""
        if self.map_servers is None:
            self.map_servers = [
                {
                    "name": "Google衛星",
                    "url": "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
                    "max_zoom": 22
                },
                {
                    "name": "OpenStreetMap",
                    "url": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
                    "max_zoom": 19
                },
                {
                    "name": "Google地圖",
                    "url": "https://mt1.google.com/vt/lyrs=m&x={x}&y={y}&z={z}",
                    "max_zoom": 22
                }
            ]


@dataclass
class ExportSettings:
    """匯出配置"""
    # MAVLink 設定
    mavlink_version: str = "QGC WPL 110"
    include_home_point: bool = True
    include_rtl: bool = True
    
    # 檔案格式
    waypoint_extension: str = ".waypoints"
    mission_extension: str = ".mission"
    
    # 匯出選項
    auto_generate_briefing: bool = True
    include_metadata: bool = True
    compress_output: bool = False
    
    # 座標精度
    coordinate_precision: int = 6  # 小數點後6位
    altitude_precision: int = 2    # 小數點後2位


@dataclass
class PerformanceSettings:
    """效能配置"""
    # 渲染設定
    max_waypoints_display: int = 5000
    path_simplification_tolerance: float = 0.5  # 公尺
    
    # 計算設定
    max_iterations: int = 10000
    convergence_threshold: float = 0.001
    
    # 多執行緒
    enable_multiprocessing: bool = True
    max_workers: int = 4


@dataclass
class SafetySettings:
    """安全配置"""
    # 高度限制
    min_altitude_m: float = 5.0
    max_altitude_m: float = 500.0
    
    # 速度限制
    min_speed_mps: float = 0.1
    max_speed_mps: float = 30.0
    
    # 間距限制
    min_spacing_m: float = 0.5
    max_spacing_m: float = 100.0
    
    # 安全距離
    default_safety_distance_m: float = 5.0
    min_safety_distance_m: float = 1.0
    
    # 群飛設定
    swarm_safety_distance_m: float = 10.0
    rtl_altitude_increment_m: float = 3.0


@dataclass
class UISettings:
    """UI配置"""
    # 視窗設定
    window_title: str = "無人機網格航線規劃工具"
    window_width: int = 1600
    window_height: int = 1000
    min_window_width: int = 1000
    min_window_height: int = 700
    
    # 主題
    theme: str = "modern"  # modern, dark, light
    
    # 路徑顯示
    default_path_width: int = 4
    path_colors: list = None
    
    # 區域顯示
    region_border_width: int = 2
    region_border_color: str = "#6aa84f"
    region_fill_colors: list = None
    
    def __post_init__(self):
        """初始化顏色列表"""
        if self.path_colors is None:
            self.path_colors = [
                "#08EC91", "#2DB7F5", "#F76560", "#F7BA1E",
                "#8A2BE2", "#00C2B8", "#FF8F1F", "#6C8CFF"
            ]
        if self.region_fill_colors is None:
            self.region_fill_colors = [
                "#cfe8cf", "#cfe3f6", "#ffd9d6", "#fbe7c3",
                "#e3d7f7", "#d5f3ef", "#ffe4cc", "#dee4ff"
            ]


class GlobalSettings:
    """全局配置管理器"""
    
    def __init__(self, config_file: Optional[str] = None):
        """
        初始化全局配置
        
        參數:
            config_file: 配置文件路徑（可選）
        """
        # 初始化各項配置
        self.paths = PathSettings()
        self.map = MapSettings()
        self.export = ExportSettings()
        self.performance = PerformanceSettings()
        self.safety = SafetySettings()
        self.ui = UISettings()
        
        # 配置文件路徑
        self.config_file = config_file or os.path.join(
            self.paths.config_dir, "settings.json"
        )
        
        # 載入配置（如果存在）
        self.load()
    
    def load(self) -> bool:
        """
        從文件載入配置
        
        返回:
            是否成功載入
        """
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config_data = json.load(f)
                
                # 更新各項配置
                if 'paths' in config_data:
                    self.paths = PathSettings(**config_data['paths'])
                if 'map' in config_data:
                    self.map = MapSettings(**config_data['map'])
                if 'export' in config_data:
                    self.export = ExportSettings(**config_data['export'])
                if 'performance' in config_data:
                    self.performance = PerformanceSettings(**config_data['performance'])
                if 'safety' in config_data:
                    self.safety = SafetySettings(**config_data['safety'])
                if 'ui' in config_data:
                    self.ui = UISettings(**config_data['ui'])
                
                return True
        except Exception as e:
            print(f"載入配置失敗: {e}")
        
        return False
    
    def save(self) -> bool:
        """
        儲存配置到文件
        
        返回:
            是否成功儲存
        """
        try:
            config_data = {
                'paths': asdict(self.paths),
                'map': asdict(self.map),
                'export': asdict(self.export),
                'performance': asdict(self.performance),
                'safety': asdict(self.safety),
                'ui': asdict(self.ui)
            }
            
            os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
            
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=4, ensure_ascii=False)
            
            return True
        except Exception as e:
            print(f"儲存配置失敗: {e}")
            return False
    
    def reset_to_default(self):
        """重設為預設配置"""
        self.paths = PathSettings()
        self.map = MapSettings()
        self.export = ExportSettings()
        self.performance = PerformanceSettings()
        self.safety = SafetySettings()
        self.ui = UISettings()
    
    def get_dict(self) -> Dict[str, Any]:
        """獲取配置字典"""
        return {
            'paths': asdict(self.paths),
            'map': asdict(self.map),
            'export': asdict(self.export),
            'performance': asdict(self.performance),
            'safety': asdict(self.safety),
            'ui': asdict(self.ui)
        }


# 全局配置實例
_global_settings: Optional[GlobalSettings] = None


def get_settings() -> GlobalSettings:
    """
    獲取全局配置實例（單例模式）
    
    返回:
        GlobalSettings實例
    """
    global _global_settings
    if _global_settings is None:
        _global_settings = GlobalSettings()
    return _global_settings


def init_settings(config_file: Optional[str] = None) -> GlobalSettings:
    """
    初始化全局配置
    
    參數:
        config_file: 配置文件路徑（可選）
    
    返回:
        GlobalSettings實例
    """
    global _global_settings
    _global_settings = GlobalSettings(config_file)
    return _global_settings
