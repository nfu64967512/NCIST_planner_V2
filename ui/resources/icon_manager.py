"""
圖示管理器模組
提供統一的圖示訪問和管理
支援 SVG、PNG、ICO 等格式
"""

from PyQt6.QtGui import QIcon, QPixmap, QPainter, QColor
from PyQt6.QtSvg import QSvgRenderer
from PyQt6.QtCore import Qt, QByteArray, QSize
from pathlib import Path
from typing import Dict, Optional
import os


class IconManager:
    """
    圖示管理器
    
    提供集中式的圖示管理和訪問
    """
    
    # 單例實例
    _instance = None
    
    # 圖示快取
    _icon_cache: Dict[str, QIcon] = {}
    
    # 圖示目錄
    _icons_dir: Optional[Path] = None
    
    def __new__(cls):
        """單例模式"""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        """初始化圖示管理器"""
        if self._initialized:
            return
        
        # 設定圖示目錄
        ui_dir = Path(__file__).parent.parent
        self._icons_dir = ui_dir / "resources" / "icons"
        
        # 確保目錄存在
        self._icons_dir.mkdir(parents=True, exist_ok=True)
        
        self._initialized = True
    
    def get_icon(self, name: str, color: Optional[str] = None) -> QIcon:
        """
        獲取圖示
        
        參數:
            name: 圖示名稱（不含副檔名）
            color: 顏色代碼（用於 SVG 圖示）
        
        返回:
            QIcon 實例
        """
        # 生成快取鍵
        cache_key = f"{name}_{color}" if color else name
        
        # 檢查快取
        if cache_key in self._icon_cache:
            return self._icon_cache[cache_key]
        
        # 嘗試載入圖示
        icon = self._load_icon(name, color)
        
        # 加入快取
        self._icon_cache[cache_key] = icon
        
        return icon
    
    def _load_icon(self, name: str, color: Optional[str] = None) -> QIcon:
        """
        載入圖示
        
        優先順序: SVG > PNG > 內建圖示
        """
        # 嘗試 SVG
        svg_path = self._icons_dir / f"{name}.svg"
        if svg_path.exists():
            return self._load_svg_icon(svg_path, color)
        
        # 嘗試 PNG
        png_path = self._icons_dir / f"{name}.png"
        if png_path.exists():
            return QIcon(str(png_path))
        
        # 使用內建圖示
        builtin_icon = self._get_builtin_icon(name, color)
        if builtin_icon:
            return builtin_icon
        
        # 返回空圖示
        return QIcon()
    
    def _load_svg_icon(self, svg_path: Path, color: Optional[str] = None) -> QIcon:
        """載入 SVG 圖示"""
        try:
            # 讀取 SVG 內容
            with open(svg_path, 'r', encoding='utf-8') as f:
                svg_content = f.read()
            
            # 如果指定顏色，替換 SVG 中的顏色
            if color:
                # 簡單的顏色替換（假設 SVG 使用 currentColor 或特定顏色）
                svg_content = svg_content.replace('currentColor', color)
                svg_content = svg_content.replace('#000000', color)
            
            # 創建 SVG 渲染器
            svg_bytes = QByteArray(svg_content.encode('utf-8'))
            renderer = QSvgRenderer(svg_bytes)
            
            # 渲染為 QPixmap
            pixmap = QPixmap(64, 64)
            pixmap.fill(Qt.GlobalColor.transparent)
            
            painter = QPainter(pixmap)
            renderer.render(painter)
            painter.end()
            
            return QIcon(pixmap)
        except Exception as e:
            print(f"載入 SVG 圖示失敗 {svg_path}: {e}")
            return QIcon()
    
    def _get_builtin_icon(self, name: str, color: Optional[str] = None) -> Optional[QIcon]:
        """
        獲取內建圖示（使用 SVG 字串定義）
        """
        svg_templates = {
            # 檔案操作
            'new': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <path d="M14 2H6a2 2 0 00-2 2v16a2 2 0 002 2h12a2 2 0 002-2V8z"/>
                <polyline points="14 2 14 8 20 8"/>
            </svg>''',
            
            'open': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <path d="M3 9h18v10a2 2 0 01-2 2H5a2 2 0 01-2-2V9z"/>
                <path d="M3 9V7a2 2 0 012-2h4l2 2h6a2 2 0 012 2v2"/>
            </svg>''',
            
            'save': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <path d="M19 21H5a2 2 0 01-2-2V5a2 2 0 012-2h11l5 5v11a2 2 0 01-2 2z"/>
                <polyline points="17 21 17 13 7 13 7 21"/>
                <polyline points="7 3 7 8 15 8"/>
            </svg>''',
            
            # 編輯操作
            'edit': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <path d="M11 4H4a2 2 0 00-2 2v14a2 2 0 002 2h14a2 2 0 002-2v-7"/>
                <path d="M18.5 2.5a2.121 2.121 0 013 3L12 15l-4 1 1-4 9.5-9.5z"/>
            </svg>''',
            
            'delete': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <polyline points="3 6 5 6 21 6"/>
                <path d="M19 6v14a2 2 0 01-2 2H7a2 2 0 01-2-2V6m3 0V4a2 2 0 012-2h4a2 2 0 012 2v2"/>
                <line x1="10" y1="11" x2="10" y2="17"/>
                <line x1="14" y1="11" x2="14" y2="17"/>
            </svg>''',
            
            # 地圖操作
            'map': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <polygon points="3 6 9 3 15 6 21 3 21 18 15 21 9 18 3 21"/>
                <line x1="9" y1="3" x2="9" y2="18"/>
                <line x1="15" y1="6" x2="15" y2="21"/>
            </svg>''',
            
            'marker': '''<svg viewBox="0 0 24 24" fill="{color}" stroke="{color}" stroke-width="2">
                <path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7z"/>
                <circle cx="12" cy="9" r="2.5" fill="white"/>
            </svg>''',
            
            # 任務操作
            'preview': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"/>
                <circle cx="12" cy="12" r="3"/>
            </svg>''',
            
            'export': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <path d="M21 15v4a2 2 0 01-2 2H5a2 2 0 01-2-2v-4"/>
                <polyline points="7 10 12 15 17 10"/>
                <line x1="12" y1="15" x2="12" y2="3"/>
            </svg>''',
            
            'clear': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <line x1="18" y1="6" x2="6" y2="18"/>
                <line x1="6" y1="6" x2="18" y2="18"/>
            </svg>''',
            
            # 設定
            'settings': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <circle cx="12" cy="12" r="3"/>
                <path d="M12 1v6m0 6v6M5.64 5.64l4.24 4.24m4.24 4.24l4.24 4.24M1 12h6m6 0h6M5.64 18.36l4.24-4.24m4.24-4.24l4.24-4.24"/>
            </svg>''',
            
            'camera': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <path d="M23 19a2 2 0 01-2 2H3a2 2 0 01-2-2V8a2 2 0 012-2h4l2-3h6l2 3h4a2 2 0 012 2z"/>
                <circle cx="12" cy="13" r="4"/>
            </svg>''',
            
            'drone': '''<svg viewBox="0 0 24 24" fill="{color}" stroke="{color}" stroke-width="1.5">
                <circle cx="12" cy="12" r="3"/>
                <circle cx="4" cy="4" r="2"/>
                <circle cx="20" cy="4" r="2"/>
                <circle cx="4" cy="20" r="2"/>
                <circle cx="20" cy="20" r="2"/>
                <line x1="12" y1="9" x2="4" y2="4"/>
                <line x1="12" y1="9" x2="20" y2="4"/>
                <line x1="12" y1="15" x2="4" y2="20"/>
                <line x1="12" y1="15" x2="20" y2="20"/>
            </svg>''',
            
            # 資訊
            'info': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <circle cx="12" cy="12" r="10"/>
                <line x1="12" y1="16" x2="12" y2="12"/>
                <line x1="12" y1="8" x2="12.01" y2="8"/>
            </svg>''',
            
            'help': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <circle cx="12" cy="12" r="10"/>
                <path d="M9.09 9a3 3 0 015.83 1c0 2-3 3-3 3"/>
                <line x1="12" y1="17" x2="12.01" y2="17"/>
            </svg>''',
            
            'warning': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <path d="M10.29 3.86L1.82 18a2 2 0 001.71 3h16.94a2 2 0 001.71-3L13.71 3.86a2 2 0 00-3.42 0z"/>
                <line x1="12" y1="9" x2="12" y2="13"/>
                <line x1="12" y1="17" x2="12.01" y2="17"/>
            </svg>''',
            
            'success': '''<svg viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="2">
                <circle cx="12" cy="12" r="10"/>
                <polyline points="9 12 11 14 15 10"/>
            </svg>''',
        }
        
        if name not in svg_templates:
            return None
        
        # 設定預設顏色
        if color is None:
            color = "#2196F3"
        
        # 替換顏色
        svg_content = svg_templates[name].replace('{color}', color)
        
        # 創建圖示
        try:
            svg_bytes = QByteArray(svg_content.encode('utf-8'))
            renderer = QSvgRenderer(svg_bytes)
            
            pixmap = QPixmap(64, 64)
            pixmap.fill(Qt.GlobalColor.transparent)
            
            painter = QPainter(pixmap)
            renderer.render(painter)
            painter.end()
            
            return QIcon(pixmap)
        except Exception as e:
            print(f"創建內建圖示失敗 {name}: {e}")
            return None
    
    def create_colored_icon(self, name: str, color: str) -> QIcon:
        """
        創建指定顏色的圖示
        
        參數:
            name: 圖示名稱
            color: 顏色代碼
        
        返回:
            QIcon 實例
        """
        return self.get_icon(name, color)
    
    def clear_cache(self):
        """清除圖示快取"""
        self._icon_cache.clear()
    
    @classmethod
    def get_instance(cls) -> 'IconManager':
        """獲取單例實例"""
        return cls()


# ==========================================
# 便捷函數
# ==========================================
def get_icon(name: str, color: Optional[str] = None) -> QIcon:
    """
    獲取圖示（便捷函數）
    
    參數:
        name: 圖示名稱
        color: 顏色代碼（可選）
    
    返回:
        QIcon 實例
    """
    manager = IconManager.get_instance()
    return manager.get_icon(name, color)


def get_colored_icon(name: str, color: str) -> QIcon:
    """
    獲取指定顏色的圖示（便捷函數）
    
    參數:
        name: 圖示名稱
        color: 顏色代碼
    
    返回:
        QIcon 實例
    """
    manager = IconManager.get_instance()
    return manager.create_colored_icon(name, color)


# ==========================================
# 圖示常數
# ==========================================
class Icons:
    """圖示名稱常數"""
    # 檔案操作
    NEW = 'new'
    OPEN = 'open'
    SAVE = 'save'
    
    # 編輯操作
    EDIT = 'edit'
    DELETE = 'delete'
    CLEAR = 'clear'
    
    # 地圖操作
    MAP = 'map'
    MARKER = 'marker'
    
    # 任務操作
    PREVIEW = 'preview'
    EXPORT = 'export'
    
    # 設定
    SETTINGS = 'settings'
    CAMERA = 'camera'
    DRONE = 'drone'
    
    # 資訊
    INFO = 'info'
    HELP = 'help'
    WARNING = 'warning'
    SUCCESS = 'success'


# ==========================================
# 使用範例
# ==========================================
if __name__ == '__main__':
    from PyQt6.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget
    import sys
    
    app = QApplication(sys.argv)
    
    # 創建測試視窗
    window = QWidget()
    layout = QVBoxLayout()
    
    # 測試各種圖示
    icons_to_test = [
        ('new', "新建", "#4CAF50"),
        ('open', "開啟", "#2196F3"),
        ('save', "儲存", "#FF9800"),
        ('preview', "預覽", "#9C27B0"),
        ('export', "匯出", "#F44336"),
        ('settings', "設定", "#607D8B"),
        ('drone', "無人機", "#2196F3"),
    ]
    
    for icon_name, text, color in icons_to_test:
        btn = QPushButton(text)
        btn.setIcon(get_icon(icon_name, color))
        btn.setMinimumHeight(40)
        layout.addWidget(btn)
    
    window.setLayout(layout)
    window.setWindowTitle("圖示管理器測試")
    window.resize(300, 400)
    window.show()
    
    sys.exit(app.exec())
