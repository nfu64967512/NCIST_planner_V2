"""
Tkinter 地圖組件
使用 tkintermapview 實現互動式地圖，嵌入 PyQt6
"""

import tkinter as tk
from tkinter import ttk
from typing import List, Tuple, Callable, Optional

try:
    import tkintermapview
    TKINTERMAPVIEW_AVAILABLE = True
except ImportError:
    TKINTERMAPVIEW_AVAILABLE = False

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QFrame
from PyQt6.QtCore import pyqtSignal, QTimer

from config import get_settings
from utils.logger import get_logger

settings = get_settings()
logger = get_logger()


class TkinterMapWidget(QWidget):
    """
    基於 tkintermapview 的地圖組件

    透過嵌入 Tkinter 視窗實現點擊功能
    """

    # 信號定義
    corner_added = pyqtSignal(float, float)
    corner_moved = pyqtSignal(int, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.corners: List[Tuple[float, float]] = []
        self.markers = []
        self.polygon = None
        self.path_line = None
        self.edit_mode = True

        # Tkinter 相關
        self.tk_root = None
        self.map_widget = None

        self.init_ui()

    def init_ui(self):
        """初始化 UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if not TKINTERMAPVIEW_AVAILABLE:
            from PyQt6.QtWidgets import QLabel
            label = QLabel("請安裝 tkintermapview:\npip install tkintermapview")
            label.setStyleSheet("color: red; font-size: 14px; padding: 20px;")
            layout.addWidget(label)
            logger.error("tkintermapview 未安裝")
            return

        # 創建嵌入框架
        self.frame = QFrame()
        layout.addWidget(self.frame)

        # 延遲初始化 Tkinter（需要在事件循環中）
        QTimer.singleShot(100, self.init_tkinter_map)

    def init_tkinter_map(self):
        """初始化 Tkinter 地圖"""
        try:
            # 創建 Tkinter 視窗（嵌入模式）
            self.tk_root = tk.Tk()
            self.tk_root.withdraw()  # 隱藏主視窗

            # 創建 Toplevel 作為地圖容器
            self.tk_window = tk.Toplevel(self.tk_root)
            self.tk_window.title("地圖")
            self.tk_window.geometry("800x600")

            # 創建地圖組件
            self.map_widget = tkintermapview.TkinterMapView(
                self.tk_window,
                width=800,
                height=600,
                corner_radius=0
            )
            self.map_widget.pack(fill="both", expand=True)

            # 設置預設位置
            self.map_widget.set_position(
                settings.map.default_lat,
                settings.map.default_lon
            )
            self.map_widget.set_zoom(settings.map.default_zoom)

            # 設置地圖伺服器（Google 衛星）
            try:
                self.map_widget.set_tile_server(
                    "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
                    max_zoom=20
                )
            except Exception:
                logger.warning("無法設置 Google 衛星圖，使用預設地圖")

            # 綁定點擊事件
            self.map_widget.add_left_click_map_command(self.on_map_click)

            # 啟動 Tkinter 事件處理
            self._process_tk_events()

            logger.info("Tkinter 地圖初始化成功")

        except Exception as e:
            logger.error(f"Tkinter 地圖初始化失敗: {e}")

    def _process_tk_events(self):
        """處理 Tkinter 事件（需要定期調用）"""
        if self.tk_root:
            try:
                self.tk_root.update()
            except tk.TclError:
                return
            QTimer.singleShot(50, self._process_tk_events)

    def on_map_click(self, coords):
        """處理地圖點擊事件"""
        if not self.edit_mode:
            return

        lat, lon = coords
        logger.info(f"地圖點擊: ({lat:.6f}, {lon:.6f})")

        # 添加角點
        self.add_corner(lat, lon)
        self.corner_added.emit(lat, lon)

    def add_corner(self, lat: float, lon: float):
        """新增邊界點"""
        index = len(self.corners)
        self.corners.append((lat, lon))

        if self.map_widget:
            # 添加標記
            marker = self.map_widget.set_marker(
                lat, lon,
                text=f"P{index + 1}",
                marker_color_circle="green",
                marker_color_outside="darkgreen"
            )
            self.markers.append(marker)

            # 更新多邊形
            self._update_polygon()

        logger.info(f"新增邊界點 #{index + 1}: ({lat:.6f}, {lon:.6f})")

    def _update_polygon(self):
        """更新多邊形顯示"""
        if not self.map_widget or len(self.corners) < 3:
            return

        # 刪除舊多邊形
        if self.polygon:
            self.polygon.delete()

        # 創建新多邊形
        self.polygon = self.map_widget.set_polygon(
            self.corners,
            fill_color="green",
            outline_color="darkgreen",
            border_width=2,
            name="boundary"
        )

    def clear_corners(self):
        """清除所有角點"""
        # 刪除標記
        for marker in self.markers:
            marker.delete()
        self.markers.clear()

        # 刪除多邊形
        if self.polygon:
            self.polygon.delete()
            self.polygon = None

        self.corners.clear()
        logger.info("已清除所有角點")

    def clear_paths(self):
        """清除路徑"""
        if self.path_line:
            self.path_line.delete()
            self.path_line = None
        logger.info("已清除路徑")

    def display_survey(self, survey_mission):
        """顯示飛行路徑"""
        if not self.map_widget:
            return

        self.clear_paths()

        try:
            waypoint_seq = survey_mission.waypoint_sequence
            if not waypoint_seq or len(waypoint_seq.waypoints) < 2:
                return

            # 收集航點座標
            path_coords = []
            for wp in waypoint_seq.waypoints:
                if wp.command in [16, 22]:  # NAV_WAYPOINT or TAKEOFF
                    path_coords.append((wp.lat, wp.lon))

            if len(path_coords) >= 2:
                # 繪製路徑線
                self.path_line = self.map_widget.set_path(
                    path_coords,
                    color="#08EC91",
                    width=3
                )

                # 標記起點和終點
                self.map_widget.set_marker(
                    path_coords[0][0], path_coords[0][1],
                    text="起點",
                    marker_color_circle="green"
                )
                self.map_widget.set_marker(
                    path_coords[-1][0], path_coords[-1][1],
                    text="終點",
                    marker_color_circle="red"
                )

            logger.info(f"顯示飛行路徑: {len(path_coords)} 個航點")

        except Exception as e:
            logger.error(f"顯示路徑失敗: {e}")

    def fit_bounds(self, coordinates):
        """調整視圖以包含所有座標"""
        if not self.map_widget or not coordinates:
            return

        # 計算中心點
        lats = [c[0] for c in coordinates]
        lons = [c[1] for c in coordinates]
        center_lat = sum(lats) / len(lats)
        center_lon = sum(lons) / len(lons)

        self.map_widget.set_position(center_lat, center_lon)

    def reset_view(self):
        """重置視圖"""
        if self.map_widget:
            self.map_widget.set_position(
                settings.map.default_lat,
                settings.map.default_lon
            )
            self.map_widget.set_zoom(settings.map.default_zoom)

    def set_edit_mode(self, enabled: bool):
        """設置編輯模式"""
        self.edit_mode = enabled
        logger.info(f"編輯模式: {'啟用' if enabled else '停用'}")

    def closeEvent(self, event):
        """關閉事件"""
        if self.tk_root:
            try:
                self.tk_root.destroy()
            except:
                pass
        event.accept()
