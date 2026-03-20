"""
主視窗模組
整合地圖、參數面板、任務面板等核心 UI 組件
"""

import sys
import math
from typing import Optional, Tuple
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QStatusBar, QToolBar, QMessageBox,
    QFileDialog, QLabel, QScrollArea, QApplication
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QIcon, QKeySequence, QShortcut

# 修正導入路徑
from config import get_settings
from utils.logger import get_logger
from utils.file_io import write_waypoints, create_waypoint_line
from mission import MissionManager
from core.global_planner.coverage_planner import CoveragePlanner, CoverageParameters, ScanPattern
from core.global_planner.astar import AStarPlanner
from core.global_planner.rrt import RRTPlanner, RRTStarPlanner
from core.global_planner.fixed_wing_planner import (
    FixedWingPlanner, FixedWingParameters, TurnRadiusChecker,
)
from core.collision import CollisionChecker

# 獲取配置和日誌實例
settings = get_settings()
logger = get_logger()

# 常數定義
MIN_CORNERS = 3   # 最少邊界點數量
MAX_CORNERS = 100  # 最大邊界點數量
_WP_LIMIT = 655   # Mission Planner 單次最大匯入航點數


class MainWindow(QMainWindow):
    """
    主視窗類
    
    整合地圖顯示、參數控制、任務管理等核心功能
    """
    
    # 信號定義
    mission_changed = pyqtSignal(object)  # 任務變更信號
    waypoints_updated = pyqtSignal(list)  # 航點更新信號
    
    def __init__(self):
        """初始化主視窗"""
        super().__init__()
        
        # 視窗基本設置（使用 settings 替代 Config）
        self.setWindowTitle(settings.ui.window_title)
        self.setGeometry(100, 100, settings.ui.window_width, settings.ui.window_height)
        self.setMinimumSize(settings.ui.min_window_width, settings.ui.min_window_height)
        
        # 初始化核心組件
        self.mission_manager = MissionManager()
        
        # 初始化變數
        self.init_variables()
        
        # 建立 UI
        self.init_ui()
        
        # 載入樣式表
        self.load_stylesheet()
        
        # 顯示歡迎信息
        self.statusBar().showMessage("無人機路徑規劃工具已就緒", 5000)
        
        logger.info("主視窗初始化完成")
    
    def init_variables(self):
        """初始化變數"""
        self.current_mission = None
        self.corners = []  # 邊界點
        self.waypoints = []  # 航點（扁平化，供單台匯出使用）
        self.spiral_waypoint_altitudes: list = []  # 螺旋掃描每航點高度（alt_step > 0 時有效）
        self.sub_paths = []  # 各子區域路徑（分割模式時使用）
        self.obstacles = []  # 障礙物

        # 當前演算法
        self.current_algorithm = 'grid'

        # 當前載具設定
        self.current_vehicle_type = '多旋翼'
        self.current_vehicle_model = 'DJI Mavic 3'

        # 飛行參數
        self.flight_params = {
            'altitude': 10.0,
            'speed': 3.0,
            'angle': 0.0,
            'spacing': 20.0,
            'yaw_speed': 60.0,
            'subdivisions': 1,
            'region_spacing': 3.0,
            'gap_spacings_m': [],        # 各邊界獨立間隔
            'v_spacing_m': 3.0,          # 網格垂直間隔
            'transit_stagger_enabled': False,
            'transit_stagger_alt': 10.0,
            'turn_radius': 50.0,  # 固定翼轉彎半徑 (m)
            'lock_heading': False,
            'heading_angle': 0.0,
            # ── 固定翼專用 ─────────────────────────────────────
            'fw_takeoff_bearing': 0.0,
            'fw_runway_length': 50.0,
            'fw_landing_bearing': 180.0,
            'fw_pattern_alt': 80.0,
            'fw_downwind_offset': 200.0,
            'fw_pattern_leg': 300.0,
            'fw_final_dist': 400.0,
            'fw_scan_mode': 'spiral',
            # ── 螺旋專用 ────────────────────────────────────────
            'spiral_curvature': 1.0,
            'spiral_alt_step': 0.0,
        }

        # 固定翼任務結果（三階段路徑）
        self.fw_mission_result: dict = {}

        # 禁航區清單（FixedWingNFZPlanner 使用）
        # 每筆格式：{'type': 'polygon'/'circle', 'name': str,
        #            'vertices': [(lat,lon),...] | 'center':(lat,lon), 'radius': float}
        self.nfz_zones: list = []

        # 螺旋/同心圓掃描圓心模式
        self.circle_center: Optional[Tuple[float, float]] = None  # (lat, lon)
        self.picking_circle_center: bool = False

        # 固定翼起飛點（跑道位置，獨立於掃描區中心）
        self.home_point: Optional[Tuple[float, float]] = None  # (lat, lon)
        self.picking_home_point: bool = False

        # 即時路徑生成設定
        self.auto_generate_path = True  # 是否自動生成路徑
        self.path_generation_timer = None  # 延遲生成計時器
        self.path_generation_delay = 300  # 延遲時間 (ms)
    
    def init_ui(self):
        """初始化 UI 組件"""
        # 創建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主佈局
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 創建分割器（地圖 | 控制面板）
        splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # 左側：地圖區域
        self.map_widget = self.create_map_widget()
        splitter.addWidget(self.map_widget)
        
        # 右側：控制面板
        control_panel = self.create_control_panel()
        splitter.addWidget(control_panel)
        
        # 依螢幕解析度自適應分割比例（地圖 60%，控制面板 40%）
        screen = QApplication.primaryScreen()
        screen_w = screen.availableGeometry().width() if screen else 1280
        map_w   = int(screen_w * 0.60)
        panel_w = int(screen_w * 0.40)
        splitter.setStretchFactor(0, 60)
        splitter.setStretchFactor(1, 40)
        splitter.setSizes([map_w, panel_w])
        
        main_layout.addWidget(splitter)
        
        # 創建工具列
        self.create_toolbar()
        
        # 創建狀態列
        self.create_statusbar()
        
        # 創建選單
        self.create_menus()

        # 設置快捷鍵
        self.setup_shortcuts()
    
    def create_map_widget(self):
        """創建地圖組件"""
        from ui.widgets.map_widget import MapWidget

        map_widget = MapWidget(self)

        # 連接信號
        map_widget.corner_added.connect(self.on_corner_added)
        map_widget.corner_moved.connect(self.on_corner_moved)

        return map_widget

    def open_click_map_window(self):
        """打開可點擊的地圖視窗（使用 tkintermapview）"""
        try:
            import tkinter as tk
            import tkintermapview

            # 創建 Tkinter 視窗
            self.tk_map_window = tk.Toplevel()
            self.tk_map_window.title("點擊地圖添加角點 - 左鍵點擊添加")
            self.tk_map_window.geometry("900x700")

            # 創建地圖
            map_widget = tkintermapview.TkinterMapView(
                self.tk_map_window,
                width=900,
                height=650,
                corner_radius=0
            )
            map_widget.pack(fill="both", expand=True)

            # 設置位置
            map_widget.set_position(
                settings.map.default_lat,
                settings.map.default_lon
            )
            map_widget.set_zoom(settings.map.default_zoom)

            # 嘗試設置 Google 衛星圖
            try:
                map_widget.set_tile_server(
                    "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
                    max_zoom=20
                )
            except:
                pass

            # 儲存標記列表
            self.tk_markers = []
            self.tk_polygon = None

            def on_click(coords):
                lat, lon = coords
                # 添加標記
                point_num = len(self.corners) + 1
                marker = map_widget.set_marker(
                    lat, lon,
                    text=f"P{point_num}",
                    marker_color_circle="green"
                )
                self.tk_markers.append(marker)

                # 添加到主視窗
                self.on_manual_corner_added(lat, lon)

                # 更新多邊形
                if len(self.corners) >= 3:
                    if hasattr(self, 'tk_polygon') and self.tk_polygon:
                        self.tk_polygon.delete()
                    self.tk_polygon = map_widget.set_polygon(
                        self.corners,
                        fill_color="green",
                        outline_color="darkgreen",
                        border_width=2
                    )

                logger.info(f"Tkinter 地圖點擊: ({lat:.6f}, {lon:.6f})")

            map_widget.add_left_click_map_command(on_click)

            # 添加說明標籤
            info_frame = tk.Frame(self.tk_map_window)
            info_frame.pack(fill="x", pady=5)
            tk.Label(
                info_frame,
                text="左鍵點擊地圖添加角點 | 滾輪縮放 | 右鍵拖曳移動",
                font=("Arial", 10)
            ).pack()

            logger.info("已打開 Tkinter 地圖視窗")

        except ImportError:
            QMessageBox.warning(
                self, "缺少套件",
                "請先安裝 tkintermapview:\n\npip install tkintermapview"
            )
        except Exception as e:
            logger.error(f"打開 Tkinter 地圖失敗: {e}")
            QMessageBox.critical(self, "錯誤", f"打開地圖視窗失敗:\n{str(e)}")
    
    def create_control_panel(self):
        """創建控制面板（右側）"""
        from ui.widgets.parameter_panel import ParameterPanel
        from ui.widgets.mission_panel import MissionPanel

        # 外層容器：垂直排列 [ScrollArea(參數面板)] + [任務面板]
        panel_widget = QWidget()
        panel_layout = QVBoxLayout(panel_widget)
        panel_layout.setContentsMargins(4, 4, 4, 4)
        panel_layout.setSpacing(6)

        # 參數面板
        self.parameter_panel = ParameterPanel(self)
        self.parameter_panel.corner_added.connect(self.on_manual_corner_added)
        self.parameter_panel.clear_corners_requested.connect(self.on_clear_corners)
        self.parameter_panel.remove_last_corner_requested.connect(self.on_delete_last_corner)
        self.parameter_panel.open_click_map_requested.connect(self.open_click_map_window)
        self.parameter_panel.pick_center_requested.connect(self.on_pick_circle_center)
        self.parameter_panel.drag_circle_requested.connect(self.on_drag_circle_mode)
        self.map_widget.circle_defined.connect(self.on_circle_drag_defined)
        self.parameter_panel.pick_home_point_requested.connect(self.on_pick_home_point)
        self.parameter_panel.clear_home_point_requested.connect(self.on_clear_home_point)
        self.parameter_panel.manage_nfz_requested.connect(self.open_nfz_dialog)

        # 將參數面板包進 QScrollArea，解決高解析度以外螢幕時內容被截斷的問題
        scroll = QScrollArea()
        scroll.setWidget(self.parameter_panel)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        # 自適應最小寬度：螢幕寬度的 22%，至少 280px
        screen = QApplication.primaryScreen()
        min_panel_w = max(280, int((screen.availableGeometry().width() if screen else 1280) * 0.22))
        scroll.setMinimumWidth(min_panel_w)
        panel_layout.addWidget(scroll, 1)   # stretch=1 讓 scroll 佔用所有剩餘高度

        # 任務面板（固定在底部，不隨捲動）
        self.mission_panel = MissionPanel(self)
        panel_layout.addWidget(self.mission_panel, 0)
        
        # 連接信號
        self.parameter_panel.parameters_changed.connect(self.on_parameters_changed)
        self.mission_panel.preview_requested.connect(self.on_preview_paths)
        self.mission_panel.export_requested.connect(self.on_export_waypoints)
        self.mission_panel.clear_requested.connect(self.on_clear_all)
        
        return panel_widget
    
    def create_toolbar(self):
        """創建工具列"""
        toolbar = QToolBar("主工具列")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        
        # 新建任務
        new_action = QAction("🆕 新建", self)
        new_action.setStatusTip("創建新任務")
        new_action.triggered.connect(self.on_new_mission)
        toolbar.addAction(new_action)
        
        # 開啟任務
        open_action = QAction("📂 開啟", self)
        open_action.setStatusTip("開啟現有任務")
        open_action.triggered.connect(self.on_open_mission)
        toolbar.addAction(open_action)
        
        # 儲存任務
        save_action = QAction("💾 儲存", self)
        save_action.setStatusTip("儲存當前任務")
        save_action.triggered.connect(self.on_save_mission)
        toolbar.addAction(save_action)
        
        toolbar.addSeparator()
        
        # 預覽路徑
        preview_action = QAction("👁 預覽", self)
        preview_action.setStatusTip("預覽飛行路徑")
        preview_action.triggered.connect(self.on_preview_paths)
        toolbar.addAction(preview_action)
        
        # 匯出航點
        export_action = QAction("📤 匯出", self)
        export_action.setStatusTip("匯出航點檔案")
        export_action.triggered.connect(self.on_export_waypoints)
        toolbar.addAction(export_action)
        
        toolbar.addSeparator()
        
        # 清除全部
        clear_action = QAction("🗑 清除", self)
        clear_action.setStatusTip("清除所有標記和路徑")
        clear_action.triggered.connect(self.on_clear_all)
        toolbar.addAction(clear_action)
    
    def create_statusbar(self):
        """創建狀態列"""
        statusbar = QStatusBar()
        self.setStatusBar(statusbar)
        
        # 添加永久顯示的資訊
        self.coord_label = QLabel("座標: --")
        statusbar.addPermanentWidget(self.coord_label)
        
        self.waypoint_label = QLabel("航點: 0")
        statusbar.addPermanentWidget(self.waypoint_label)
        
        self.distance_label = QLabel("距離: 0.0m")
        statusbar.addPermanentWidget(self.distance_label)
    
    def create_menus(self):
        """創建選單列（PyQt6 兼容版本）"""
        menubar = self.menuBar()
        
        # === 檔案選單 ===
        file_menu = menubar.addMenu("檔案(&F)")
        
        action = file_menu.addAction("新建任務")
        action.setShortcut(QKeySequence("Ctrl+N"))
        action.triggered.connect(self.on_new_mission)
        
        action = file_menu.addAction("開啟任務")
        action.setShortcut(QKeySequence("Ctrl+O"))
        action.triggered.connect(self.on_open_mission)
        
        action = file_menu.addAction("儲存任務")
        action.setShortcut(QKeySequence("Ctrl+S"))
        action.triggered.connect(self.on_save_mission)
        
        file_menu.addSeparator()
        
        action = file_menu.addAction("匯出航點")
        action.setShortcut(QKeySequence("Ctrl+E"))
        action.triggered.connect(self.on_export_waypoints)
        
        file_menu.addSeparator()
        
        action = file_menu.addAction("退出")
        action.setShortcut(QKeySequence("Ctrl+Q"))
        action.triggered.connect(self.close)
        
        # === 編輯選單 ===
        edit_menu = menubar.addMenu("編輯(&E)")
        
        action = edit_menu.addAction("清除路徑")
        action.triggered.connect(self.on_clear_paths)
        
        action = edit_menu.addAction("清除邊界")
        action.triggered.connect(self.on_clear_corners)
        
        action = edit_menu.addAction("清除全部")
        action.setShortcut(QKeySequence("Ctrl+R"))
        action.triggered.connect(self.on_clear_all)
        
        # === 檢視選單 ===
        view_menu = menubar.addMenu("檢視(&V)")
        
        action = view_menu.addAction("重置視圖")
        action.triggered.connect(self.on_reset_view)
        
        action = view_menu.addAction("顯示網格")
        action.triggered.connect(self.on_toggle_grid)
        
        # === 工具選單 ===
        tools_menu = menubar.addMenu("工具(&T)")

        action = tools_menu.addAction("🗺️ 多邊形編輯器")
        action.setShortcut(QKeySequence("Ctrl+M"))
        action.triggered.connect(self.on_open_polygon_editor)

        action = tools_menu.addAction("🖱️ 點擊地圖視窗 (Tkinter)")
        action.triggered.connect(self.open_click_map_window)

        tools_menu.addSeparator()

        action = tools_menu.addAction("相機配置")
        action.triggered.connect(self.on_camera_config)

        action = tools_menu.addAction("飛行器配置")
        action.triggered.connect(self.on_vehicle_config)

        tools_menu.addSeparator()

        action = tools_menu.addAction("障礙物管理")
        action.triggered.connect(self.on_obstacle_manager)
        
        # === 說明選單 ===
        help_menu = menubar.addMenu("說明(&H)")
        
        action = help_menu.addAction("使用說明")
        action.triggered.connect(self.on_show_help)
        
        action = help_menu.addAction("關於")
        action.triggered.connect(self.on_about)
    
    def setup_shortcuts(self):
        """設置快捷鍵"""
        # Enter 鍵 - 生成路徑
        enter_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Return), self)
        enter_shortcut.activated.connect(self.on_preview_paths)

        # 也支援小鍵盤的 Enter
        enter_shortcut2 = QShortcut(QKeySequence(Qt.Key.Key_Enter), self)
        enter_shortcut2.activated.connect(self.on_preview_paths)

        # Space 鍵 - 也可以生成路徑（備選）
        space_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        space_shortcut.activated.connect(self.on_preview_paths)

        # Escape 鍵 - 清除路徑
        esc_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Escape), self)
        esc_shortcut.activated.connect(self.on_clear_paths)

        # Delete 鍵 - 刪除最後一個角點
        del_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Delete), self)
        del_shortcut.activated.connect(self.on_delete_last_corner)

        # Backspace 鍵 - 也可以刪除最後一個角點
        backspace_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Backspace), self)
        backspace_shortcut.activated.connect(self.on_delete_last_corner)

        logger.info("快捷鍵設置完成: Enter=生成路徑, Delete=刪除角點, Esc=清除路徑")

    def _update_transit_paths(self, planner, sub_paths):
        """計算並顯示轉場路徑（若啟用高度錯層）"""
        if not self.flight_params.get('transit_stagger_enabled', False):
            self.map_widget.clear_transit_paths()
            return
        if not self.corners or not sub_paths:
            return
        # 以多邊形質心作為預設起飛點
        home_lat = sum(c[0] for c in self.corners) / len(self.corners)
        home_lon = sum(c[1] for c in self.corners) / len(self.corners)
        home = (home_lat, home_lon)

        stagger_alt = self.flight_params.get('transit_stagger_alt', 10.0)
        base_alt = self.flight_params.get('altitude', 10.0)
        transit_paths = planner.generate_transit_paths(
            home=home,
            sub_paths=sub_paths,
            base_alt=base_alt,
            stagger_alt=stagger_alt,
        )
        self.map_widget.display_transit_paths(transit_paths)
        logger.info(f"轉場路徑已更新: {len(transit_paths)} 條，高度錯層 {stagger_alt}m")

    def on_pick_circle_center(self):
        """進入圓心點選模式：下次地圖點擊將設定為圓心"""
        self.picking_circle_center = True
        self.statusBar().showMessage("點擊地圖上任意位置以設定螺旋/圓心掃描的圓心...", 0)

    def on_drag_circle_mode(self):
        """進入地圖拖曳定義圓形模式（Mission Planner 風格）"""
        self.picking_circle_center = True   # 確保點擊攔截
        self.map_widget.set_circle_draw_mode(True)
        self.statusBar().showMessage(
            "按住滑鼠左鍵並拖曳以定義圓形掃描區域（放開滑鼠確認）...", 0
        )
        logger.info("進入地圖拖曳定義圓形模式")

    def on_circle_drag_defined(self, lat: float, lon: float, radius_m: float):
        """接收地圖拖曳圓形結果（同時設定圓心 + 半徑）"""
        self.circle_center = (lat, lon)
        self.picking_circle_center = False
        # 若拖曳半徑有效，更新 flight_params
        if radius_m > 1.0:
            self.flight_params['circle_max_radius'] = radius_m
        # 更新參數面板（顯示圓心 + 半徑，並更新 spinbox）
        self.parameter_panel.set_circle_center_display(lat, lon, radius_m)
        # 地圖圓形已由 map_widget.on_circle_draw_complete 渲染，不重複渲染
        r_disp = radius_m if radius_m > 1.0 else self.flight_params.get('circle_max_radius', 200.0)
        self.statusBar().showMessage(
            f"圓形已定義：圓心 ({lat:.6f}, {lon:.6f})，半徑 {r_disp:.0f} m", 5000
        )
        logger.info(f"拖曳圓形定義：圓心={lat:.6f},{lon:.6f}, r={radius_m:.1f}m")

    def on_circle_center_set(self, lat: float, lon: float):
        """設定圓心座標並更新 UI（僅點選模式）"""
        self.circle_center = (lat, lon)
        self.picking_circle_center = False
        # 更新參數面板顯示
        self.parameter_panel.set_circle_center_display(lat, lon)
        # 在地圖上繪製掃描範圍圓形
        radius = self.flight_params.get('circle_max_radius', 200.0)
        if hasattr(self.map_widget, 'draw_circle_overlay'):
            self.map_widget.draw_circle_overlay(lat, lon, radius)
        self.statusBar().showMessage(f"圓心已設定: ({lat:.6f}, {lon:.6f})，半徑 {radius:.0f}m", 5000)
        logger.info(f"圓心掃描中心點設定: ({lat:.6f}, {lon:.6f})")

    def on_pick_home_point(self):
        """進入起飛點點選模式：下次地圖點擊將設定為起飛點"""
        self.picking_home_point = True
        self.statusBar().showMessage("點擊地圖上任意位置以設定起飛點（跑道位置）...", 0)
        logger.info("進入起飛點選模式")

    def on_home_point_set(self, lat: float, lon: float):
        """設定起飛點並更新 UI"""
        self.home_point = (lat, lon)
        self.picking_home_point = False
        self.parameter_panel.set_home_point_display(lat, lon)
        self.map_widget.set_home_point_overlay(lat, lon)
        self.statusBar().showMessage(f"起飛點已設定: ({lat:.6f}, {lon:.6f})", 5000)
        logger.info(f"起飛點設定: ({lat:.6f}, {lon:.6f})")
        if self.auto_generate_path:
            self._schedule_path_generation()

    def on_clear_home_point(self):
        """清除起飛點"""
        self.home_point = None
        self.picking_home_point = False
        self.parameter_panel.clear_home_point_display()
        if hasattr(self.map_widget, 'clear_home_point_overlay'):
            self.map_widget.clear_home_point_overlay()
        self.statusBar().showMessage("起飛點已清除", 3000)
        logger.info("起飛點已清除")

    @staticmethod
    def _haversine(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """兩點間大地距離（公尺）"""
        R = 6371000.0
        lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
        lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
        return R * 2 * math.asin(math.sqrt(a))

    def on_delete_last_corner(self):
        """刪除最後一個角點"""
        if self.corners:
            removed = self.corners.pop()
            # 同步地圖 widget 的 corners 與 markers 列表
            if self.map_widget.corners:
                self.map_widget.corners.pop()
            if self.map_widget.markers:
                self.map_widget.markers.pop()
            # 重新渲染地圖
            self.map_widget._render_map()
            # 更新 UI
            self.parameter_panel.update_corner_count(len(self.corners))
            self.update_statusbar()
            logger.info(f"刪除角點: ({removed[0]:.6f}, {removed[1]:.6f}), 剩餘 {len(self.corners)} 個")

            # 如果啟用自動生成，觸發路徑更新
            if self.auto_generate_path and len(self.corners) >= MIN_CORNERS:
                self._schedule_path_generation()

    def _schedule_path_generation(self):
        """排程延遲路徑生成（防止頻繁更新）"""
        if self.path_generation_timer:
            self.path_generation_timer.stop()

        self.path_generation_timer = QTimer()
        self.path_generation_timer.setSingleShot(True)
        self.path_generation_timer.timeout.connect(self._auto_generate_path)
        self.path_generation_timer.start(self.path_generation_delay)

    def _auto_generate_path(self):
        """自動生成路徑（靜默模式，不顯示對話框）"""
        algorithm = getattr(self, 'current_algorithm', 'grid')
        _circle_mode = algorithm in ('spiral', 'circular') and self.circle_center is not None
        if not _circle_mode and len(self.corners) < MIN_CORNERS:
            return

        try:
            # 使用 CoveragePlanner 生成路徑
            planner = CoveragePlanner()

            # 獲取當前選擇的演算法模式
            algorithm = getattr(self, 'current_algorithm', 'grid')
            if algorithm == 'grid':
                pattern = ScanPattern.GRID
            elif algorithm == 'spiral':
                pattern = ScanPattern.SPIRAL
            elif algorithm == 'circular':
                pattern = ScanPattern.CIRCULAR
            else:
                pattern = ScanPattern.GRID

            # 判斷是否為固定翼
            is_fixed_wing = self.current_vehicle_type == '固定翼'

            # 建立覆蓋參數
            params = CoverageParameters(
                spacing=self.flight_params['spacing'],
                angle=self.flight_params['angle'],
                pattern=pattern,
                is_fixed_wing=is_fixed_wing,
                turn_radius=self.flight_params.get('turn_radius', 50.0),
                smooth_turns=is_fixed_wing,
                circle_base_altitude=self.flight_params['altitude'],
            )

            # 生成覆蓋路徑（支援分區）
            n_regions = self.flight_params.get('subdivisions', 1)
            spacing_m = self.flight_params.get('region_spacing', 0.0)
            gap_spacings_m = self.flight_params.get('gap_spacings_m') or None
            v_spacing_m = self.flight_params.get('v_spacing_m', 0.0) or None

            if n_regions > 1:
                sub_paths = planner.plan_subdivided_coverage(
                    self.corners, params,
                    n_regions=n_regions,
                    spacing_m=spacing_m,
                    gap_spacings_m=gap_spacings_m,
                    v_spacing_m=v_spacing_m,
                )
                self.sub_paths = sub_paths  # 保存各子區域路徑供匯出使用
                path = [pt for sp in sub_paths for pt in sp]
                if not path:
                    return
                self.map_widget.display_paths(sub_paths, self.flight_params['altitude'])
                # 顯示轉場路徑（如啟用高度錯層）
                self._update_transit_paths(planner, sub_paths)
            else:
                self.sub_paths = []
                path = planner.plan_coverage(self.corners, params)
                if not path:
                    return
                self.map_widget.display_path(path, self.flight_params['altitude'])
                self.map_widget.clear_transit_paths()

            # 計算總飛行距離
            total_distance = 0.0
            for i in range(len(path) - 1):
                lat1, lon1 = path[i]
                lat2, lon2 = path[i + 1]
                dlat = (lat2 - lat1) * 111111.0
                dlon = (lon2 - lon1) * 111111.0 * math.cos(math.radians((lat1 + lat2) / 2))
                total_distance += math.sqrt(dlat**2 + dlon**2)

            # 儲存航點
            self.waypoints = path

            # 更新狀態列
            self.waypoint_label.setText(f"航點: {len(path)}")
            self.distance_label.setText(f"距離: {total_distance:.0f}m")

            # 啟用匯出按鈕並更新任務統計
            self.mission_panel.update_mission_stats({
                'waypoint_count': len(path),
                'total_distance': total_distance,
                'estimated_time': total_distance / max(self.flight_params['speed'], 0.1),
                'area': 0,
                'regions': self.flight_params.get('subdivisions', 1),
            })

            self.statusBar().showMessage(f"即時生成: {len(path)} 個航點, {total_distance:.0f}m", 2000)
            logger.info(f"即時路徑生成: {len(path)} 個航點")

        except Exception as e:
            logger.error(f"即時路徑生成失敗: {e}")

    def load_stylesheet(self):
        """載入樣式表"""
        try:
            from pathlib import Path
            style_path = Path(__file__).parent / "resources" / "styles" / "dark_theme.qss"

            if style_path.exists():
                with open(style_path, 'r', encoding='utf-8') as f:
                    self.setStyleSheet(f.read())
                logger.info("樣式表載入成功")
            else:
                logger.warning(f"樣式表不存在: {style_path}")
        except Exception as e:
            logger.error(f"載入樣式表失敗: {e}")
    
    # ==========================================
    # 信號處理函數
    # ==========================================
    
    def on_corner_added(self, lat, lon):
        """處理新增邊界點（從地圖點擊）"""
        # 起飛點點選模式：優先攔截（在圓心模式之前）
        if self.picking_home_point:
            if self.map_widget.corners:
                self.map_widget.corners.pop()
            if self.map_widget.markers:
                self.map_widget.markers.pop()
            self.map_widget._render_map()
            self.on_home_point_set(lat, lon)
            return

        # 圓心點選模式：攔截點擊作為掃描圓心
        if self.picking_circle_center:
            # 撤銷 map_widget 已自動加入的角點
            if self.map_widget.corners:
                self.map_widget.corners.pop()
            if self.map_widget.markers:
                self.map_widget.markers.pop()
            self.map_widget._render_map()
            self.on_circle_center_set(lat, lon)
            return

        # 檢查是否超過最大數量
        if len(self.corners) >= MAX_CORNERS:
            QMessageBox.warning(
                self, "已達上限",
                f"已達到最大邊界點數量 ({MAX_CORNERS} 個)！"
            )
            return

        self.corners.append((lat, lon))
        remaining = MAX_CORNERS - len(self.corners)
        logger.info(f"新增邊界點 #{len(self.corners)}: ({lat:.6f}, {lon:.6f}) [剩餘: {remaining}]")
        self.parameter_panel.update_corner_count(len(self.corners))
        self.update_statusbar()

        # 如果啟用自動生成，觸發路徑更新
        if self.auto_generate_path and len(self.corners) >= MIN_CORNERS:
            self._schedule_path_generation()

    def on_manual_corner_added(self, lat, lon):
        """處理手動新增邊界點（從參數面板）"""
        # 檢查是否超過最大數量
        if len(self.corners) >= MAX_CORNERS:
            QMessageBox.warning(
                self, "已達上限",
                f"已達到最大邊界點數量 ({MAX_CORNERS} 個)！"
            )
            return

        self.corners.append((lat, lon))
        # 在地圖上添加標記
        self.map_widget.add_corner(lat, lon)
        remaining = MAX_CORNERS - len(self.corners)
        logger.info(f"手動新增邊界點 #{len(self.corners)}: ({lat:.6f}, {lon:.6f}) [剩餘: {remaining}]")
        self.parameter_panel.update_corner_count(len(self.corners))
        self.update_statusbar()

        # 如果啟用自動生成，觸發路徑更新
        if self.auto_generate_path and len(self.corners) >= MIN_CORNERS:
            self._schedule_path_generation()
    
    def on_corner_moved(self, index, lat, lon):
        """處理移動邊界點"""
        if 0 <= index < len(self.corners):
            self.corners[index] = (lat, lon)
            logger.info(f"移動邊界點 #{index+1}: ({lat:.6f}, {lon:.6f})")
            self.update_statusbar()
    
    def on_parameters_changed(self, params):
        """處理參數變更"""
        # 處理演算法變更
        if 'algorithm' in params:
            self.current_algorithm = params['algorithm']
            logger.info(f"演算法變更: {self.current_algorithm}")

        # 處理載具變更
        if 'vehicle_type' in params:
            self.current_vehicle_type = params['vehicle_type']
        if 'vehicle_model' in params:
            self.current_vehicle_model = params['vehicle_model']

        # 更新飛行參數
        flight_keys = [
            'altitude', 'speed', 'angle', 'spacing', 'yaw_speed',
            'subdivisions', 'region_spacing', 'gap_spacings_m', 'v_spacing_m',
            'transit_stagger_enabled', 'transit_stagger_alt',
            'turn_radius', 'lock_heading', 'heading_angle',
            # 固定翼專用
            'fw_takeoff_bearing', 'fw_runway_length',
            'fw_landing_bearing', 'fw_pattern_alt',
            'fw_downwind_offset', 'fw_pattern_leg', 'fw_final_dist',
            'fw_scan_mode',
            # 螺旋/同心圓圓心模式
            'circle_center_lat', 'circle_center_lon',
            'circle_max_radius', 'circle_alt_step', 'circle_min_radius',
            # 螺旋專用
            'spiral_curvature', 'spiral_alt_step',
        ]
        for key in flight_keys:
            if key in params:
                self.flight_params[key] = params[key]

        # 圓心半徑變更時更新地圖圓形
        if self.circle_center and 'circle_max_radius' in params:
            c_lat, c_lon = self.circle_center
            if hasattr(self.map_widget, 'draw_circle_overlay'):
                self.map_widget.draw_circle_overlay(
                    c_lat, c_lon, self.flight_params['circle_max_radius']
                )

        logger.info(f"參數已更新: {params}")
    
    def on_preview_paths(self):
        """預覽飛行路徑"""
        algorithm = getattr(self, 'current_algorithm', 'grid')
        # 螺旋/同心圓圓心模式：不需要邊界點，只需設定圓心
        _is_circle_center_mode = (
            algorithm in ('spiral', 'circular') and self.circle_center is not None
        )
        if not _is_circle_center_mode and len(self.corners) < MIN_CORNERS:
            QMessageBox.warning(
                self, "邊界不足",
                f"需要至少 {MIN_CORNERS} 個邊界點才能生成路徑\n"
                f"（螺旋/同心圓模式請先點擊「地圖拖曳定義圓形」設定圓心）"
            )
            return

        try:
            # 獲取當前選擇的演算法
            algorithm = getattr(self, 'current_algorithm', 'grid')

            # 判斷是否為固定翼
            is_fixed_wing = self.current_vehicle_type == '固定翼'

            path = None

            # ── 固定翼三階段路徑規劃（螺旋/同心圓/網格均啟用）──────────
            # _preview_fixed_wing_mission 內部已完整處理：
            #   地圖顯示、self.waypoints、self.fw_mission_result、統計、對話框
            # 成功後直接 return，避免下方的多旋翼後處理覆蓋地圖與航點
            if is_fixed_wing and algorithm in ['spiral', 'circular', 'grid']:
                path = self._preview_fixed_wing_mission(algorithm)
                if path is None:
                    return  # 已在內部顯示錯誤
                return  # ← 固定翼任務已完整處理，跳過後續多旋翼流程

            # 根據演算法類型生成路徑
            elif algorithm in ['grid', 'spiral', 'circular']:
                # ── 螺旋/同心圓：圓心模式 ──────────────────────────────────────
                if algorithm in ('spiral', 'circular'):
                    if self.circle_center is None:
                        QMessageBox.information(
                            self, "提示",
                            "螺旋/同心圓掃描需要先設定圓心。\n\n"
                            "方法：\n"
                            "  1. 在右側參數面板按「從地圖點選圓心」\n"
                            "  2. 在地圖上點擊想要的中心位置\n\n"
                            "現已自動進入點選圓心模式，請點擊地圖。"
                        )
                        self.on_pick_circle_center()
                        return

                    # 圓心模式：直接用圓心 + 最大半徑生成路徑
                    c_lat, c_lon = self.circle_center
                    max_r    = self.flight_params.get('circle_max_radius', 200.0)
                    alt_step = self.flight_params.get('circle_alt_step', 0.0)
                    min_r    = self.flight_params.get('circle_min_radius', 0.0)
                    spacing  = self.flight_params['spacing']
                    fw       = is_fixed_wing
                    tr       = self.flight_params.get('turn_radius', 50.0)
                    alt      = self.flight_params['altitude']

                    spiral_curvature = self.flight_params.get('spiral_curvature', 1.0)
                    spiral_alt_step  = self.flight_params.get('spiral_alt_step', 0.0)

                    circle_planner = CoveragePlanner()
                    if algorithm == 'spiral':
                        path = circle_planner.plan_spiral_from_center(
                            c_lat, c_lon, max_r, spacing,
                            is_fixed_wing=fw, turn_radius=tr,
                            curvature=spiral_curvature,
                            alt_step=spiral_alt_step,
                            base_alt=alt,
                        )
                        _spiral_alts = circle_planner.get_spiral_altitudes()
                    else:  # circular
                        path = circle_planner.plan_circle_survey_from_center(
                            c_lat, c_lon, max_r, spacing,
                            base_alt=alt, alt_step=alt_step,
                            min_radius_m=min_r,
                        )
                        _spiral_alts = []

                    if not path:
                        QMessageBox.warning(self, "路徑為空", "無法生成路徑，請檢查參數設定（半徑/間距）。")
                        return

                    _orig_n = len(path)
                    if _orig_n > _WP_LIMIT:
                        path, _spiral_alts = CoveragePlanner.limit_path_waypoints(
                            path, _WP_LIMIT, _spiral_alts if _spiral_alts else None
                        )
                        _trimmed = True
                    else:
                        _trimmed = False
                    self.waypoints = path
                    self.spiral_waypoint_altitudes = _spiral_alts
                    self.sub_paths = []

                    # 確保圓形資料是最新的（不觸發額外渲染）
                    self.map_widget._circle_center = (c_lat, c_lon)
                    self.map_widget._circle_radius_m = max_r
                    # 顯示路徑（同時包含圓形，只渲染一次）
                    self.map_widget.display_path(path, alt)

                    # 統計
                    total_distance = sum(
                        self._haversine(path[i], path[i+1]) for i in range(len(path)-1)
                    )
                    _wp_txt = f"航點: {len(path)}"
                    if _trimmed:
                        _wp_txt += f" (原{_orig_n},已簡化)"
                    self.waypoint_label.setText(_wp_txt)
                    self.distance_label.setText(f"距離: {total_distance:.0f}m")
                    self.mission_panel.update_mission_stats({
                        'waypoint_count': len(path),
                        'total_distance': total_distance,
                        'estimated_time': total_distance / max(self.flight_params['speed'], 0.1),
                        'area': math.pi * max_r ** 2,
                        'regions': 1,
                    })
                    self.statusBar().showMessage(f"路徑生成完成：{len(path)} 個航點", 5000)

                    # ── 任務預覽彈窗 ──────────────────────────────────────
                    _speed = max(self.flight_params.get('speed', 15.0), 0.1)
                    _est_s = total_distance / _speed
                    _mode_name = '螺旋掃描' if algorithm == 'spiral' else '同心圓掃描'
                    _wp_note = (f"（原 {_orig_n} 點，已簡化至 {len(path)}）" if _trimmed else "")
                    QMessageBox.information(
                        self, "任務預覽",
                        f"模式: {_mode_name}\n"
                        f"中心: {c_lat:.6f}°, {c_lon:.6f}°\n"
                        f"最大半徑: {max_r:.0f} m   間距: {spacing:.0f} m\n\n"
                        f"航點數: {len(path)} {_wp_note}\n"
                        f"總飛行距離: {total_distance:.0f} m\n"
                        f"預估飛行時間: {_est_s / 60:.1f} min\n"
                        f"覆蓋面積: {math.pi * max_r ** 2 / 10000:.2f} ha"
                    )
                    return  # skip the rest of the elif block
                # ── END 圓心模式 ──────────────────────────────────────────────

                # 覆蓋路徑規劃（Grid/Spiral/Circular，多邊形模式）
                planner = CoveragePlanner()
                if algorithm == 'grid':
                    pattern = ScanPattern.GRID
                elif algorithm == 'spiral':
                    pattern = ScanPattern.SPIRAL
                else:
                    pattern = ScanPattern.CIRCULAR

                params = CoverageParameters(
                    spacing=self.flight_params['spacing'],
                    angle=self.flight_params['angle'],
                    pattern=pattern,
                    is_fixed_wing=is_fixed_wing,
                    turn_radius=self.flight_params.get('turn_radius', 50.0),
                    smooth_turns=is_fixed_wing,
                    circle_base_altitude=self.flight_params['altitude'],
                )

                n_regions = self.flight_params.get('subdivisions', 1)
                spacing_m = self.flight_params.get('region_spacing', 0.0)
                gap_spacings_m = self.flight_params.get('gap_spacings_m') or None
                v_spacing_m = self.flight_params.get('v_spacing_m', 0.0) or None

                if n_regions > 1:
                    # 分區覆蓋：各子區域獨立規劃，顯示不同顏色
                    sub_paths = planner.plan_subdivided_coverage(
                        self.corners, params,
                        n_regions=n_regions,
                        spacing_m=spacing_m,
                        gap_spacings_m=gap_spacings_m,
                        v_spacing_m=v_spacing_m,
                    )
                    self.sub_paths = sub_paths  # 保存各子區域路徑供匯出使用
                    # 扁平化供匯出使用，各子區域之間不插入多餘航點
                    path = [pt for sp in sub_paths for pt in sp]
                    self.map_widget.display_paths(sub_paths, self.flight_params['altitude'])
                    # 顯示轉場路徑
                    self._update_transit_paths(planner, sub_paths)
                else:
                    self.sub_paths = []
                    path = planner.plan_coverage(self.corners, params)
                    self.map_widget.clear_transit_paths()

            elif algorithm == 'astar':
                # A* 路徑規劃（點對點）
                if len(self.corners) >= 2:
                    astar_planner = AStarPlanner(
                        collision_checker=None,
                        step_size=self.flight_params['spacing'],
                        heuristic='euclidean'
                    )
                    path = astar_planner.plan(
                        start=self.corners[0],
                        goal=self.corners[-1],
                        boundary=self.corners
                    )

            elif algorithm in ['rrt', 'rrt_star']:
                # RRT/RRT* 路徑規劃
                if len(self.corners) >= 2:
                    # 計算搜索區域
                    lats = [c[0] for c in self.corners]
                    lons = [c[1] for c in self.corners]
                    search_area = (min(lats), min(lons), max(lats), max(lons))

                    # 創建空的碰撞檢測器（無障礙物）
                    collision_checker = CollisionChecker()

                    if algorithm == 'rrt':
                        rrt_planner = RRTPlanner(
                            collision_checker=collision_checker,
                            step_size=0.0001,  # 經緯度單位
                            goal_sample_rate=0.1,
                            max_iter=1000
                        )
                    else:
                        rrt_planner = RRTStarPlanner(
                            collision_checker=collision_checker,
                            step_size=0.0001,
                            goal_sample_rate=0.1,
                            max_iter=1000,
                            search_radius=0.0005
                        )

                    path = rrt_planner.plan(
                        start=self.corners[0],
                        goal=self.corners[-1],
                        search_area=search_area
                    )

            elif algorithm == 'dijkstra':
                # Dijkstra 使用與 A* 相同的邏輯
                if len(self.corners) >= 2:
                    astar_planner = AStarPlanner(
                        collision_checker=None,
                        step_size=self.flight_params['spacing'],
                        heuristic='euclidean',
                        heuristic_weight=0.0  # weight=0 等同於 Dijkstra
                    )
                    path = astar_planner.plan(
                        start=self.corners[0],
                        goal=self.corners[-1],
                        boundary=self.corners
                    )

            elif algorithm == 'dwa':
                # DWA 需要即時規劃，這裡生成直線路徑作為全域參考
                QMessageBox.information(
                    self, "DWA 演算法",
                    "DWA (動態窗口) 是局域即時規劃演算法，\n"
                    "需要配合飛行控制器使用。\n\n"
                    "目前生成直線路徑作為參考。"
                )
                path = self.corners.copy()

            else:
                # 預設使用 Grid
                planner = CoveragePlanner()
                params = CoverageParameters(
                    spacing=self.flight_params['spacing'],
                    angle=self.flight_params['angle'],
                    pattern=ScanPattern.GRID
                )
                path = planner.plan_coverage(self.corners, params)

            if not path:
                QMessageBox.warning(self, "路徑生成失敗", "無法生成覆蓋路徑，請檢查邊界點設定")
                return

            # 計算統計資訊（使用 CoveragePlanner 工具函數）
            coverage_planner = CoveragePlanner()
            area = coverage_planner.calculate_coverage_area(self.corners)
            mission_time = coverage_planner.estimate_mission_time(path, self.flight_params['speed'])

            # 計算總飛行距離
            total_distance = 0.0
            for i in range(len(path) - 1):
                lat1, lon1 = path[i]
                lat2, lon2 = path[i + 1]
                dlat = (lat2 - lat1) * 111111.0
                dlon = (lon2 - lon1) * 111111.0 * math.cos(math.radians((lat1 + lat2) / 2))
                total_distance += math.sqrt(dlat**2 + dlon**2)

            # 若超出 MP 匯入上限，自動簡化
            path, _trimmed, _orig_n = self._trim_to_wp_limit(path, "覆蓋路徑")

            # 儲存航點
            self.waypoints = path

            # 在地圖上顯示路徑
            # 多區域時 display_paths 已在規劃迴圈中呼叫，此處只處理單區域
            n_reg = self.flight_params.get('subdivisions', 1)
            if n_reg <= 1:
                self.map_widget.display_path(path, self.flight_params['altitude'])

            # 更新狀態列
            _wp_txt = f"航點: {len(path)}"
            if _trimmed:
                _wp_txt += f" (原{_orig_n},已簡化)"
            self.waypoint_label.setText(_wp_txt)
            self.distance_label.setText(f"距離: {total_distance:.0f}m")

            # 顯示結果
            region_info = (f"  分割區域: {n_reg} 個\n" if n_reg > 1 else "")
            _trim_note = (f" (原 {_orig_n}，已簡化至 {_WP_LIMIT} 上限)" if _trimmed else "")
            QMessageBox.information(
                self, "路徑生成完成",
                f"覆蓋路徑已生成！\n\n"
                f"邊界點: {len(self.corners)} 個\n"
                f"航點數: {len(path)} 個{_trim_note}\n"
                f"覆蓋面積: {area:.0f} m²\n"
                f"總飛行距離: {total_distance:.0f} m\n"
                f"預估飛行時間: {mission_time/60:.1f} 分鐘\n\n"
                f"參數:\n"
                f"  高度: {self.flight_params['altitude']} m\n"
                f"  速度: {self.flight_params['speed']} m/s\n"
                f"  間距: {self.flight_params['spacing']} m\n"
                f"  角度: {self.flight_params['angle']}°\n"
                f"{region_info}"
                f"  演算法: {algorithm}"
            )

            # 啟用匯出按鈕並更新任務統計
            self.mission_panel.update_mission_stats({
                'waypoint_count': len(path),
                'total_distance': total_distance,
                'estimated_time': mission_time,
                'area': area,
                'regions': self.flight_params.get('subdivisions', 1),
            })

            self.statusBar().showMessage(f"路徑生成完成：{len(path)} 個航點", 5000)
            logger.info(f"路徑生成完成：{len(path)} 個航點，距離 {total_distance:.0f}m")

        except Exception as e:
            logger.error(f"預覽失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "預覽錯誤", f"生成路徑時發生錯誤：\n{str(e)}")
    
    @staticmethod
    def _calc_bearing(lat1, lon1, lat2, lon2) -> float:
        """計算兩點間方位角（度，0=北，順時針）"""
        lat1r = math.radians(lat1)
        lat2r = math.radians(lat2)
        dlon  = math.radians(lon2 - lon1)
        y = math.sin(dlon) * math.cos(lat2r)
        x = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
        return (math.degrees(math.atan2(y, x)) + 360) % 360

    # ──────────────────────────────────────────────────────────────────────────
    # 固定翼專用：三階段任務規劃
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _trim_to_wp_limit(path, label: str = "") -> tuple:
        """
        若路徑超過 _WP_LIMIT，使用 RDP 自動簡化後回傳。
        回傳 (trimmed_path, was_trimmed, original_count)
        """
        orig_n = len(path)
        if orig_n <= _WP_LIMIT:
            return list(path), False, orig_n
        trimmed, _ = CoveragePlanner.limit_path_waypoints(path, _WP_LIMIT)
        if label:
            logger.warning(
                f"[WP 限制] {label}: {orig_n} → {len(trimmed)} 航點"
                f"（Mission Planner 上限 {_WP_LIMIT}）"
            )
        return trimmed, True, orig_n

    def _build_fw_params(self) -> FixedWingParameters:
        """從 flight_params 建立 FixedWingParameters"""
        import math
        fp       = self.flight_params
        speed    = fp.get('speed', 18.0)
        bank_deg = fp.get('fw_max_bank_deg', 45.0)
        tan_b    = math.tan(math.radians(max(bank_deg, 1.0)))
        r_min    = speed ** 2 / (9.81 * tan_b)          # R_min = V²/(g·tan φ)
        turn_radius_m = max(fp.get('turn_radius', 50.0), r_min)  # 不得低於 R_min
        return FixedWingParameters(
            cruise_speed_mps=speed,
            mission_altitude_m=fp.get('altitude', 100.0),
            turn_radius_m=turn_radius_m,
            takeoff_bearing_deg=fp.get('fw_takeoff_bearing', 0.0),
            takeoff_runway_m=fp.get('fw_runway_length', 50.0),
            takeoff_climb_alt_m=fp.get('altitude', 100.0) * 0.3,
            landing_bearing_deg=fp.get('fw_landing_bearing', 180.0),
            pattern_altitude_m=fp.get('fw_pattern_alt', 80.0),
            downwind_offset_m=fp.get('fw_downwind_offset', 200.0),
            pattern_leg_length_m=fp.get('fw_pattern_leg', 300.0),
            final_approach_dist_m=fp.get('fw_final_dist', 400.0),
            scan_spacing_m=fp.get('spacing', 50.0),
        )

    def _preview_fixed_wing_mission(self, algorithm: str):
        """
        固定翼三階段任務規劃與地圖顯示

        顯示三種顏色路徑：
          黃色 - 起飛路徑
          藍色 - 主任務（螺旋/同心圓）
          橙色 - 五邊進場降落

        返回扁平化的路徑列表（lat, lon），供統計使用；
        出錯時返回 None。
        """
        try:
            fw_params = self._build_fw_params()
            scan_mode = algorithm  # 'spiral' | 'circular' | 'grid'
            scan_angle = self.flight_params.get('angle', 0.0)   # 網格掃描角度（度）

            # 決定中心點與任務多邊形：優先使用圓心模式，其次使用邊界點
            # 網格掃描必須使用邊界多邊形，圓心模式下跳過圓形多邊形邏輯
            if scan_mode == 'grid':
                # 網格掃描需要實際多邊形邊界
                if len(self.corners) < 3:
                    QMessageBox.warning(
                        self, "缺少區域定義",
                        "固定翼網格掃描需要在地圖上點選至少 3 個邊界點。"
                    )
                    return None
                center_lat = sum(c[0] for c in self.corners) / len(self.corners)
                center_lon = sum(c[1] for c in self.corners) / len(self.corners)
                mission_polygon = self.corners
            elif self.circle_center is not None:
                center_lat, center_lon = self.circle_center
                max_r = self.flight_params.get('circle_max_radius', 200.0)
                # 以 16 邊形近似圓形多邊形，供 plan_coverage 計算半徑用
                mission_polygon = []
                for i in range(16):
                    angle = math.radians(i * 22.5)
                    dlat = max_r * math.cos(angle) / 111111.0
                    dlon = max_r * math.sin(angle) / (
                        111111.0 * math.cos(math.radians(center_lat)))
                    mission_polygon.append(
                        (center_lat + dlat, center_lon + dlon))
            elif len(self.corners) >= 3:
                center_lat = sum(c[0] for c in self.corners) / len(self.corners)
                center_lon = sum(c[1] for c in self.corners) / len(self.corners)
                mission_polygon = self.corners
            else:
                QMessageBox.warning(
                    self, "缺少區域定義",
                    "請先設定飛行區域：\n"
                    "  • 螺旋/同心圓：按「從地圖點選圓心」後點擊地圖\n"
                    "  • 網格：在地圖上點選至少 3 個邊界點"
                )
                return None

            planner = FixedWingPlanner()
            n_regions = self.flight_params.get('subdivisions', 1)
            spacing_m = self.flight_params.get('region_spacing', 0.0)

            # 起飛/降落點：優先使用獨立設定的 home_point，否則沿用掃描區中心
            if self.home_point is not None:
                home_lat, home_lon = self.home_point
            else:
                home_lat, home_lon = center_lat, center_lon

            # 進場方向 = home → survey_center（兩點距離 > 10m 才有意義）
            _dist_to_center = self._haversine((home_lat, home_lon), (center_lat, center_lon))
            if _dist_to_center > 10.0:
                _dlat_r = math.radians(center_lat - home_lat)
                _dlon_r = math.radians(center_lon - home_lon)
                _x = math.sin(_dlon_r) * math.cos(math.radians(center_lat))
                _y = (math.cos(math.radians(home_lat)) * math.sin(math.radians(center_lat))
                      - math.sin(math.radians(home_lat)) * math.cos(math.radians(center_lat))
                      * math.cos(_dlon_r))
                approach_bearing = (math.degrees(math.atan2(_x, _y)) + 360) % 360
            else:
                approach_bearing = fw_params.takeoff_bearing_deg

            # 共用起飛 / 降落路徑（不論分區與否皆相同）
            takeoff_path = planner.generate_takeoff_path(home_lat, home_lon, fw_params)
            landing_path = planner.generate_landing_path(home_lat, home_lon, fw_params)
            takeoff_ll = [(lat, lon) for lat, lon, _ in takeoff_path]
            landing_ll = [(lat, lon) for lat, lon, _ in landing_path]

            # 轉彎半徑合規性檢測（共用）
            check = TurnRadiusChecker.check(
                fw_params.turn_radius_m,
                fw_params.cruise_speed_mps,
                self.current_vehicle_model,
            )
            turn_status = {
                'ok': '符合規格',
                'warning': '接近極限（建議增大）',
                'critical': '超出極限！請立即修改',
            }.get(check['status'], '')

            def _path_dist(pts):
                return sum(self._haversine(pts[i], pts[i + 1]) for i in range(len(pts) - 1))

            if n_regions > 1:
                # ── 多分區固定翼掃描 ────────────────────────────────────────
                if scan_mode == 'spiral':
                    _pattern = ScanPattern.SPIRAL
                elif scan_mode == 'grid':
                    _pattern = ScanPattern.GRID
                else:
                    _pattern = ScanPattern.CIRCULAR
                coverage_params = CoverageParameters(
                    spacing=fw_params.scan_spacing_m,
                    angle=scan_angle,
                    pattern=_pattern,
                    is_fixed_wing=True,
                    turn_radius=fw_params.turn_radius_m,
                    smooth_turns=True,
                    circle_base_altitude=fw_params.mission_altitude_m,
                    circle_direction=fw_params.circle_direction,
                )
                cov_planner = CoveragePlanner()
                sub_scan_paths = cov_planner.plan_subdivided_coverage(
                    mission_polygon, coverage_params,
                    n_regions=n_regions,
                    spacing_m=spacing_m,
                )
                if not sub_scan_paths:
                    QMessageBox.warning(self, "路徑為空", "分區掃描路徑生成失敗，請檢查區域定義。")
                    return None

                self.fw_mission_result = None
                self.sub_paths = sub_scan_paths
                all_scan_pts = [pt for sp in sub_scan_paths for pt in sp]
                path = takeoff_ll + all_scan_pts + landing_ll
                path, _trimmed, _orig_n = self._trim_to_wp_limit(path, "固定翼多分區")
                self.waypoints = path

                self.map_widget.display_fw_paths(takeoff_ll, sub_scan_paths, landing_ll)

                takeoff_dist = _path_dist(takeoff_ll)
                landing_dist = _path_dist(landing_ll)
                scan_dists   = [_path_dist(sp) for sp in sub_scan_paths if len(sp) >= 2]
                total_dist   = takeoff_dist + sum(scan_dists) + landing_dist

                _wp_txt = f"航點: {len(path)}"
                if _trimmed:
                    _wp_txt += f" (原{_orig_n},已簡化)"
                self.waypoint_label.setText(_wp_txt)
                self.distance_label.setText(f"距離: {total_dist:.0f}m")
                self.mission_panel.update_mission_stats({
                    'waypoint_count': len(path),
                    'total_distance': total_dist,
                    'estimated_time': total_dist / max(self.flight_params.get('speed', 18.0), 0.1),
                    'area': CoveragePlanner().calculate_coverage_area(mission_polygon),
                    'regions': n_regions,
                })

                _fw_speed = max(self.flight_params.get('speed', 18.0), 0.1)
                scan_lines = "".join(
                    f"  區域 {i + 1}: {sd:.0f} m  ({sd / _fw_speed / 60:.1f} min)\n"
                    for i, sd in enumerate(scan_dists)
                )
                _takeoff_time = takeoff_dist / _fw_speed / 60
                _landing_time = landing_dist / _fw_speed / 60
                QMessageBox.information(
                    self, "固定翼多分區任務生成完成",
                    f"任務區域已分割為 {n_regions} 個子區域，各區域獨立掃描路徑。\n\n"
                    f"[起飛] {takeoff_dist:.0f} m  ({_takeoff_time:.1f} min)\n"
                    f"[掃描]\n{scan_lines}"
                    f"[降落] {landing_dist:.0f} m  ({_landing_time:.1f} min)\n\n"
                    f"總飛行距離: {total_dist:.0f} m\n"
                    f"預估總時間: {total_dist / _fw_speed / 60:.1f} min\n\n"
                    f"[轉彎半徑] {fw_params.turn_radius_m:.0f}m  "
                    f"傾斜: {check['required_bank']:.1f}°  {turn_status}"
                )
                logger.info(
                    f"固定翼多分區任務: {n_regions} 區, {len(path)} 航點, {total_dist:.0f}m"
                )
                return path

            else:
                # ── 單區域固定翼任務（三階段完整生成）──────────────────────
                # 螺旋/同心圓 + 圓心模式：使用 AdvancedScanGenerator（固定翼曲率約束）
                # 避免多邊形裁切把圓環切成不連續段，再直線相連形成八字路徑
                if scan_mode in ('spiral', 'circular') and self.circle_center is not None:
                    _alt = fw_params.mission_altitude_m
                    _max_r = self.flight_params.get('circle_max_radius', 200.0)
                    _mission_path, mission_latlon = self._generate_advanced_scan_mission(
                        scan_mode, center_lat, center_lon, _max_r,
                        fw_params, approach_bearing,
                    )
                    if _mission_path and fw_params.turn_radius_m > 0:
                        _mission_path = planner.smooth_path_with_arcs(
                            _mission_path, fw_params.turn_radius_m
                        )
                        mission_latlon = [(lat, lon) for lat, lon, _ in _mission_path]
                    result = {
                        'takeoff_path': takeoff_path,
                        'mission_path': _mission_path,
                        'mission_latlon': mission_latlon,
                        'landing_path': landing_path,
                        'full_path': takeoff_ll + mission_latlon + landing_ll,
                    }
                else:
                    if scan_mode == 'grid':
                        # 使用 CoveragePathPlanner（燈泡型轉彎）取代預設網格生成
                        result = self._generate_grid_with_coverage_planner(
                            mission_polygon, fw_params, scan_angle,
                            planner, takeoff_path, landing_path,
                            home_lat, home_lon,
                        )
                    else:
                        result = planner.generate_full_mission(
                            polygon=mission_polygon,
                            home_lat=home_lat,
                            home_lon=home_lon,
                            params=fw_params,
                            scan_pattern=scan_mode,
                            scan_angle_deg=scan_angle,
                        )

                # ── NFZ 路徑修正（繞行禁航區）───────────────────────────
                if self.nfz_zones:
                    _raw_mission_ll = result.get('mission_latlon', [])
                    if _raw_mission_ll:
                        _corrected_ll = self._apply_nfz_correction_latlon(
                            _raw_mission_ll, center_lat, center_lon
                        )
                        _alt_nfz = fw_params.mission_altitude_m
                        result['mission_latlon'] = _corrected_ll
                        result['mission_path'] = [
                            (lat, lon, _alt_nfz) for lat, lon in _corrected_ll
                        ]
                        result['full_path'] = (
                            takeoff_ll + _corrected_ll + landing_ll
                        )

                # ── 插入進場弧：起飛末端 → 任務第一點 ─────────────────
                _mission_ll = result.get('mission_latlon', [])
                _tkoff_path = result.get('takeoff_path', [])
                if _mission_ll and _tkoff_path:
                    _p_end = (_tkoff_path[-1][0], _tkoff_path[-1][1])
                    _p_first = _mission_ll[0]
                    _entry_arc = planner.generate_entry_arc(
                        _p_end,
                        fw_params.takeoff_bearing_deg,
                        _p_first,
                        fw_params.turn_radius_m,
                        fw_params.mission_altitude_m,
                        min_turn_deg=5.0,   # 低門檻，小角度也平滑銜接
                    )
                    if _entry_arc:
                        _arc_ll = [(_la, _lo) for _la, _lo, _ in _entry_arc]
                        result['mission_latlon'] = _arc_ll + _mission_ll
                        result['mission_path'] = (
                            _entry_arc + result.get('mission_path', [])
                        )
                        result['full_path'] = (
                            [(la, lo) for la, lo, _ in result['takeoff_path']]
                            + result['mission_latlon']
                            + [(la, lo) for la, lo, _ in result['landing_path']]
                        )
                        logger.info(
                            f"插入進場弧：{len(_entry_arc)} 個弧段點，"
                            f"起飛→任務轉向平滑化"
                        )

                # ── 插入離場弧：任務最後一點 → 對齊下風邊 → 五邊進場 ────
                # 幾何關係：
                #   飛機在下風邊飛行方向 = runway_hdg（downwind_start → downwind_end）
                #   ∴ lead_in 須在 downwind_start 的 opp_hdg 方向（飛行路徑的前方）
                #   弧段以 lead_in 為目標，飛機抵達後航向 ≈ runway_hdg，
                #   再直線飛入 downwind_start，完全消除急轉彎。
                _mission_ll_ex = result.get('mission_latlon', [])
                _landing_path  = result.get('landing_path', [])
                if len(_mission_ll_ex) >= 2 and _landing_path:
                    _p_last           = _mission_ll_ex[-1]
                    _p_downwind_start = (_landing_path[0][0], _landing_path[0][1])
                    _pat_alt          = fw_params.pattern_altitude_m

                    # 飛機進入下風邊的飛行方向 = runway_hdg
                    # lead_in 在 downwind_start 的 opp_hdg 方向（= 路徑上游）
                    _runway_hdg = fw_params.landing_bearing_deg
                    _opp_hdg    = (_runway_hdg + 180.0) % 360.0

                    _lead_dist = max(2.0 * fw_params.turn_radius_m, 150.0)
                    _dlat_m = 1.0 / 111_111.0
                    _dlon_m = 1.0 / (111_111.0 * math.cos(
                        math.radians(_p_downwind_start[0])) + 1e-12)
                    # lead_in = downwind_start + lead_dist * opp_hdg（上游方向）
                    _lead_lat = (_p_downwind_start[0]
                                 + _lead_dist * math.cos(math.radians(_opp_hdg)) * _dlat_m)
                    _lead_lon = (_p_downwind_start[1]
                                 + _lead_dist * math.sin(math.radians(_opp_hdg)) * _dlon_m)
                    _p_lead_in = (_lead_lat, _lead_lon)

                    # 最後一段航向（倒數第二點 → 最後一點）
                    _lx = (math.sin(math.radians(_p_last[1] - _mission_ll_ex[-2][1]))
                           * math.cos(math.radians(_p_last[0])))
                    _ly = (math.cos(math.radians(_mission_ll_ex[-2][0]))
                           * math.sin(math.radians(_p_last[0]))
                           - math.sin(math.radians(_mission_ll_ex[-2][0]))
                           * math.cos(math.radians(_p_last[0]))
                           * math.cos(math.radians(_p_last[1] - _mission_ll_ex[-2][1])))
                    _h_last = (math.degrees(math.atan2(_lx, _ly)) + 360) % 360

                    # 弧段：從任務末端 → lead_in（不管有無弧，lead_in 都插入）
                    _exit_arc = planner.generate_entry_arc(
                        _p_last,
                        _h_last,
                        _p_lead_in,
                        fw_params.turn_radius_m,
                        _pat_alt,
                        min_turn_deg=5.0,   # 低門檻，確保小角度也平滑
                    )
                    _exit_ll   = ([(_la, _lo) for _la, _lo, _ in _exit_arc]
                                  + [_p_lead_in])
                    _exit_path = (_exit_arc
                                  + [(_lead_lat, _lead_lon, _pat_alt)])
                    result['mission_latlon'] = _mission_ll_ex + _exit_ll
                    result['mission_path']   = (
                        result.get('mission_path', []) + _exit_path
                    )
                    result['full_path'] = (
                        [(la, lo) for la, lo, _ in result['takeoff_path']]
                        + result['mission_latlon']
                        + [(la, lo) for la, lo, _ in result['landing_path']]
                    )
                    logger.info(
                        f"插入離場弧：{len(_exit_arc)} 個弧段點 + lead_in，"
                        f"runway_hdg={_runway_hdg:.0f}°，opp={_opp_hdg:.0f}°→五邊進場"
                    )

                # ── 最終任務段弧段平滑：消除進/離場弧接入殘留銳角 ─────
                if result.get('mission_path') and fw_params.turn_radius_m > 0:
                    _mp_final = planner.smooth_path_with_arcs(
                        result['mission_path'], fw_params.turn_radius_m
                    )
                    result['mission_path']   = _mp_final
                    result['mission_latlon'] = [(la, lo) for la, lo, _ in _mp_final]
                    result['full_path'] = (
                        [(la, lo) for la, lo, _ in result['takeoff_path']]
                        + result['mission_latlon']
                        + [(la, lo) for la, lo, _ in result['landing_path']]
                    )

                # ── 任務末端 → 五邊進場橋接 + lead_in 轉角弧平滑 ────────────────────
                # 問題：離場弧使飛機到達 lead_in 時的航向 = bearing(p_last→lead_in)，
                #        但 lead_in→downwind_start 的航向 = runway_hdg，
                #        兩者不同 → lead_in 處出現銳角。
                # 修正：
                #   1. 以 [arc_exit, lead_in, downwind_start] 做 3 點弧平滑，
                #      將 lead_in 轉角替換為內切圓弧，弧出口與 runway_hdg 對齊。
                #   2. 弧尾接上 downwind_start，消除藍橙路徑視覺斷線。
                _mp_tail = result.get('mission_path', [])
                _lp_head = result.get('landing_path', [])
                if len(_mp_tail) >= 2 and len(_lp_head) >= 1:
                    _bridge_pt3d  = _lp_head[0]          # downwind_start
                    _bridge_ll    = (_bridge_pt3d[0], _bridge_pt3d[1])

                    # 只對末端 3 點做弧平滑，避免干擾已平滑的前段路徑
                    _tail3        = [_mp_tail[-2], _mp_tail[-1], _bridge_pt3d]
                    _tail3_smooth = planner.smooth_path_with_arcs(
                        _tail3, fw_params.turn_radius_m
                    )
                    # _tail3_smooth[0] = arc_exit（已在 _mp_tail[:-1] 末端，跳過避免重複）
                    # _tail3_smooth[1:] = [t_in, arc_pts..., t_out, downwind_start]
                    _extended = _mp_tail[:-1] + _tail3_smooth[1:]

                    result['mission_path']   = _extended
                    result['mission_latlon'] = [(la, lo) for la, lo, _ in _extended]
                    result['full_path'] = (
                        [(la, lo) for la, lo, _ in result['takeoff_path']]
                        + result['mission_latlon']
                        + landing_ll
                    )
                    _arc_n = max(0, len(_tail3_smooth) - 3)
                    logger.info(
                        f"任務→五邊接縫弧：lead_in 轉角平滑，"
                        f"插入 {_arc_n} 個弧段點，"
                        f"downwind_start ({_bridge_ll[0]:.6f}, {_bridge_ll[1]:.6f})"
                    )

                self.fw_mission_result = result
                self.sub_paths = []
                path = result['full_path']
                path, _trimmed, _orig_n = self._trim_to_wp_limit(path, "固定翼單區任務")
                self.waypoints = path

                # 確保 MAVLink 匯出的任務航點也在 _WP_LIMIT 以內
                # 預算 = 限制 - 起飛點數 - 降落點數 - 固定指令頭(1)
                _tkoff_n = len(result.get('takeoff_path', []))
                _land_n  = len(result.get('landing_path', []))
                _mission_budget = max(10, _WP_LIMIT - _tkoff_n - _land_n - 1)
                _mp = result.get('mission_path', [])
                if len(_mp) > _mission_budget:
                    _mp_ll  = [(la, lo) for la, lo, _ in _mp]
                    _mp_alt = [al for _, _, al in _mp]
                    _mp_ll_trim, _mp_alt_trim = CoveragePlanner.limit_path_waypoints(
                        _mp_ll, _mission_budget, _mp_alt
                    )
                    if not _mp_alt_trim:
                        _mp_alt_trim = [(_mp[0][2] if _mp else 100.0)] * len(_mp_ll_trim)
                    result['mission_path']   = [(la, lo, al) for (la, lo), al
                                                in zip(_mp_ll_trim, _mp_alt_trim)]
                    result['mission_latlon'] = _mp_ll_trim
                    _trimmed = True

                mission_ll = result['mission_latlon']
                self.map_widget.display_fw_paths(takeoff_ll, mission_ll, landing_ll)
                if self.nfz_zones:
                    self.map_widget.display_nfz_zones(self.nfz_zones)

                stats = FixedWingPlanner.estimate_mission_stats(
                    result, self.flight_params.get('speed', 18.0)
                )

                _wp_txt = f"航點: {stats['waypoint_count']}"
                if _trimmed:
                    _wp_txt = f"航點: {len(path)} (原{_orig_n},已簡化)"
                self.waypoint_label.setText(_wp_txt)
                total_dist = stats['total_dist_m']
                self.distance_label.setText(f"距離: {total_dist:.0f}m")
                self.mission_panel.update_mission_stats({
                    'waypoint_count': stats['waypoint_count'],
                    'total_distance': total_dist,
                    'estimated_time': stats['estimated_time_s'],
                    'area': CoveragePlanner().calculate_coverage_area(mission_polygon),
                    'regions': 1,
                })

                QMessageBox.information(
                    self, "固定翼任務生成完成",
                    f"三階段任務路徑已生成！\n\n"
                    f"[起飛階段]\n"
                    f"  方向: {fw_params.takeoff_bearing_deg:.0f}°  "
                    f"跑道: {fw_params.takeoff_runway_m:.0f}m\n"
                    f"  起飛距離: {stats['takeoff_dist_m']:.0f}m\n\n"
                    f"[主任務 ({scan_mode.upper()} 掃描)]\n"
                    f"  航點數: {len(result['mission_path'])} 個\n"
                    f"  掃描距離: {stats['mission_dist_m']:.0f}m\n\n"
                    f"[五邊進場降落]\n"
                    f"  進場方向: {fw_params.landing_bearing_deg:.0f}°  "
                    f"五邊高度: {fw_params.pattern_altitude_m:.0f}m\n"
                    f"  降落距離: {stats['landing_dist_m']:.0f}m\n\n"
                    f"總飛行距離: {total_dist:.0f}m\n"
                    f"預估時間: {stats['estimated_time_s'] / 60:.1f} min\n\n"
                    f"[轉彎半徑檢測]\n"
                    f"  半徑: {fw_params.turn_radius_m:.0f}m  "
                    f"所需傾斜: {check['required_bank']:.1f}°\n"
                    f"  狀態: {turn_status}"
                )
                logger.info(
                    f"固定翼任務生成: {stats['waypoint_count']} 個航點, "
                    f"總距離 {total_dist:.0f}m, 轉彎半徑={fw_params.turn_radius_m:.0f}m"
                )
                return path

        except Exception as e:
            logger.error(f"固定翼任務生成失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "固定翼任務錯誤", f"生成路徑時發生錯誤：\n{str(e)}")
            return None

    # ══════════════════════════════════════════════════════════════════
    # 整合 3：FixedWingNFZPlanner — 禁航區管理與路徑修正
    # ══════════════════════════════════════════════════════════════════
    def open_nfz_dialog(self):
        """開啟禁航區管理對話框"""
        from ui.dialogs.nfz_manager_dialog import NFZManagerDialog
        dialog = NFZManagerDialog(self, self.nfz_zones)
        dialog.nfz_changed.connect(self.on_nfz_changed)
        dialog.exec()

    def on_nfz_changed(self, nfz_zones: list):
        """更新禁航區清單，更新地圖顯示，並重新生成路徑"""
        self.nfz_zones = nfz_zones
        count = len(nfz_zones)
        logger.info(f"禁航區更新：{count} 個")
        # 更新 parameter_panel 計數顯示
        if hasattr(self.parameter_panel, 'nfz_count_label'):
            self.parameter_panel.nfz_count_label.setText(f"目前禁航區：{count} 個")
        # 更新地圖上的 NFZ 視覺化
        self.map_widget.display_nfz_zones(nfz_zones)
        # 重新生成路徑（若已有路徑）
        if self.waypoints:
            self.on_preview_paths()

    def _apply_nfz_correction_latlon(
        self,
        path_latlon: list,
        ref_lat: float,
        ref_lon: float,
    ) -> list:
        """
        對 lat/lon 路徑套用禁航區繞行修正。

        內部使用公尺座標計算，完成後轉回 lat/lon 返回。

        Args:
            path_latlon: [(lat, lon), ...] 原始路徑
            ref_lat, ref_lon: 座標轉換參考原點

        Returns:
            修正後的 [(lat, lon), ...]（若無 NFZ 或無交叉則原樣返回）
        """
        if not self.nfz_zones or not path_latlon:
            return path_latlon

        from nfz_planner import FixedWingNFZPlanner
        from utils.math_utils import latlon_to_meters, meters_to_latlon

        fw_params = self._build_fw_params()
        constraints = self._build_fw_constraints(fw_params)
        nfz_planner = FixedWingNFZPlanner(constraints, buffer_factor=1.5)

        # 加入所有禁航區（轉為公尺座標）
        for zone in self.nfz_zones:
            if zone['type'] == 'polygon':
                nfz_planner.add_polygon_nfz(
                    zone['vertices'], zone['name'],
                    coord_type='latlon',
                    ref_latlon=(ref_lat, ref_lon),
                )
            elif zone['type'] == 'circle':
                clat, clon = zone['center']
                cx, cy = latlon_to_meters(clat, clon, ref_lat, ref_lon)
                nfz_planner.add_circle_nfz(
                    (cx, cy), zone['radius'], zone['name']
                )

        # 將路徑轉為公尺座標
        metric_path = [
            latlon_to_meters(lat, lon, ref_lat, ref_lon)
            for lat, lon in path_latlon
        ]

        correction = nfz_planner.correct_path(metric_path)

        if correction.is_modified:
            logger.info(
                f"NFZ 路徑修正：{correction.segments_rerouted} 段繞行，"
                f"增加 {correction.added_length:.0f}m，"
                f"航點 {len(path_latlon)} → {len(correction.corrected_path)}"
            )
        else:
            logger.info("NFZ 檢查：路徑無需修正")

        return [
            meters_to_latlon(x, y, ref_lat, ref_lon)
            for x, y in correction.corrected_path
        ]

    # ══════════════════════════════════════════════════════════════════
    # 輔助：固定翼約束建立
    # ══════════════════════════════════════════════════════════════════
    def _build_fw_constraints(self, fw_params: 'FixedWingParameters'):
        """
        從 FixedWingParameters 建立 FixedWingConstraints。

        以使用者設定的 turn_radius_m 反推等效傾斜角：
            R = V²/(g·tan φ)  →  φ = atan(V²/(g·R))
        safety_factor=1.0（不額外放大），使 get_min_turn_radius() 直接返回 turn_radius_m，
        確保 AdvancedScanGenerator / CoveragePathPlanner 的轉彎幾何與 UI 設定一致。
        """
        from core.base.fixed_wing_constraints import FixedWingConstraints
        import math as _math
        speed = fw_params.cruise_speed_mps
        stall = min(12.0, speed * 0.6)
        r = max(fw_params.turn_radius_m, 10.0)
        # tan φ = V²/(g·R)
        tan_phi = speed ** 2 / (9.81 * r)
        bank_deg = _math.degrees(_math.atan(tan_phi))
        bank_deg = max(5.0, min(bank_deg, 80.0))  # 安全夾限
        return FixedWingConstraints(
            cruise_airspeed_mps=max(speed, stall + 1.0),
            max_bank_angle_deg=bank_deg,
            stall_speed_mps=stall,
            safety_factor=1.0,  # 不放大：R_min 直接等於 turn_radius_m
        )

    def _poses_to_latlon(self, poses, ref_lat: float, ref_lon: float):
        """將 List[Pose3D]（公尺座標）轉回 [(lat, lon), ...]"""
        from utils.math_utils import meters_to_latlon
        return [meters_to_latlon(p.x, p.y, ref_lat, ref_lon) for p in poses]

    # ══════════════════════════════════════════════════════════════════
    # 整合 1：AdvancedScanGenerator — 螺旋 / 同心圓掃描
    # ══════════════════════════════════════════════════════════════════
    def _generate_advanced_scan_mission(
        self,
        scan_mode: str,
        center_lat: float,
        center_lon: float,
        max_radius: float,
        fw_params: 'FixedWingParameters',
        approach_bearing: float,
    ):
        """
        使用 AdvancedScanGenerator 生成螺旋或同心圓掃描路徑。

        Args:
            scan_mode: 'spiral' 或 'circular'
            center_lat/lon: 掃描圓心（地理座標）
            max_radius: 最大半徑 [m]
            fw_params: 固定翼飛行參數
            approach_bearing: 進場航向 [度]，用於螺旋起始角

        Returns:
            (_mission_path, mission_latlon)
            _mission_path: [(lat, lon, alt), ...]
            mission_latlon: [(lat, lon), ...]
        """
        from advanced_scan_patterns import (
            AdvancedScanGenerator, SpiralParams, ConcentricParams
        )
        from core.trajectory.dubins_trajectory import DubinsTrajectoryGenerator

        constraints = self._build_fw_constraints(fw_params)
        dubins_gen = DubinsTrajectoryGenerator(constraints)
        adv_gen = AdvancedScanGenerator(
            constraints=constraints,
            dubins_gen=dubins_gen,
            altitude_m=fw_params.mission_altitude_m,
        )
        _alt = fw_params.mission_altitude_m
        spacing = fw_params.scan_spacing_m

        if scan_mode == 'spiral':
            direction = fw_params.circle_direction  # 1=CCW, -1=CW
            params = SpiralParams(
                center=(0.0, 0.0),
                max_radius=max_radius,
                spacing=spacing,
                direction=direction,
                start_angle_deg=approach_bearing,
            )
            adv_result = adv_gen.generate_spiral(params, inward=True)
        else:  # circular (同心圓)
            params = ConcentricParams(
                center=(0.0, 0.0),
                max_radius=max_radius,
                spacing=spacing,
                arc_step_deg=5.0,
            )
            adv_result = adv_gen.generate_concentric(params)

        if adv_result.warnings:
            for w in adv_result.warnings:
                logger.warning(f"AdvancedScan: {w}")

        if not adv_result.poses:
            # 退回舊方法
            logger.warning("AdvancedScanGenerator 無法生成路徑，退回 CoveragePlanner")
            _cov = CoveragePlanner()
            if scan_mode == 'spiral':
                _sp_curv = self.flight_params.get('spiral_curvature', 1.0)
                _sp_alt_step = self.flight_params.get('spiral_alt_step', 0.0)
                mission_latlon = _cov.plan_spiral_from_center(
                    center_lat, center_lon, max_radius,
                    spacing, is_fixed_wing=True,
                    turn_radius=fw_params.turn_radius_m,
                    curvature=_sp_curv, alt_step=_sp_alt_step,
                    base_alt=_alt, entry_bearing_deg=approach_bearing,
                )
            else:
                _min_r = self.flight_params.get('circle_min_radius', 0.0)
                mission_latlon = _cov.plan_circle_survey_from_center(
                    center_lat, center_lon, max_radius, spacing,
                    base_alt=_alt, direction=fw_params.circle_direction,
                    min_radius_m=_min_r, start_angle_deg=approach_bearing,
                )
            _mission_path = [(lat, lon, _alt) for lat, lon in mission_latlon]
            return _mission_path, mission_latlon

        mission_latlon = self._poses_to_latlon(adv_result.poses, center_lat, center_lon)
        _mission_path = [(lat, lon, _alt) for lat, lon in mission_latlon]
        logger.info(
            f"AdvancedScan ({scan_mode}): {len(mission_latlon)} 航點, "
            f"總長={adv_result.total_length:.0f}m, "
            f"可行={'是' if adv_result.is_feasible else '否'}"
        )
        return _mission_path, mission_latlon

    # ══════════════════════════════════════════════════════════════════
    # 整合 2：CoveragePathPlanner — 固定翼網格掃描（燈泡型轉彎）
    # ══════════════════════════════════════════════════════════════════
    def _generate_grid_with_coverage_planner(
        self,
        mission_polygon,
        fw_params: 'FixedWingParameters',
        scan_angle: float,
        planner: 'FixedWingPlanner',
        takeoff_path,
        landing_path,
        home_lat: float,
        home_lon: float,
    ) -> dict:
        """
        使用 CoveragePathPlanner（Bulb Turn 燈泡型轉彎）生成固定翼網格掃描路徑。
        取代原本 generate_full_mission() 中的網格覆蓋邏輯。

        Returns:
            與 generate_full_mission() 相容的 result dict
        """
        from core.global_planner.coverage_path_planner import CoveragePathPlanner
        from core.trajectory.dubins_trajectory import DubinsTrajectoryGenerator
        from utils.math_utils import latlon_to_meters, meters_to_latlon

        constraints = self._build_fw_constraints(fw_params)
        dubins_gen = DubinsTrajectoryGenerator(constraints)
        cov_planner = CoveragePathPlanner(
            constraints=constraints,
            dubins_gen=dubins_gen,
            altitude_m=fw_params.mission_altitude_m,
            airspeed_mps=fw_params.cruise_speed_mps,
        )

        # 以第一個角點為參考原點轉換到公尺座標
        ref_lat = sum(p[0] for p in mission_polygon) / len(mission_polygon)
        ref_lon = sum(p[1] for p in mission_polygon) / len(mission_polygon)
        metric_poly = [
            latlon_to_meters(lat, lon, ref_lat, ref_lon)
            for lat, lon in mission_polygon
        ]

        cov_result = cov_planner.plan_coverage(
            polygon=metric_poly,
            scan_angle_deg=scan_angle,
            spacing_m=fw_params.scan_spacing_m,
            coord_type="metric",
        )

        if cov_result.warnings:
            for w in cov_result.warnings:
                logger.warning(f"CoveragePathPlanner: {w}")

        _alt = fw_params.mission_altitude_m
        takeoff_ll = [(lat, lon) for lat, lon, _ in takeoff_path]
        landing_ll = [(lat, lon) for lat, lon, _ in landing_path]

        if not cov_result.poses_3d:
            # 退回原方法
            logger.warning("CoveragePathPlanner 無可用路徑，退回 generate_full_mission")
            return planner.generate_full_mission(
                polygon=mission_polygon,
                home_lat=home_lat,
                home_lon=home_lon,
                params=fw_params,
                scan_pattern='grid',
                scan_angle_deg=scan_angle,
            )

        mission_latlon = self._poses_to_latlon(cov_result.poses_3d, ref_lat, ref_lon)
        _mission_path = [(lat, lon, _alt) for lat, lon in mission_latlon]

        logger.info(
            f"CoveragePathPlanner: {cov_result.num_scan_lines} 掃描線, "
            f"{len(mission_latlon)} 航點, 總長={cov_result.total_length:.0f}m"
        )

        return {
            'takeoff_path': takeoff_path,
            'mission_path': _mission_path,
            'mission_latlon': mission_latlon,
            'landing_path': landing_path,
            'full_path': takeoff_ll + mission_latlon + landing_ll,
        }

    def on_export_waypoints(self):
        """匯出航點"""
        if not self.waypoints:
            QMessageBox.warning(self, "無資料", "請先設定邊界點並預覽路徑生成航點")
            return

        # ── 固定翼三階段任務匯出 ──────────────────────────────────────
        if self.current_vehicle_type == '固定翼' and self.fw_mission_result:
            self._export_fixed_wing_waypoints()
            return

        # ── 多台無人機分區匯出 ──────────────────────────────────────────
        if self.sub_paths and len(self.sub_paths) > 1:
            self._export_multi_uav_waypoints()
            return

        # ── 單台無人機：以高度為預設檔名 ────────────────────────────────
        altitude = self.flight_params['altitude']
        default_name = f"高度{altitude:.0f}m"

        # 開啟匯出對話框
        filepath, selected_filter = QFileDialog.getSaveFileName(
            self, "儲存航點檔案",
            default_name,
            "Mission Planner (*.waypoints);;QGC Waypoint Files (*.waypoints);;CSV Files (*.csv);;All Files (*)"
        )

        if not filepath:
            return

        try:
            speed    = self.flight_params['speed']
            yaw_spd  = self.flight_params.get('yaw_speed', 60.0)

            # ── CSV ──────────────────────────────────────────────────────────
            if selected_filter == "CSV Files (*.csv)" or filepath.endswith('.csv'):
                if not filepath.endswith('.csv'):
                    filepath += '.csv'
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write("sequence,latitude,longitude,altitude,speed\n")
                    for i, (lat, lon) in enumerate(self.waypoints):
                        _wp_alt = (self.spiral_waypoint_altitudes[i]
                                   if self.spiral_waypoint_altitudes and i < len(self.spiral_waypoint_altitudes)
                                   else altitude)
                        f.write(f"{i},{lat:.8f},{lon:.8f},{_wp_alt:.1f},{speed:.1f}\n")
                format_name = "CSV"

            # ── Mission Planner ──────────────────────────────────────────────
            elif selected_filter == "Mission Planner (*.waypoints)":
                if not filepath.endswith('.waypoints'):
                    filepath += '.waypoints'

                waypoint_lines = ['QGC WPL 110']
                seq = 0

                # seq 0: DO_SET_HOME（以目前位置為 Home）
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=179,  # MAV_CMD_DO_SET_HOME
                    lat=0.0, lon=0.0, alt=0.0,
                    current=0, autocontinue=1
                ))
                seq += 1

                # seq 1: DO_CHANGE_SPEED（設定巡航速度）
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=178,  # MAV_CMD_DO_CHANGE_SPEED
                    param1=0.0,    # 0=Airspeed（旋翼機等效地速）
                    param2=speed,  # 目標速度 (m/s)
                    param3=0.0,
                    current=0, autocontinue=1
                ))
                seq += 1

                # seq 2+: 航點 + CONDITION_YAW 交錯
                lock_heading  = self.flight_params.get('lock_heading', False)
                fixed_heading = self.flight_params.get('heading_angle', 0.0)

                for i, (lat, lon) in enumerate(self.waypoints):
                    _wp_alt = (self.spiral_waypoint_altitudes[i]
                               if self.spiral_waypoint_altitudes and i < len(self.spiral_waypoint_altitudes)
                               else altitude)
                    waypoint_lines.append(create_waypoint_line(
                        seq=seq, command=16,  # MAV_CMD_NAV_WAYPOINT
                        lat=lat, lon=lon, alt=_wp_alt,
                        param1=0.0,  # hold time
                        param2=2.0,  # acceptance radius
                        current=0, autocontinue=1
                    ))
                    seq += 1

                    # 鎖定 Heading：每個航點都插入固定方向的 CONDITION_YAW
                    # 自動 Heading：只在相鄰航點之間插入，計算到下一點的方位角
                    if lock_heading:
                        heading = fixed_heading
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=115,  # MAV_CMD_CONDITION_YAW
                            param1=round(heading, 1),
                            param2=yaw_spd,
                            param3=0.0,  # 0=絕對角度
                            param4=0.0,  # 0=順時針
                            current=0, autocontinue=1
                        ))
                        seq += 1
                    elif i < len(self.waypoints) - 1:
                        next_lat, next_lon = self.waypoints[i + 1]
                        heading = self._calc_bearing(lat, lon, next_lat, next_lon)
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=115,  # MAV_CMD_CONDITION_YAW
                            param1=round(heading, 1),
                            param2=yaw_spd,
                            param3=0.0,  # 0=絕對角度
                            param4=0.0,  # 0=順時針
                            current=0, autocontinue=1
                        ))
                        seq += 1

                # 返航（RTL）
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=20,  # MAV_CMD_NAV_RETURN_TO_LAUNCH
                    current=0, autocontinue=1
                ))

                write_waypoints(filepath, waypoint_lines)
                format_name = "Mission Planner"

            # ── QGC WPL 110 ──────────────────────────────────────────────────
            else:
                if not filepath.endswith('.waypoints'):
                    filepath += '.waypoints'

                home_lat, home_lon = self.waypoints[0]
                waypoint_lines = ['QGC WPL 110']

                # HOME 點
                waypoint_lines.append(create_waypoint_line(
                    seq=0, command=16,
                    lat=home_lat, lon=home_lon, alt=0.0,
                    current=1, autocontinue=1
                ))
                # 起飛點
                waypoint_lines.append(create_waypoint_line(
                    seq=1, command=22,  # MAV_CMD_NAV_TAKEOFF
                    lat=home_lat, lon=home_lon, alt=altitude,
                    param1=15.0,
                    current=0, autocontinue=1
                ))
                # 航點
                for i, (lat, lon) in enumerate(self.waypoints):
                    _wp_alt = (self.spiral_waypoint_altitudes[i]
                               if self.spiral_waypoint_altitudes and i < len(self.spiral_waypoint_altitudes)
                               else altitude)
                    waypoint_lines.append(create_waypoint_line(
                        seq=2 + i, command=16,
                        lat=lat, lon=lon, alt=_wp_alt,
                        param1=0.0, param2=2.0,
                        current=0, autocontinue=1
                    ))
                # RTL
                waypoint_lines.append(create_waypoint_line(
                    seq=2 + len(self.waypoints), command=20,
                    current=0, autocontinue=1
                ))

                write_waypoints(filepath, waypoint_lines)
                format_name = "QGC"

            QMessageBox.information(
                self, "匯出成功",
                f"航點檔案已匯出！\n\n"
                f"格式：{format_name}\n"
                f"檔案：{filepath}\n"
                f"航點數：{len(self.waypoints)}\n"
                f"高度：{altitude} m\n"
                f"速度：{speed} m/s"
            )
            self.statusBar().showMessage(f"已匯出 {len(self.waypoints)} 個航點至 {filepath}", 5000)
            logger.info(f"匯出航點 ({format_name}): {filepath}")

        except Exception as e:
            logger.error(f"匯出失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "匯出錯誤", f"匯出時發生錯誤：\n{str(e)}")

    def _export_fixed_wing_waypoints(self):
        """匯出固定翼三階段任務 .waypoints 檔（Mission Planner 格式）"""
        altitude = self.flight_params['altitude']
        default_name = f"固定翼任務_{altitude:.0f}m"

        filepath, _ = QFileDialog.getSaveFileName(
            self, "儲存固定翼任務檔案",
            default_name,
            "Mission Planner (*.waypoints);;All Files (*)"
        )
        if not filepath:
            return
        if not filepath.endswith('.waypoints'):
            filepath += '.waypoints'

        try:
            fw_params = self._build_fw_params()
            planner = FixedWingPlanner()
            lines = planner.generate_mavlink_waypoints(
                mission_result=self.fw_mission_result,
                params=fw_params,
                speed_mps=self.flight_params.get('speed', 18.0),
            )

            from utils.file_io import write_waypoints
            write_waypoints(filepath, lines)

            stats = FixedWingPlanner.estimate_mission_stats(
                self.fw_mission_result, self.flight_params.get('speed', 18.0)
            )

            QMessageBox.information(
                self, "固定翼任務匯出成功",
                f"已匯出固定翼三階段任務！\n\n"
                f"檔案：{filepath}\n"
                f"總航點：{stats['waypoint_count']} 個\n"
                f"總距離：{stats['total_dist_m']:.0f} m\n"
                f"預估時間：{stats['estimated_time_s'] / 60:.1f} min\n\n"
                f"包含：起飛路徑 + {self.current_algorithm.upper()} 掃描 + 五邊進場降落"
            )
            self.statusBar().showMessage(f"已匯出固定翼任務至 {filepath}", 5000)
            logger.info(f"固定翼任務匯出: {filepath}")

        except Exception as e:
            logger.error(f"固定翼匯出失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "匯出錯誤", f"匯出時發生錯誤：\n{str(e)}")

    def _export_multi_uav_waypoints(self):
        """多台無人機分區匯出：每個子區域生成一個獨立 .waypoints 檔案"""
        import os

        output_dir = QFileDialog.getExistingDirectory(
            self, "選擇輸出資料夾", "",
            QFileDialog.Option.ShowDirsOnly
        )
        if not output_dir:
            return

        altitude        = self.flight_params['altitude']
        speed           = self.flight_params['speed']
        yaw_spd         = self.flight_params.get('yaw_speed', 60.0)
        stagger_enabled = self.flight_params.get('transit_stagger_enabled', False)
        stagger_alt     = self.flight_params.get('transit_stagger_alt', 10.0)
        lock_heading    = self.flight_params.get('lock_heading', False)
        fixed_heading   = self.flight_params.get('heading_angle', 0.0)

        exported_files = []
        try:
            for i, sub_path in enumerate(self.sub_paths):
                if not sub_path:
                    continue

                # 此台無人機的實際飛行高度
                uav_alt = altitude + i * stagger_alt if stagger_enabled else altitude

                # 自動命名
                if stagger_enabled:
                    filename = f"區域{i+1}_高度{uav_alt:.0f}m.waypoints"
                else:
                    filename = f"區域{i+1}_高度{altitude:.0f}m.waypoints"
                filepath = os.path.join(output_dir, filename)

                # 生成 Mission Planner 格式航點
                waypoint_lines = ['QGC WPL 110']
                seq = 0

                # HOME
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=179,
                    lat=0.0, lon=0.0, alt=0.0,
                    current=0, autocontinue=1
                ))
                seq += 1

                # DO_CHANGE_SPEED
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=178,
                    param1=0.0, param2=speed, param3=0.0,
                    current=0, autocontinue=1
                ))
                seq += 1

                # 航點 + CONDITION_YAW
                for j, (lat, lon) in enumerate(sub_path):
                    waypoint_lines.append(create_waypoint_line(
                        seq=seq, command=16,
                        lat=lat, lon=lon, alt=uav_alt,
                        param1=0.0, param2=2.0,
                        current=0, autocontinue=1
                    ))
                    seq += 1

                    if lock_heading:
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=115,
                            param1=round(fixed_heading, 1),
                            param2=yaw_spd, param3=0.0, param4=0.0,
                            current=0, autocontinue=1
                        ))
                        seq += 1
                    elif j < len(sub_path) - 1:
                        next_lat, next_lon = sub_path[j + 1]
                        heading = self._calc_bearing(lat, lon, next_lat, next_lon)
                        waypoint_lines.append(create_waypoint_line(
                            seq=seq, command=115,
                            param1=round(heading, 1),
                            param2=yaw_spd, param3=0.0, param4=0.0,
                            current=0, autocontinue=1
                        ))
                        seq += 1

                # RTL
                waypoint_lines.append(create_waypoint_line(
                    seq=seq, command=20,
                    current=0, autocontinue=1
                ))

                write_waypoints(filepath, waypoint_lines)
                exported_files.append(filename)
                logger.info(f"匯出子區域 {i+1}: {filepath}")

            QMessageBox.information(
                self, "匯出成功",
                f"已匯出 {len(exported_files)} 個航點檔案至：\n{output_dir}\n\n"
                + "\n".join(exported_files)
            )
            self.statusBar().showMessage(
                f"已匯出 {len(exported_files)} 個子區域航點至 {output_dir}", 5000
            )
            logger.info(f"多台無人機匯出完成: {len(exported_files)} 個檔案")

        except Exception as e:
            logger.error(f"多台無人機匯出失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "匯出錯誤", f"匯出時發生錯誤：\n{str(e)}")

    def on_new_mission(self):
        """創建新任務"""
        # 如果有未儲存的變更，詢問是否儲存
        if self.current_mission and self.has_unsaved_changes():
            reply = QMessageBox.question(
                self, "未儲存的變更",
                "當前任務有未儲存的變更，是否儲存？",
                QMessageBox.StandardButton.Yes | 
                QMessageBox.StandardButton.No | 
                QMessageBox.StandardButton.Cancel
            )
            
            if reply == QMessageBox.StandardButton.Yes:
                self.on_save_mission()
            elif reply == QMessageBox.StandardButton.Cancel:
                return
        
        # 清除當前任務
        self.on_clear_all_silent()
        
        # 創建新任務
        self.current_mission = self.mission_manager.create_mission("新任務")
        
        self.statusBar().showMessage("已創建新任務", 3000)
        logger.info("創建新任務")
    
    def on_open_mission(self):
        """開啟任務"""
        filepath, _ = QFileDialog.getOpenFileName(
            self, "開啟任務檔案",
            "",
            "Mission Files (*.json);;All Files (*)"
        )
        
        if filepath:
            try:
                mission = self.mission_manager.load_mission(filepath)
                self.current_mission = mission
                
                # TODO: 載入任務參數到 UI
                
                self.statusBar().showMessage(f"已載入任務：{mission.name}", 3000)
                logger.info(f"載入任務: {filepath}")
                
            except Exception as e:
                logger.error(f"載入任務失敗: {e}")
                QMessageBox.critical(self, "載入錯誤", f"載入任務時發生錯誤：\n{str(e)}")
    
    def on_save_mission(self):
        """儲存任務"""
        if not self.current_mission:
            QMessageBox.warning(self, "無任務", "沒有任務可儲存")
            return
        
        try:
            filepath = self.mission_manager.save_mission(self.current_mission)
            
            if filepath:
                self.statusBar().showMessage(f"任務已儲存", 3000)
                logger.info(f"儲存任務: {filepath}")
            else:
                QMessageBox.warning(self, "儲存失敗", "無法儲存任務")
                
        except Exception as e:
            logger.error(f"儲存任務失敗: {e}")
            QMessageBox.critical(self, "儲存錯誤", f"儲存時發生錯誤：\n{str(e)}")
    
    def on_clear_paths(self):
        """清除路徑"""
        self.map_widget.clear_paths()
        self.waypoints.clear()
        self.spiral_waypoint_altitudes.clear()
        self.waypoint_label.setText("航點: 0")
        self.distance_label.setText("距離: 0.0m")
        logger.info("已清除路徑")
    
    def on_clear_corners(self):
        """清除邊界"""
        self.map_widget.clear_corners()
        self.corners.clear()
        self.parameter_panel.update_corner_count(0)
        # 同時清除圓心設定
        self.circle_center = None
        self.picking_circle_center = False
        if hasattr(self, 'parameter_panel') and hasattr(self.parameter_panel, 'circle_center_label'):
            self.parameter_panel.circle_center_label.setText("圓心：尚未設定")
            self.parameter_panel.circle_center_label.setStyleSheet(
                "color: #888; font-size: 11px; font-style: italic;"
            )
        if hasattr(self.map_widget, 'clear_circle_overlay'):
            self.map_widget.clear_circle_overlay()
        # 同時清除起飛點設定
        self.home_point = None
        self.picking_home_point = False
        if hasattr(self, 'parameter_panel'):
            self.parameter_panel.clear_home_point_display()
        if hasattr(self.map_widget, 'clear_home_point_overlay'):
            self.map_widget.clear_home_point_overlay()
        logger.info("已清除邊界")
    
    def on_clear_all(self):
        """清除全部（帶確認）"""
        reply = QMessageBox.question(
            self, "確認清除",
            "確定要清除所有標記和路徑嗎？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            self.on_clear_all_silent()
    
    def on_clear_all_silent(self):
        """清除全部（不帶確認）"""
        self.on_clear_corners()
        self.on_clear_paths()
        self.obstacles.clear()
        logger.info("已清除全部")
    
    def on_reset_view(self):
        """重置視圖"""
        self.map_widget.reset_view()
    
    def on_toggle_grid(self):
        """切換網格顯示"""
        # TODO: 實現網格顯示切換
        QMessageBox.information(self, "網格顯示", "網格顯示功能開發中")
    
    def on_camera_config(self):
        """相機配置"""
        try:
            from ui.dialogs.camera_config import CameraConfigDialog
            dialog = CameraConfigDialog(self)
            dialog.exec()
        except ImportError:
            QMessageBox.information(self, "相機配置", "相機配置功能開發中")
    
    def on_vehicle_config(self):
        """飛行器配置"""
        try:
            from ui.dialogs.vehicle_config import VehicleConfigDialog
            dialog = VehicleConfigDialog(self)
            dialog.exec()
        except ImportError:
            QMessageBox.information(self, "飛行器配置", "飛行器配置功能開發中")
    
    def on_obstacle_manager(self):
        """障礙物管理"""
        try:
            from ui.dialogs.obstacle_manager import ObstacleManagerDialog

            dialog = ObstacleManagerDialog(self, self.obstacles)
            dialog.obstacles_changed.connect(self.on_obstacles_changed)
            dialog.exec()

        except ImportError as e:
            logger.error(f"無法載入障礙物管理對話框: {e}")
            QMessageBox.warning(self, "載入失敗", "無法載入障礙物管理功能")

    def on_open_polygon_editor(self):
        """開啟多邊形編輯器"""
        try:
            from ui.widgets.polygon_editor import PolygonEditorWindow

            # 創建編輯器視窗
            self.polygon_editor_window = PolygonEditorWindow(max_corners=MAX_CORNERS)

            # 如果已有角點，載入到編輯器
            if self.corners:
                self.polygon_editor_window.editor.set_corners(self.corners)

            # 連接信號 - 當編輯完成時同步角點
            self.polygon_editor_window.polygon_completed.connect(self._on_polygon_editor_completed)
            self.polygon_editor_window.editor.corners_changed.connect(self._on_polygon_editor_corners_changed)

            self.polygon_editor_window.show()
            logger.info("已開啟多邊形編輯器")

        except Exception as e:
            logger.error(f"開啟多邊形編輯器失敗: {e}")
            import traceback
            traceback.print_exc()
            QMessageBox.critical(self, "錯誤", f"無法開啟多邊形編輯器：\n{str(e)}")

    def _on_polygon_editor_completed(self, corners):
        """多邊形編輯器完成編輯"""
        self._sync_corners_from_editor(corners)
        QMessageBox.information(
            self, "角點已同步",
            f"已從編輯器同步 {len(corners)} 個角點到主視窗"
        )

    def _on_polygon_editor_corners_changed(self, corners):
        """多邊形編輯器角點變更（即時同步）"""
        self._sync_corners_from_editor(corners)

    def _sync_corners_from_editor(self, corners):
        """從編輯器同步角點"""
        # 清除現有角點
        self.corners.clear()
        self.map_widget.corners.clear()
        self.map_widget.markers.clear()

        # 添加新角點
        for lat, lon in corners:
            self.corners.append((lat, lon))

        # 重新初始化地圖並添加角點
        self.map_widget.init_map()
        for lat, lon in self.corners:
            self.map_widget.add_corner(lat, lon)

        # 更新 UI
        self.parameter_panel.update_corner_count(len(self.corners))
        self.update_statusbar()
        logger.info(f"已同步 {len(corners)} 個角點")

    def on_obstacles_changed(self, obstacles):
        """處理障礙物變更"""
        self.obstacles = obstacles
        logger.info(f"障礙物已更新: {len(obstacles)} 個")
    
    def on_show_help(self):
        """顯示說明"""
        help_text = """
        <h2>無人機路徑規劃工具</h2>
        <h3>基本操作：</h3>
        <ul>
            <li><b>新增邊界點：</b> 在地圖上點擊</li>
            <li><b>移動邊界點：</b> 拖動地圖上的標記</li>
            <li><b>預覽路徑：</b> 點擊"預覽"按鈕</li>
            <li><b>匯出航點：</b> 點擊"匯出"按鈕</li>
        </ul>
        <h3>快捷鍵：</h3>
        <ul>
            <li>Ctrl+N: 新建任務</li>
            <li>Ctrl+O: 開啟任務</li>
            <li>Ctrl+S: 儲存任務</li>
            <li>Ctrl+E: 匯出航點</li>
            <li>Ctrl+R: 清除全部</li>
        </ul>
        """
        
        QMessageBox.information(self, "使用說明", help_text)
    
    def on_about(self):
        """關於"""
        about_text = """
        <h2>無人機網格航線規劃工具 V2.0</h2>
        <p><b>基於 PyQt6 的專業級路徑規劃系統</b></p>
        <p>支援功能：</p>
        <ul>
            <li>Survey Grid 測繪任務</li>
            <li>多機群飛協調</li>
            <li>智能避撞系統</li>
            <li>MAVLink 航點匯出</li>
        </ul>
        <p>© 2026 UAV Path Planner Team</p>
        """
        
        QMessageBox.about(self, "關於", about_text)
    
    # ==========================================
    # 輔助函數
    # ==========================================
    
    def has_unsaved_changes(self):
        """檢查是否有未儲存的變更"""
        # TODO: 實現變更檢測
        return False
    
    def update_statusbar(self):
        """更新狀態列"""
        # 更新航點數量
        self.waypoint_label.setText(f"航點: {len(self.waypoints)}")
        
        # 更新邊界點數量
        if self.corners:
            self.statusBar().showMessage(f"邊界點: {len(self.corners)} 個", 2000)
    
    def closeEvent(self, event):
        """視窗關閉事件"""
        if self.current_mission and self.has_unsaved_changes():
            reply = QMessageBox.question(
                self, "未儲存的變更",
                "當前任務有未儲存的變更，確定要退出嗎？",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            
            if reply == QMessageBox.StandardButton.No:
                event.ignore()
                return
        
        logger.info("應用程式關閉")
        event.accept()