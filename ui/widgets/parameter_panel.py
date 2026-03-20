"""
參數面板模組
提供飛行參數、測繪參數的設置界面
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QSlider, QSpinBox, QDoubleSpinBox,
    QComboBox, QCheckBox, QPushButton, QFormLayout, QFrame
)
from PyQt6.QtCore import Qt, pyqtSignal

from config import get_settings
from utils.logger import get_logger

# 獲取配置和日誌實例
settings = get_settings()
logger = get_logger()


class ParameterPanel(QWidget):
    """
    參數面板
    
    提供各種飛行和測繪參數的設置
    """
    
    # 信號定義
    parameters_changed = pyqtSignal(dict)  # 參數變更信號
    corner_added = pyqtSignal(float, float)  # 新增邊界點信號
    clear_corners_requested = pyqtSignal()  # 清除邊界點信號
    remove_last_corner_requested = pyqtSignal()  # 刪除最後一點信號
    open_click_map_requested = pyqtSignal()  # 打開點擊地圖視窗
    pick_center_requested = pyqtSignal()    # 從地圖點選圓心
    drag_circle_requested = pyqtSignal()    # 地圖拖曳定義圓形區域
    pick_home_point_requested  = pyqtSignal()  # 從地圖點選起飛點
    clear_home_point_requested = pyqtSignal()  # 清除起飛點
    manage_nfz_requested       = pyqtSignal()  # 開啟禁航區管理對話框

    def __init__(self, parent=None):
        """初始化參數面板"""
        super().__init__(parent)
        
        # 初始化參數字典
        self.parameters = {
            'altitude': 10.0,
            'speed': 3.0,
            'angle': 0.0,
            'spacing': 20.0,
            'yaw_speed': 60.0,
            'subdivisions': 1,
            'region_spacing': 3.0,
            'gap_spacings_m': [],        # 各邊界獨立間隔（條帶模式）
            'v_spacing_m': 3.0,          # 網格垂直間隔
            'transit_stagger_enabled': False,  # 轉場高度錯層
            'transit_stagger_alt': 10.0,       # 每台無人機高度錯層間距 (m)
            'reduce_overlap': True,
            'flight_mode': 'smart_collision',
            'algorithm': 'grid',
            'vehicle_type': '多旋翼',
            'vehicle_model': 'DJI Mavic 3',
            'turn_radius': 50.0,  # 固定翼轉彎半徑
            'lock_heading': False,
            'heading_angle': 0.0,
            # ── 固定翼專用參數 ──────────────────────────────────
            'fw_takeoff_bearing': 0.0,       # 起飛方向（度，0=北）
            'fw_runway_length': 50.0,        # 跑道長度（公尺）
            'fw_landing_bearing': 180.0,     # 降落進場方向（度）
            'fw_pattern_alt': 80.0,          # 五邊飛行高度（公尺）
            'fw_downwind_offset': 200.0,     # 下風邊側偏距離（公尺）
            'fw_pattern_leg': 300.0,         # 五邊腿長（公尺）
            'fw_final_dist': 400.0,          # 最終進場距離（公尺）
            'fw_scan_mode': 'spiral',        # 掃描模式: spiral / circular
            'fw_max_bank_deg': 45.0,         # 最大傾斜角 (ArduPlane LIM_ROLL_CD = 4500 cd)
            # ── 螺旋/同心圓圓心模式 ─────────────────────────────────────
            'circle_center_lat': 0.0,
            'circle_center_lon': 0.0,
            'circle_max_radius': 200.0,
            'circle_alt_step': 0.0,
            'circle_min_radius': 0.0,
            # ── 螺旋專用 ─────────────────────────────────────────────────
            'spiral_curvature': 1.0,         # 螺旋曲率倍數
            'spiral_alt_step': 0.0,          # 螺旋每圈高度步階 (m/rev)
            # ── 起飛點（固定翼跑道位置）────────────────────────────────
            'home_point_lat': 0.0,
            'home_point_lon': 0.0,
        }
        # 動態間隔 spinbox 列表（隨 n_regions 重建）
        self._gap_spinboxes: list = []
        
        # 建立 UI
        self.init_ui()
        
        logger.info("參數面板初始化完成")
    
    def init_ui(self):
        """初始化 UI"""
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        # 邊界點管理群組（放在最上方）
        self.corner_group = self.create_corner_management()
        layout.addWidget(self.corner_group)

        # 演算法與載具選擇群組
        algo_vehicle_group = self.create_algorithm_vehicle_selection()
        layout.addWidget(algo_vehicle_group)

        # 飛行參數群組
        flight_group = self.create_flight_parameters()
        layout.addWidget(flight_group)

        # 固定翼專用設定群組（預設隱藏，切換到固定翼時顯示）
        self.fixed_wing_group = self.create_fixed_wing_panel()
        layout.addWidget(self.fixed_wing_group)
        self.fixed_wing_group.setVisible(False)

        # 測繪參數群組
        survey_group = self.create_survey_parameters()
        layout.addWidget(survey_group)

        # 螺旋/圓心掃描設定群組（僅在 spiral / circular 模式顯示）
        self.circle_center_group = self.create_circle_center_panel()
        layout.addWidget(self.circle_center_group)
        self.circle_center_group.setVisible(False)

        # 子區域安全設定群組（分割 > 1 時才顯示）
        self.region_safety_group = self.create_region_safety_group()
        layout.addWidget(self.region_safety_group)
        self.region_safety_group.setVisible(False)

        # 進階參數群組
        advanced_group = self.create_advanced_parameters()
        layout.addWidget(advanced_group)

        # 添加彈性空間
        layout.addStretch()
    
    def create_corner_management(self):
        """創建邊界點管理群組"""
        group = QGroupBox("邊界點管理")
        layout = QVBoxLayout(group)

        # 座標輸入區（隱藏 - 改用地圖直接點擊）
        coord_layout = QHBoxLayout()

        # 緯度輸入
        lat_layout = QVBoxLayout()
        lat_layout.addWidget(QLabel("緯度:"))
        self.lat_input = QDoubleSpinBox()
        self.lat_input.setRange(-90.0, 90.0)
        self.lat_input.setDecimals(6)
        self.lat_input.setValue(settings.map.default_lat)
        lat_layout.addWidget(self.lat_input)
        coord_layout.addLayout(lat_layout)

        # 經度輸入
        lon_layout = QVBoxLayout()
        lon_layout.addWidget(QLabel("經度:"))
        self.lon_input = QDoubleSpinBox()
        self.lon_input.setRange(-180.0, 180.0)
        self.lon_input.setDecimals(6)
        self.lon_input.setValue(settings.map.default_lon)
        lon_layout.addWidget(self.lon_input)
        coord_layout.addLayout(lon_layout)

        # 隱藏手動輸入區域
        coord_widget = QWidget()
        coord_widget.setLayout(coord_layout)
        coord_widget.setVisible(False)  # 隱藏
        layout.addWidget(coord_widget)

        # 提示：使用地圖點擊添加
        click_hint = QLabel("🖱️ 請直接在地圖上左鍵點擊添加角點（右鍵開啟工具選單）")
        click_hint.setStyleSheet("""
            background-color: rgba(33, 150, 243, 0.1);
            color: #2196F3;
            padding: 10px;
            border-radius: 5px;
            font-weight: bold;
            font-size: 12px;
        """)
        click_hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(click_hint)

        # 刪除最後一點按鈕
        self.remove_last_corner_btn = QPushButton("↩️ 刪除最後一點")
        self.remove_last_corner_btn.setStyleSheet("background-color: #FF9800; color: white; font-weight: bold;")
        self.remove_last_corner_btn.setToolTip("刪除最後一個新增的角點（快捷鍵: Backspace）")
        self.remove_last_corner_btn.clicked.connect(self.on_remove_last_corner)
        layout.addWidget(self.remove_last_corner_btn)

        # 角點數量顯示
        self.corner_count_label = QLabel("目前角點: 0 個")
        self.corner_count_label.setStyleSheet("color: #2196F3; font-weight: bold;")
        layout.addWidget(self.corner_count_label)

        return group

    def create_circle_center_panel(self) -> QGroupBox:
        """創建螺旋/同心圓圓心掃描設定群組"""
        group = QGroupBox("螺旋 / 同心圓掃描設定")
        group.setObjectName("circleCenterGroup")
        group.setStyleSheet(
            "QGroupBox#circleCenterGroup { font-weight: bold; color: #2196F3; }"
        )
        layout = QVBoxLayout(group)
        layout.setSpacing(6)

        # ── 圓心座標顯示 ──
        center_frame = QFrame()
        center_frame.setFrameShape(QFrame.Shape.StyledPanel)
        center_frame_layout = QHBoxLayout(center_frame)
        center_frame_layout.setContentsMargins(6, 4, 6, 4)
        self.circle_center_label = QLabel("圓心：尚未設定")
        self.circle_center_label.setStyleSheet(
            "color: #888; font-size: 11px; font-style: italic;"
        )
        self.circle_center_label.setWordWrap(True)
        center_frame_layout.addWidget(self.circle_center_label, 1)
        layout.addWidget(center_frame)

        # ── 操作按鈕（拖曳定義 / 點選圓心）──
        btn_row = QHBoxLayout()
        drag_btn = QPushButton("地圖拖曳定義圓形")
        drag_btn.setStyleSheet(
            "background-color: #1565C0; color: white; font-weight: bold; padding: 5px;"
        )
        drag_btn.setToolTip(
            "按住滑鼠左鍵並在地圖上拖曳\n"
            "即可同時設定圓心與半徑（Mission Planner 風格）"
        )
        drag_btn.clicked.connect(self.drag_circle_requested.emit)
        btn_row.addWidget(drag_btn, 2)

        pick_btn = QPushButton("僅點選圓心")
        pick_btn.setStyleSheet(
            "background-color: #455A64; color: white; padding: 5px;"
        )
        pick_btn.setToolTip("點擊此鈕後，在地圖上左鍵點一下即可設定圓心（半徑手動調整）")
        pick_btn.clicked.connect(self.pick_center_requested.emit)
        btn_row.addWidget(pick_btn, 1)
        layout.addLayout(btn_row)

        # ── 共用參數表單 ──
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)

        # 最大半徑
        self.circle_max_radius_spin = QDoubleSpinBox()
        self.circle_max_radius_spin.setRange(10.0, 10000.0)
        self.circle_max_radius_spin.setValue(self.parameters['circle_max_radius'])
        self.circle_max_radius_spin.setSuffix(" m")
        self.circle_max_radius_spin.setDecimals(0)
        self.circle_max_radius_spin.setToolTip("從圓心向外掃描的最大半徑（拖曳時自動填入）")
        self.circle_max_radius_spin.valueChanged.connect(
            lambda v: self.update_parameter('circle_max_radius', v)
        )
        form.addRow("最大半徑:", self.circle_max_radius_spin)

        # 起始半徑（同心圓最小圈）
        self.circle_min_radius_spin = QDoubleSpinBox()
        self.circle_min_radius_spin.setRange(0.0, 5000.0)
        self.circle_min_radius_spin.setValue(self.parameters['circle_min_radius'])
        self.circle_min_radius_spin.setSuffix(" m")
        self.circle_min_radius_spin.setDecimals(0)
        self.circle_min_radius_spin.setToolTip("掃描起始半徑（0 = 從圓心開始）")
        self.circle_min_radius_spin.valueChanged.connect(
            lambda v: self.update_parameter('circle_min_radius', v)
        )
        form.addRow("起始半徑:", self.circle_min_radius_spin)

        # ── 同心圓專用：高度步階 ──
        self.circle_alt_step_label_row = QLabel("高度步階:")
        self.circle_alt_step_spin = QDoubleSpinBox()
        self.circle_alt_step_spin.setRange(0.0, 200.0)
        self.circle_alt_step_spin.setValue(self.parameters['circle_alt_step'])
        self.circle_alt_step_spin.setSuffix(" m/圈")
        self.circle_alt_step_spin.setDecimals(1)
        self.circle_alt_step_spin.setToolTip(
            "【同心圓】每圈高度遞增量（公尺）\n"
            "0 = 所有圈等高，正值 = 向外擴張同時爬升"
        )
        self.circle_alt_step_spin.valueChanged.connect(
            lambda v: self.update_parameter('circle_alt_step', v)
        )
        form.addRow(self.circle_alt_step_label_row, self.circle_alt_step_spin)

        layout.addLayout(form)

        # ── 螺旋專用參數（僅 spiral 模式顯示）──
        self.spiral_params_frame = QFrame()
        self.spiral_params_frame.setFrameShape(QFrame.Shape.StyledPanel)
        self.spiral_params_frame.setStyleSheet(
            "QFrame { border: 1px solid #1565C0; border-radius: 4px; }"
        )
        spiral_form_layout = QVBoxLayout(self.spiral_params_frame)
        spiral_form_layout.setContentsMargins(6, 4, 6, 4)
        spiral_title = QLabel("螺旋路徑參數")
        spiral_title.setStyleSheet("font-weight: bold; color: #1565C0; font-size: 11px;")
        spiral_form_layout.addWidget(spiral_title)

        spiral_form = QFormLayout()
        spiral_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)

        # 螺旋曲率
        self.spiral_curvature_spin = QDoubleSpinBox()
        self.spiral_curvature_spin.setRange(0.1, 5.0)
        self.spiral_curvature_spin.setValue(self.parameters['spiral_curvature'])
        self.spiral_curvature_spin.setSingleStep(0.1)
        self.spiral_curvature_spin.setDecimals(2)
        self.spiral_curvature_spin.setToolTip(
            "螺旋曲率倍數\n"
            "1.0 = 標準（圈距 = 航線間距）\n"
            "< 1.0 = 緊密螺旋（圈距較小）\n"
            "> 1.0 = 鬆散螺旋（圈距較大）"
        )
        self.spiral_curvature_spin.valueChanged.connect(
            lambda v: self.update_parameter('spiral_curvature', v)
        )
        spiral_form.addRow("螺旋曲率:", self.spiral_curvature_spin)

        # 螺旋高度步階
        self.spiral_alt_step_spin = QDoubleSpinBox()
        self.spiral_alt_step_spin.setRange(0.0, 200.0)
        self.spiral_alt_step_spin.setValue(self.parameters['spiral_alt_step'])
        self.spiral_alt_step_spin.setSuffix(" m/圈")
        self.spiral_alt_step_spin.setDecimals(1)
        self.spiral_alt_step_spin.setToolTip(
            "【螺旋】每旋轉一圈（360°）的高度增量\n"
            "0 = 等高螺旋，正值 = 螺旋上升（3D 螺旋）"
        )
        self.spiral_alt_step_spin.valueChanged.connect(
            lambda v: self.update_parameter('spiral_alt_step', v)
        )
        spiral_form.addRow("高度步階:", self.spiral_alt_step_spin)

        spiral_form_layout.addLayout(spiral_form)
        layout.addWidget(self.spiral_params_frame)
        self.spiral_params_frame.setVisible(False)  # 預設隱藏，切到 spiral 時顯示

        # ── 提示說明 ──
        hint = QLabel(
            "圈間間距請使用上方「航線間距」調整。\n"
            "飛行高度請使用上方「飛行高度」設定。"
        )
        hint.setStyleSheet(
            "background-color: rgba(33,150,243,0.08);"
            "color: #2196F3; padding: 6px; border-radius: 4px; font-size: 10px;"
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)

        return group

    def set_circle_radius_display(self, radius_m: float):
        """從拖曳結果更新最大半徑 spinbox"""
        if hasattr(self, 'circle_max_radius_spin') and radius_m > 1.0:
            self.circle_max_radius_spin.blockSignals(True)
            self.circle_max_radius_spin.setValue(round(radius_m))
            self.circle_max_radius_spin.blockSignals(False)
            self.update_parameter('circle_max_radius', round(radius_m))

    def create_algorithm_vehicle_selection(self):
        """創建演算法與載具選擇群組"""
        group = QGroupBox("演算法與載具")
        layout = QFormLayout(group)

        # 路徑演算法選擇
        self.algorithm_combo = QComboBox()
        self.algorithm_combo.addItems([
            "網格掃描 (Grid) - 覆蓋任務",
            "螺旋掃描 (Spiral) - 覆蓋任務",
            "同心圓掃描 (Circular) - 覆蓋任務",
            "RRT 路徑規劃 - 點對點",
            "RRT* 路徑規劃 - 最優路徑",
            "Dijkstra 路徑規劃 - 最短路徑",
            "DWA 動態窗口 - 即時避障"
        ])
        self.algorithm_combo.setCurrentIndex(0)
        self.algorithm_combo.currentIndexChanged.connect(self.on_algorithm_changed)
        self.algorithm_combo.setToolTip(
            "Grid/Spiral/Circular: 適用於區域覆蓋任務\n"
            "RRT/Dijkstra: 適用於點對點路徑規劃\n"
            "DWA: 適用於即時避障"
        )
        layout.addRow("路徑演算法:", self.algorithm_combo)

        # 載具類型選擇
        self.vehicle_type_combo = QComboBox()
        self.vehicle_type_combo.addItems(["多旋翼", "固定翼", "VTOL"])
        self.vehicle_type_combo.setCurrentIndex(0)
        self.vehicle_type_combo.currentIndexChanged.connect(self.on_vehicle_type_changed)
        layout.addRow("載具類型:", self.vehicle_type_combo)

        # 載具型號選擇
        self.vehicle_model_combo = QComboBox()
        self._update_vehicle_models("多旋翼")
        self.vehicle_model_combo.currentIndexChanged.connect(self.on_vehicle_model_changed)
        layout.addRow("載具型號:", self.vehicle_model_combo)

        # 載具資訊標籤
        self.vehicle_info_label = QLabel("選擇載具以顯示資訊")
        self.vehicle_info_label.setStyleSheet("color: #888; font-size: 10px;")
        self.vehicle_info_label.setWordWrap(True)
        layout.addRow("", self.vehicle_info_label)

        return group

    def _update_vehicle_models(self, vehicle_type: str):
        """更新載具型號列表"""
        self.vehicle_model_combo.clear()

        if vehicle_type == "多旋翼":
            self.vehicle_model_combo.addItems([
                "DJI Mavic 3",
                "DJI Phantom 4 Pro",
                "DJI Mini 3 Pro",
                "Generic Quadcopter"
            ])
        elif vehicle_type == "固定翼":
            self.vehicle_model_combo.addItems([
                "衝浪者 (Surfer)",
                "Generic Fixed Wing",
            ])
        elif vehicle_type == "VTOL":
            self.vehicle_model_combo.addItems([
                "Generic VTOL"
            ])

    def on_algorithm_changed(self, index):
        """處理演算法變更"""
        algorithms = ['grid', 'spiral', 'circular', 'rrt', 'rrt_star', 'dijkstra', 'dwa']
        algorithm = algorithms[index] if index < len(algorithms) else 'grid'
        self.update_parameter('algorithm', algorithm)

        # 螺旋/同心圓模式：顯示圓心掃描設定
        is_circle_mode = algorithm in ('spiral', 'circular')
        is_spiral = algorithm == 'spiral'
        is_circular = algorithm == 'circular'
        if hasattr(self, 'circle_center_group'):
            self.circle_center_group.setVisible(is_circle_mode)
        # 螺旋專用參數區塊（曲率 / 高度步階）
        if hasattr(self, 'spiral_params_frame'):
            self.spiral_params_frame.setVisible(is_spiral)
        # 同心圓高度步階（螺旋模式用 spiral_alt_step，同心圓用 circle_alt_step）
        if hasattr(self, 'circle_alt_step_spin'):
            self.circle_alt_step_spin.setVisible(is_circular)
        if hasattr(self, 'circle_alt_step_label_row'):
            self.circle_alt_step_label_row.setVisible(is_circular)

        # 更新主視窗的演算法設定
        main_window = self.parent()
        if main_window:
            main_window.current_algorithm = algorithm

        # 顯示演算法說明
        algorithm_info = {
            'grid': "網格掃描：適合覆蓋測繪任務，之字形路徑",
            'spiral': "螺旋掃描：從外圍向中心螺旋掃描",
            'circular': "同心圓掃描：從中心向外擴張的同心圓環路徑，可逐圈上升",
            'rrt': "RRT 演算法：快速探索隨機樹，適合複雜環境",
            'rrt_star': "RRT* 演算法：RRT 的最優化版本",
            'dijkstra': "Dijkstra 演算法：保證最短路徑",
            'dwa': "DWA 動態窗口：即時避障，適合動態環境"
        }
        info = algorithm_info.get(algorithm, "")
        self.algorithm_combo.setToolTip(info)

        logger.info(f"演算法變更: {algorithm} - {info}")

    def on_vehicle_type_changed(self, index):
        """處理載具類型變更"""
        vehicle_types = ["多旋翼", "固定翼", "VTOL"]
        vehicle_type = vehicle_types[index] if index < len(vehicle_types) else "多旋翼"
        self._update_vehicle_models(vehicle_type)
        self.update_parameter('vehicle_type', vehicle_type)

        is_fixed_wing = (vehicle_type == "固定翼")

        # 顯示/隱藏轉彎半徑（飛行參數群組內）
        if hasattr(self, 'turn_radius_spin'):
            self.turn_radius_spin.setVisible(is_fixed_wing)
            self.turn_radius_label.setVisible(is_fixed_wing)

        # 顯示/隱藏固定翼專用設定群組
        if hasattr(self, 'fixed_wing_group'):
            self.fixed_wing_group.setVisible(is_fixed_wing)
            if is_fixed_wing:
                # 切換到固定翼時立即執行轉彎半徑檢測
                self._update_turn_radius_check()

        logger.info(f"載具類型變更: {vehicle_type}")

    def on_vehicle_model_changed(self, index):
        """處理載具型號變更"""
        model = self.vehicle_model_combo.currentText()
        self.update_parameter('vehicle_model', model)

        # 更新載具資訊
        vehicle_info = self._get_vehicle_info(model)
        self.vehicle_info_label.setText(vehicle_info)

        # 如果有對應的參數，自動更新飛行參數
        self._apply_vehicle_defaults(model)

        logger.info(f"載具型號變更: {model}")

    def _get_vehicle_info(self, model: str) -> str:
        """獲取載具資訊"""
        info_db = {
            "DJI Mavic 3": "最大速度: 19m/s | 飛行時間: 46min | 抗風: 12m/s",
            "DJI Phantom 4 Pro": "最大速度: 20m/s | 飛行時間: 30min | 抗風: 10m/s",
            "DJI Mini 3 Pro": "最大速度: 16m/s | 飛行時間: 34min | 抗風: 10.7m/s",
            "Generic Quadcopter": "最大速度: 15m/s | 飛行時間: 25min | 抗風: 10m/s",
            "衝浪者 (Surfer)": "巡航: 18m/s | 飛行時間: 90min | 最小轉彎半徑: 30m | 跑道: 50m",
            "Generic Fixed Wing": "最大速度: 25m/s | 飛行時間: 120min | 最小轉彎半徑: 50m",
            "Generic VTOL": "最大速度: 30m/s | 飛行時間: 90min | 抗風: 12m/s",
        }
        return info_db.get(model, "無資訊")

    def _apply_vehicle_defaults(self, model: str):
        """根據載具型號應用預設參數（含固定翼五邊參數）"""
        defaults_db = {
            "DJI Mavic 3":       {'speed': 15.0, 'altitude': 60.0,  'turn_radius': 0},
            "DJI Phantom 4 Pro": {'speed': 12.0, 'altitude': 50.0,  'turn_radius': 0},
            "DJI Mini 3 Pro":    {'speed': 10.0, 'altitude': 40.0,  'turn_radius': 0},
            "Generic Quadcopter":{'speed': 3.0,  'altitude': 10.0,  'turn_radius': 0},
            "衝浪者 (Surfer)": {
                'speed': 18.0, 'altitude': 100.0, 'turn_radius': 50.0,
                'fw_runway_length': 50.0, 'fw_pattern_alt': 80.0,
                'fw_downwind_offset': 200.0, 'fw_pattern_leg': 300.0,
                'fw_final_dist': 400.0,
            },
            "Generic Fixed Wing": {
                'speed': 18.0, 'altitude': 100.0, 'turn_radius': 80.0,
                'fw_runway_length': 30.0, 'fw_pattern_alt': 100.0,
                'fw_downwind_offset': 250.0, 'fw_pattern_leg': 400.0,
                'fw_final_dist': 500.0,
            },
            "Generic VTOL": {'speed': 15.0, 'altitude': 80.0, 'turn_radius': 30.0},
        }

        defaults = defaults_db.get(model)
        if not defaults:
            return

        self.speed_spin.setValue(defaults['speed'])
        self.altitude_spin.setValue(defaults['altitude'])

        if defaults.get('turn_radius', 0) > 0:
            self.turn_radius_spin.setValue(defaults['turn_radius'])

        # 固定翼五邊降落參數
        if hasattr(self, 'fw_runway_spin') and 'fw_runway_length' in defaults:
            self.fw_runway_spin.setValue(defaults['fw_runway_length'])
        if hasattr(self, 'fw_pattern_alt_spin') and 'fw_pattern_alt' in defaults:
            self.fw_pattern_alt_spin.setValue(defaults['fw_pattern_alt'])
        if hasattr(self, 'fw_downwind_offset_spin') and 'fw_downwind_offset' in defaults:
            self.fw_downwind_offset_spin.setValue(defaults['fw_downwind_offset'])
        if hasattr(self, 'fw_pattern_leg_spin') and 'fw_pattern_leg' in defaults:
            self.fw_pattern_leg_spin.setValue(defaults['fw_pattern_leg'])
        if hasattr(self, 'fw_final_dist_spin') and 'fw_final_dist' in defaults:
            self.fw_final_dist_spin.setValue(defaults['fw_final_dist'])

        # 套用後立即更新轉彎半徑檢測
        self._update_turn_radius_check()

    def on_add_preset_area(self):
        """添加預設測試區域"""
        # 清除現有角點
        self.clear_corners_requested.emit()

        # 在預設位置周圍創建一個約 200m x 200m 的矩形
        center_lat = settings.map.default_lat
        center_lon = settings.map.default_lon

        # 大約 0.0018 度 ≈ 200m
        offset = 0.0009

        corners = [
            (center_lat + offset, center_lon - offset),  # 左上
            (center_lat + offset, center_lon + offset),  # 右上
            (center_lat - offset, center_lon + offset),  # 右下
            (center_lat - offset, center_lon - offset),  # 左下
        ]

        for lat, lon in corners:
            self.corner_added.emit(lat, lon)

        logger.info("已添加預設測試區域（4個角點）")

    def on_add_corner(self):
        """新增角點"""
        lat = self.lat_input.value()
        lon = self.lon_input.value()
        self.corner_added.emit(lat, lon)
        logger.info(f"手動新增角點: ({lat:.6f}, {lon:.6f})")

    # ──────────────────────────────────────────────────────────────────
    # 子區域安全設定（動態 UI）
    # ──────────────────────────────────────────────────────────────────

    def create_region_safety_group(self) -> QGroupBox:
        """建立「子區域安全設定」群組（分割 > 1 時才顯示）"""
        group = QGroupBox("子區域安全設定")
        outer = QVBoxLayout(group)
        outer.setSpacing(6)

        # ── 說明標籤 ────────────────────────────────────────────────
        hint = QLabel("設定各子區域邊界間隔，防止相鄰無人機物理碰撞")
        hint.setStyleSheet("color:#888; font-size:10px;")
        hint.setWordWrap(True)
        outer.addWidget(hint)

        # ── 動態間隔區塊（由 _rebuild_region_gaps 填充）────────────
        self._gap_container = QWidget()
        self._gap_vbox = QVBoxLayout(self._gap_container)
        self._gap_vbox.setContentsMargins(0, 0, 0, 0)
        self._gap_vbox.setSpacing(4)
        outer.addWidget(self._gap_container)

        # ── 轉場高度錯層 ─────────────────────────────────────────────
        self._transit_check = QCheckBox("啟用轉場高度錯層（防止路線交錯碰撞）")
        self._transit_check.setToolTip(
            "各無人機從起飛點飛往各子區域時，\n"
            "以不同高度轉場，避免路線在空中交錯"
        )
        self._transit_check.stateChanged.connect(self._on_transit_stagger_toggled)
        outer.addWidget(self._transit_check)

        # 錯層間距（預設隱藏）
        stagger_row = QHBoxLayout()
        stagger_row.addWidget(QLabel("各機高度間距:"))
        self._stagger_spin = QDoubleSpinBox()
        self._stagger_spin.setRange(5.0, 100.0)
        self._stagger_spin.setValue(10.0)
        self._stagger_spin.setSuffix(" m")
        self._stagger_spin.setDecimals(1)
        self._stagger_spin.setToolTip("相鄰無人機轉場時的高度差（公尺）")
        self._stagger_spin.valueChanged.connect(
            lambda v: self.update_parameter('transit_stagger_alt', v)
        )
        stagger_row.addWidget(self._stagger_spin)
        stagger_row.addStretch()

        self._stagger_detail = QLabel("")
        self._stagger_detail.setStyleSheet("color:#2196F3; font-size:10px;")
        self._stagger_detail.setWordWrap(True)

        self._stagger_widget = QWidget()
        stagger_inner = QVBoxLayout(self._stagger_widget)
        stagger_inner.setContentsMargins(16, 0, 0, 0)
        stagger_inner.addLayout(stagger_row)
        stagger_inner.addWidget(self._stagger_detail)
        self._stagger_widget.setVisible(False)
        outer.addWidget(self._stagger_widget)

        return group

    def _rebuild_region_gaps(self, n_regions: int):
        """依分割數量重建動態間隔 spinbox"""
        # 清除舊 spinbox
        self._gap_spinboxes.clear()
        while self._gap_vbox.count():
            item = self._gap_vbox.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        if n_regions <= 1:
            self.region_safety_group.setVisible(False)
            self.parameters['gap_spacings_m'] = []
            return

        self.region_safety_group.setVisible(True)
        default_gap = self.parameters.get('region_spacing', 3.0)

        if n_regions in (2, 3, 5):
            # 條帶模式：n-1 個邊界間隔
            n_gaps = n_regions - 1
            self.parameters['gap_spacings_m'] = [default_gap] * n_gaps
            for i in range(n_gaps):
                row_w = QWidget()
                row = QHBoxLayout(row_w)
                row.setContentsMargins(0, 0, 0, 0)
                lbl = QLabel(f"區域 {i+1}–{i+2} 間距:")
                lbl.setMinimumWidth(95)
                row.addWidget(lbl)
                sb = QDoubleSpinBox()
                sb.setRange(0.0, 100.0)
                sb.setValue(default_gap)
                sb.setSuffix(" m")
                sb.setDecimals(1)
                sb.valueChanged.connect(
                    lambda v, idx=i: self._on_gap_changed(idx, v)
                )
                row.addWidget(sb)
                row.addStretch()
                self._gap_vbox.addWidget(row_w)
                self._gap_spinboxes.append(sb)

        elif n_regions == 4:
            # 2×2 網格：水平間距 + 垂直間距
            self.parameters['gap_spacings_m'] = []
            for label_text, param_key in [
                ("水平（列）間距:", 'region_spacing'),
                ("垂直（行）間距:", 'v_spacing_m'),
            ]:
                row_w = QWidget()
                row = QHBoxLayout(row_w)
                row.setContentsMargins(0, 0, 0, 0)
                lbl = QLabel(label_text)
                lbl.setMinimumWidth(95)
                row.addWidget(lbl)
                sb = QDoubleSpinBox()
                sb.setRange(0.0, 100.0)
                sb.setValue(self.parameters.get(param_key, default_gap))
                sb.setSuffix(" m")
                sb.setDecimals(1)
                key = param_key
                sb.valueChanged.connect(
                    lambda v, k=key: self.update_parameter(k, v)
                )
                row.addWidget(sb)
                row.addStretch()
                self._gap_vbox.addWidget(row_w)
                self._gap_spinboxes.append(sb)

        elif n_regions == 6:
            # 3×2 網格：水平間距 + 垂直間距
            self.parameters['gap_spacings_m'] = []
            for label_text, param_key in [
                ("水平（列）間距:", 'region_spacing'),
                ("垂直（行）間距:", 'v_spacing_m'),
            ]:
                row_w = QWidget()
                row = QHBoxLayout(row_w)
                row.setContentsMargins(0, 0, 0, 0)
                lbl = QLabel(label_text)
                lbl.setMinimumWidth(95)
                row.addWidget(lbl)
                sb = QDoubleSpinBox()
                sb.setRange(0.0, 100.0)
                sb.setValue(self.parameters.get(param_key, default_gap))
                sb.setSuffix(" m")
                sb.setDecimals(1)
                key = param_key
                sb.valueChanged.connect(
                    lambda v, k=key: self.update_parameter(k, v)
                )
                row.addWidget(sb)
                row.addStretch()
                self._gap_vbox.addWidget(row_w)
                self._gap_spinboxes.append(sb)

        # 更新轉場高度錯層說明
        self._update_stagger_detail(n_regions)

    def _on_gap_changed(self, idx: int, value: float):
        """某條邊界間隔變更"""
        gaps = self.parameters.get('gap_spacings_m', [])
        if idx < len(gaps):
            gaps[idx] = value
        else:
            while len(gaps) <= idx:
                gaps.append(value)
            gaps[idx] = value
        self.parameters['gap_spacings_m'] = gaps
        self.parameters_changed.emit({'gap_spacings_m': gaps})
        logger.debug(f"邊界間隔更新: index={idx}, value={value}m, all={gaps}")

    def _on_transit_stagger_toggled(self, state):
        """轉場錯層開關"""
        enabled = (state == Qt.CheckState.Checked.value)
        self._stagger_widget.setVisible(enabled)
        self.update_parameter('transit_stagger_enabled', enabled)
        if enabled:
            self._update_stagger_detail(self.parameters.get('subdivisions', 1))

    def _update_stagger_detail(self, n_regions: int):
        """更新轉場高度說明標籤"""
        if not self._transit_check.isChecked() or n_regions <= 1:
            self._stagger_detail.setText("")
            return
        base = self.parameters.get('altitude', 10.0)
        step = self._stagger_spin.value()
        lines = []
        for i in range(n_regions):
            lines.append(f"區域 {i+1}: {base + i * step:.0f} m")
        self._stagger_detail.setText("轉場高度: " + "  |  ".join(lines))

    def on_remove_last_corner(self):
        """刪除最後一個角點"""
        self.remove_last_corner_requested.emit()
        logger.info("請求刪除最後一個角點")

    def on_clear_corners(self):
        """清除所有角點"""
        self.clear_corners_requested.emit()
        logger.info("請求清除所有角點")

    def update_corner_count(self, count: int):
        """更新角點數量顯示"""
        self.corner_count_label.setText(f"目前角點: {count} 個")
        if count >= 3:
            self.corner_count_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        else:
            self.corner_count_label.setStyleSheet("color: #f44336; font-weight: bold;")

    def set_circle_center_display(self, lat: float, lon: float, radius_m: float = 0.0):
        """更新圓心座標顯示（拖曳時同時帶入半徑）"""
        radius_str = f"  ／  半徑 {radius_m:.0f} m" if radius_m > 1.0 else ""
        if hasattr(self, 'circle_center_label'):
            self.circle_center_label.setText(
                f"圓心：{lat:.6f}, {lon:.6f}{radius_str}"
            )
            self.circle_center_label.setStyleSheet(
                "color: #4CAF50; font-size: 11px; font-weight: bold;"
            )
        self.update_parameter('circle_center_lat', lat)
        self.update_parameter('circle_center_lon', lon)
        if radius_m > 1.0:
            self.set_circle_radius_display(radius_m)

    def set_home_point_display(self, lat: float, lon: float):
        """更新起飛點座標顯示"""
        if hasattr(self, 'home_point_label'):
            self.home_point_label.setText(f"起飛點：{lat:.6f}, {lon:.6f}")
            self.home_point_label.setStyleSheet(
                "color: #4CAF50; font-size: 11px; font-weight: bold;"
            )

    def clear_home_point_display(self):
        """清除起飛點顯示（由 main_window 呼叫，純 UI 更新）"""
        if hasattr(self, 'home_point_label'):
            self.home_point_label.setText("起飛點：尚未設定（預設使用掃描區中心）")
            self.home_point_label.setStyleSheet(
                "color: #888; font-size: 11px; font-style: italic;"
            )

    def create_flight_parameters(self):
        """創建飛行參數群組"""
        group = QGroupBox("飛行參數")
        layout = QFormLayout(group)
        
        # 飛行高度
        self.altitude_spin = QDoubleSpinBox()
        self.altitude_spin.setRange(settings.safety.min_altitude_m, 
                                settings.safety.max_altitude_m)
        self.altitude_spin.setValue(self.parameters['altitude'])
        self.altitude_spin.setSuffix(" m")
        self.altitude_spin.setDecimals(1)
        self.altitude_spin.valueChanged.connect(lambda v: self.update_parameter('altitude', v))
        layout.addRow("飛行高度:", self.altitude_spin)
        
        # 飛行速度
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(settings.safety.min_speed_mps,
                                settings.safety.max_speed_mps)
        self.speed_spin.setValue(self.parameters['speed'])
        self.speed_spin.setSuffix(" m/s")
        self.speed_spin.setDecimals(1)
        self.speed_spin.valueChanged.connect(lambda v: self.update_parameter('speed', v))
        self.speed_spin.valueChanged.connect(lambda _: self._update_turn_radius_check())
        layout.addRow("飛行速度:", self.speed_spin)
        
        # 轉向速度
        self.yaw_speed_spin = QDoubleSpinBox()
        self.yaw_speed_spin.setRange(10.0, 360.0)
        self.yaw_speed_spin.setValue(self.parameters['yaw_speed'])
        self.yaw_speed_spin.setSuffix(" °/s")
        self.yaw_speed_spin.setDecimals(1)
        self.yaw_speed_spin.valueChanged.connect(lambda v: self.update_parameter('yaw_speed', v))
        layout.addRow("轉向速度:", self.yaw_speed_spin)

        # 固定翼轉彎半徑（預設隱藏）
        self.turn_radius_label = QLabel("轉彎半徑:")
        self.turn_radius_spin = QDoubleSpinBox()
        self.turn_radius_spin.setRange(10.0, 500.0)
        self.turn_radius_spin.setValue(self.parameters['turn_radius'])
        self.turn_radius_spin.setSuffix(" m")
        self.turn_radius_spin.setDecimals(1)
        self.turn_radius_spin.setToolTip("固定翼飛機的最小轉彎半徑，用於生成平滑路徑")
        self.turn_radius_spin.valueChanged.connect(lambda v: self.update_parameter('turn_radius', v))
        self.turn_radius_spin.valueChanged.connect(lambda _: self._update_turn_radius_check())
        layout.addRow(self.turn_radius_label, self.turn_radius_spin)

        # 預設隱藏固定翼參數
        self.turn_radius_label.setVisible(False)
        self.turn_radius_spin.setVisible(False)

        # ── Heading Lock（多旋翼專用）────────────────────────────────────────
        self.lock_heading_check = QCheckBox("鎖定機頭朝向 (Heading Lock)")
        self.lock_heading_check.setToolTip(
            "勾選後，匯出 Mission Planner 檔案時所有\n"
            "CONDITION_YAW 指令都會使用固定方向，\n"
            "而非自動計算到下一航點的方位角。"
        )
        self.lock_heading_check.stateChanged.connect(self.on_lock_heading_changed)
        layout.addRow("", self.lock_heading_check)

        # Heading 滑條（預設隱藏，勾選後顯示）
        self.heading_row_label = QLabel("機頭方向:")
        heading_layout = QHBoxLayout()
        self.heading_slider = QSlider(Qt.Orientation.Horizontal)
        self.heading_slider.setRange(0, 359)
        self.heading_slider.setValue(0)
        self.heading_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.heading_slider.setTickInterval(45)
        self.heading_value_label = QLabel("0° 北")
        self.heading_value_label.setMinimumWidth(55)
        self.heading_value_label.setStyleSheet("font-weight: bold; color: #2196F3;")
        self.heading_slider.valueChanged.connect(self.on_heading_changed)
        heading_layout.addWidget(self.heading_slider)
        heading_layout.addWidget(self.heading_value_label)
        self.heading_container = QWidget()
        self.heading_container.setLayout(heading_layout)
        layout.addRow(self.heading_row_label, self.heading_container)

        # 預設隱藏 heading 滑條
        self.heading_row_label.setVisible(False)
        self.heading_container.setVisible(False)

        return group
    
    def create_fixed_wing_panel(self) -> QGroupBox:
        """
        創建固定翼專用設定群組

        包含：
          1. 轉彎半徑合規性檢測（即時更新）
          2. 起飛設定（方向、跑道長度）
          3. 降落設定（五邊進場各參數）
        """
        group = QGroupBox("固定翼專用設定")
        group.setObjectName("fixedWingGroup")
        # 使用 object-name 選擇器，防止樣式向下繼承至子 QGroupBox
        group.setStyleSheet("QGroupBox#fixedWingGroup { font-weight: bold; color: #FF9800; }")
        outer = QVBoxLayout(group)
        outer.setSpacing(8)

        # ── 0. 起飛點設定 ─────────────────────────────────────
        home_group = QGroupBox("起飛點設定")
        home_layout = QVBoxLayout(home_group)
        home_layout.setSpacing(4)

        self.home_point_label = QLabel("起飛點：尚未設定（預設使用掃描區中心）")
        self.home_point_label.setStyleSheet(
            "color: #888; font-size: 11px; font-style: italic;"
        )
        self.home_point_label.setWordWrap(True)
        home_layout.addWidget(self.home_point_label)

        pick_home_btn = QPushButton("📍 從地圖點選起飛點")
        pick_home_btn.setStyleSheet(
            "background-color: #455A64; color: white; padding: 6px; border-radius: 3px;"
        )
        pick_home_btn.clicked.connect(self.pick_home_point_requested.emit)
        home_layout.addWidget(pick_home_btn)

        clear_home_btn = QPushButton("✖ 清除起飛點")
        clear_home_btn.setStyleSheet(
            "background-color: #78909C; color: white; padding: 4px; border-radius: 3px;"
        )
        clear_home_btn.clicked.connect(self.clear_home_point_requested.emit)
        home_layout.addWidget(clear_home_btn)

        outer.addWidget(home_group)

        # ── 0.5. 禁航區設定 (NFZ) ─────────────────────────────
        nfz_group = QGroupBox("禁航區設定 (NFZ)")
        nfz_layout = QVBoxLayout(nfz_group)
        nfz_layout.setSpacing(4)

        self.nfz_count_label = QLabel("目前禁航區：0 個")
        self.nfz_count_label.setStyleSheet("color: #888; font-size: 11px;")
        nfz_layout.addWidget(self.nfz_count_label)

        manage_nfz_btn = QPushButton("🚫 禁航區管理...")
        manage_nfz_btn.setStyleSheet(
            "background-color: #B71C1C; color: white; padding: 6px; border-radius: 3px;"
        )
        manage_nfz_btn.clicked.connect(self.manage_nfz_requested.emit)
        nfz_layout.addWidget(manage_nfz_btn)

        outer.addWidget(nfz_group)

        # ── 1. 轉彎半徑合規性檢測 ────────────────────────────
        check_group = QGroupBox("轉彎半徑合規性檢測")
        check_layout = QVBoxLayout(check_group)
        check_layout.setSpacing(4)

        # 狀態燈 + 摘要（一行）
        status_row = QHBoxLayout()
        self.fw_status_dot = QLabel("●")
        self.fw_status_dot.setStyleSheet("color: #888; font-size: 20px;")
        self.fw_status_dot.setFixedWidth(24)
        self.fw_status_summary = QLabel("請先設定轉彎半徑與速度")
        self.fw_status_summary.setStyleSheet("font-weight: bold; font-size: 11px;")
        self.fw_status_summary.setWordWrap(True)
        status_row.addWidget(self.fw_status_dot)
        status_row.addWidget(self.fw_status_summary, 1)
        check_layout.addLayout(status_row)

        # 詳細說明
        self.fw_status_detail = QLabel("")
        self.fw_status_detail.setStyleSheet("color: #888; font-size: 10px;")
        self.fw_status_detail.setWordWrap(True)
        check_layout.addWidget(self.fw_status_detail)

        # ── ArduPlane LIM_ROLL_CD 設定 ─────────────────────────
        bank_form = QFormLayout()
        bank_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        bank_form.setContentsMargins(0, 4, 0, 4)

        # 最大傾斜角欄位
        bank_row = QHBoxLayout()
        self.fw_max_bank_spin = QDoubleSpinBox()
        self.fw_max_bank_spin.setRange(10.0, 60.0)
        self.fw_max_bank_spin.setValue(self.parameters.get('fw_max_bank_deg', 45.0))
        self.fw_max_bank_spin.setSuffix("°")
        self.fw_max_bank_spin.setSingleStep(1.0)
        self.fw_max_bank_spin.setDecimals(1)
        self.fw_max_bank_spin.setToolTip(
            "最大傾斜角（對應 ArduPlane LIM_ROLL_CD）\n"
            "Mission Planner SITL 預設: 45° (= 4500 cd)\n"
            "衝浪者建議: 45°   一般固定翼: 30–45°\n\n"
            "物理公式：R_min = V² / (g × tan(φ))\n"
            "  V=巡航速度, g=9.81, φ=傾斜角"
        )
        self.fw_max_bank_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_max_bank_deg', v)
        )
        self.fw_max_bank_spin.valueChanged.connect(self._on_bank_angle_changed)

        # 即時顯示對應的最小轉彎半徑
        self.fw_min_radius_label = QLabel("→ 最小半徑: ─ m")
        self.fw_min_radius_label.setStyleSheet(
            "color: #2196F3; font-size: 10px; font-weight: bold;"
        )
        bank_row.addWidget(self.fw_max_bank_spin)
        bank_row.addWidget(self.fw_min_radius_label, 1)
        bank_form.addRow("最大傾斜角\n(LIM_ROLL_CD):", bank_row)

        # 套用：由傾斜角→最小半徑→填入轉彎半徑欄位
        apply_bank_btn = QPushButton("套用最小半徑至轉彎半徑設定")
        apply_bank_btn.setStyleSheet(
            "background-color: #37474F; color: white; font-size: 10px; padding: 3px;"
        )
        apply_bank_btn.setToolTip(
            "依目前「最大傾斜角」和「速度」計算最小轉彎半徑，\n"
            "並自動填入上方「轉彎半徑」欄位"
        )
        apply_bank_btn.clicked.connect(self._apply_min_radius_from_bank)
        bank_form.addRow("", apply_bank_btn)

        check_layout.addLayout(bank_form)

        # 手動觸發檢測按鈕
        check_btn = QPushButton("立即檢測轉彎半徑")
        check_btn.setStyleSheet(
            "background-color: #1565C0; color: white; font-weight: bold;"
        )
        check_btn.clicked.connect(self._update_turn_radius_check)
        check_layout.addWidget(check_btn)

        outer.addWidget(check_group)

        # ── 2. 起飛設定 ──────────────────────────────────────
        takeoff_group = QGroupBox("起飛設定")
        takeoff_layout = QFormLayout(takeoff_group)

        # 起飛方向（跑道方向）
        takeoff_bearing_layout = QHBoxLayout()
        self.fw_takeoff_bearing_spin = QDoubleSpinBox()
        self.fw_takeoff_bearing_spin.setRange(0.0, 359.9)
        self.fw_takeoff_bearing_spin.setValue(self.parameters['fw_takeoff_bearing'])
        self.fw_takeoff_bearing_spin.setSuffix("°")
        self.fw_takeoff_bearing_spin.setDecimals(1)
        self.fw_takeoff_bearing_spin.setToolTip(
            "跑道方向（度，0=北，90=東，180=南，270=西）\n"
            "建議設定為逆風方向"
        )
        self.fw_takeoff_bearing_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_takeoff_bearing', v)
        )
        self.fw_takeoff_bearing_label = QLabel("")
        self.fw_takeoff_bearing_label.setStyleSheet("color: #2196F3; font-size: 10px; min-width: 30px;")
        self.fw_takeoff_bearing_spin.valueChanged.connect(
            lambda v: self.fw_takeoff_bearing_label.setText(self._bearing_to_compass(v))
        )
        self.fw_takeoff_bearing_label.setText(
            self._bearing_to_compass(self.parameters['fw_takeoff_bearing'])
        )
        takeoff_bearing_layout.addWidget(self.fw_takeoff_bearing_spin)
        takeoff_bearing_layout.addWidget(self.fw_takeoff_bearing_label)
        takeoff_layout.addRow("起飛方向:", takeoff_bearing_layout)

        # 跑道長度
        self.fw_runway_spin = QDoubleSpinBox()
        self.fw_runway_spin.setRange(20.0, 500.0)
        self.fw_runway_spin.setValue(self.parameters['fw_runway_length'])
        self.fw_runway_spin.setSuffix(" m")
        self.fw_runway_spin.setDecimals(0)
        self.fw_runway_spin.setToolTip(
            "起飛滑跑距離（公尺）\n"
            "衝浪者建議: 50m 以上"
        )
        self.fw_runway_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_runway_length', v)
        )
        takeoff_layout.addRow("跑道長度:", self.fw_runway_spin)

        outer.addWidget(takeoff_group)

        # ── 3. 五邊進場降落設定 ───────────────────────────────
        land_group = QGroupBox("五邊進場降落設定")
        land_layout = QFormLayout(land_group)

        # 降落進場方向
        land_bearing_layout = QHBoxLayout()
        self.fw_landing_bearing_spin = QDoubleSpinBox()
        self.fw_landing_bearing_spin.setRange(0.0, 359.9)
        self.fw_landing_bearing_spin.setValue(self.parameters['fw_landing_bearing'])
        self.fw_landing_bearing_spin.setSuffix("°")
        self.fw_landing_bearing_spin.setDecimals(1)
        self.fw_landing_bearing_spin.setToolTip(
            "飛機降落時的飛行方向（進場方向）\n"
            "通常與起飛方向相反或逆風"
        )
        self.fw_landing_bearing_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_landing_bearing', v)
        )
        self.fw_landing_bearing_label = QLabel("")
        self.fw_landing_bearing_label.setStyleSheet("color: #2196F3; font-size: 10px; min-width: 30px;")
        self.fw_landing_bearing_spin.valueChanged.connect(
            lambda v: self.fw_landing_bearing_label.setText(self._bearing_to_compass(v))
        )
        self.fw_landing_bearing_label.setText(
            self._bearing_to_compass(self.parameters['fw_landing_bearing'])
        )
        land_bearing_layout.addWidget(self.fw_landing_bearing_spin)
        land_bearing_layout.addWidget(self.fw_landing_bearing_label)
        land_layout.addRow("進場方向:", land_bearing_layout)

        # 五邊飛行高度
        self.fw_pattern_alt_spin = QDoubleSpinBox()
        self.fw_pattern_alt_spin.setRange(30.0, 500.0)
        self.fw_pattern_alt_spin.setValue(self.parameters['fw_pattern_alt'])
        self.fw_pattern_alt_spin.setSuffix(" m")
        self.fw_pattern_alt_spin.setDecimals(0)
        self.fw_pattern_alt_spin.setToolTip("五邊飛行高度 AGL（衝浪者建議: 80m）")
        self.fw_pattern_alt_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_pattern_alt', v)
        )
        land_layout.addRow("五邊高度:", self.fw_pattern_alt_spin)

        # 下風邊側偏距離
        self.fw_downwind_offset_spin = QDoubleSpinBox()
        self.fw_downwind_offset_spin.setRange(50.0, 1000.0)
        self.fw_downwind_offset_spin.setValue(self.parameters['fw_downwind_offset'])
        self.fw_downwind_offset_spin.setSuffix(" m")
        self.fw_downwind_offset_spin.setDecimals(0)
        self.fw_downwind_offset_spin.setToolTip(
            "下風邊與跑道的側向偏移距離（公尺）\n"
            "衝浪者建議: 200m（右手模式）"
        )
        self.fw_downwind_offset_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_downwind_offset', v)
        )
        land_layout.addRow("側偏距離:", self.fw_downwind_offset_spin)

        # 下風邊腿長
        self.fw_pattern_leg_spin = QDoubleSpinBox()
        self.fw_pattern_leg_spin.setRange(100.0, 2000.0)
        self.fw_pattern_leg_spin.setValue(self.parameters['fw_pattern_leg'])
        self.fw_pattern_leg_spin.setSuffix(" m")
        self.fw_pattern_leg_spin.setDecimals(0)
        self.fw_pattern_leg_spin.setToolTip("下風邊腿長（公尺）")
        self.fw_pattern_leg_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_pattern_leg', v)
        )
        land_layout.addRow("下風邊腿長:", self.fw_pattern_leg_spin)

        # 最終進場距離
        self.fw_final_dist_spin = QDoubleSpinBox()
        self.fw_final_dist_spin.setRange(100.0, 2000.0)
        self.fw_final_dist_spin.setValue(self.parameters['fw_final_dist'])
        self.fw_final_dist_spin.setSuffix(" m")
        self.fw_final_dist_spin.setDecimals(0)
        self.fw_final_dist_spin.setToolTip(
            "最終進場起始距離（Final Approach Fix 到跑道頭的距離）"
        )
        self.fw_final_dist_spin.valueChanged.connect(
            lambda v: self.update_parameter('fw_final_dist', v)
        )
        land_layout.addRow("最終進場距:", self.fw_final_dist_spin)

        outer.addWidget(land_group)

        # ── 4. 固定翼掃描模式說明 ─────────────────────────────
        hint = QLabel(
            "固定翼建議使用「螺旋掃描」或「同心圓掃描」。\n"
            "路徑已自動考慮最小轉彎半徑，若轉彎半徑不足\n"
            "將顯示警告並調整掃描間距。"
        )
        hint.setStyleSheet(
            "background-color: rgba(255,152,0,0.1);"
            "color: #FF9800; padding: 8px; border-radius: 4px;"
            "font-size: 10px;"
        )
        hint.setWordWrap(True)
        outer.addWidget(hint)

        # 連接 turn_radius 和 speed 變更到即時檢測
        # （在 create_flight_parameters 建立後由 init_ui 串接）

        return group

    @staticmethod
    def _bearing_to_compass(bearing_deg: float) -> str:
        """將方位角轉換為羅盤文字"""
        dirs = ['北', '東北', '東', '東南', '南', '西南', '西', '西北', '北']
        idx = int((bearing_deg + 22.5) / 45) % 8
        return dirs[idx]

    def _update_turn_radius_check(self):
        """
        即時更新轉彎半徑合規性狀態

        讀取當前的 turn_radius 和 speed，
        查詢機體規格（衝浪者 / Generic），
        更新 fw_status_dot, fw_status_summary, fw_status_detail 的顏色與文字。
        """
        if not hasattr(self, 'fw_status_dot'):
            return

        from core.global_planner.fixed_wing_planner import TurnRadiusChecker

        turn_r = self.parameters.get('turn_radius', 50.0)
        speed  = self.parameters.get('speed', 18.0)
        model  = self.parameters.get('vehicle_model', '衝浪者 (Surfer)')

        result = TurnRadiusChecker.check(turn_r, speed, model)

        color = result['color']
        status_icons = {'ok': '●', 'warning': '▲', 'critical': '✖'}
        icon = status_icons.get(result['status'], '●')

        self.fw_status_dot.setText(icon)
        self.fw_status_dot.setStyleSheet(f"color: {color}; font-size: 18px; font-weight: bold;")

        # 摘要
        status_text = {'ok': '符合規格', 'warning': '接近極限', 'critical': '超出極限！'}
        summary = f"{status_text.get(result['status'], '')}  |  轉彎半徑: {turn_r:.0f} m"
        self.fw_status_summary.setText(summary)
        self.fw_status_summary.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 11px;")

        # 詳細說明
        detail = (
            f"理論所需傾斜角: {result['required_bank']:.1f}°  "
            f"(機體上限: {result['max_bank']:.0f}°)\n"
            f"機體最小轉彎半徑: {result['min_radius']:.0f} m  "
            f"@速度 {speed:.1f} m/s"
        )
        self.fw_status_detail.setText(detail)
        self.fw_status_detail.setStyleSheet(f"color: {color}; font-size: 10px;")

        # 同步更新「最大傾斜角 → 最小半徑」顯示
        self._refresh_min_radius_label()

        logger.info(
            f"轉彎半徑檢測: 半徑={turn_r:.0f}m, 速度={speed:.1f}m/s, "
            f"狀態={result['status']}, 傾斜角={result['required_bank']:.1f}°"
        )

    def _refresh_min_radius_label(self):
        """依目前「最大傾斜角」和「速度」更新最小半徑顯示標籤"""
        import math
        if not hasattr(self, 'fw_min_radius_label'):
            return
        bank_deg = self.parameters.get('fw_max_bank_deg', 45.0)
        speed    = self.parameters.get('speed', 18.0)
        if bank_deg <= 0:
            return
        tan_b = math.tan(math.radians(bank_deg))
        if abs(tan_b) < 1e-9:
            return
        min_r = speed ** 2 / (9.81 * tan_b)
        self.fw_min_radius_label.setText(f"→ 最小半徑: {min_r:.0f} m")

    def _on_bank_angle_changed(self, _value: float = 0.0):
        """
        傾斜角改變時：
          1. 即時更新最小半徑顯示
          2. 觸發合規性檢測（以更新狀態燈）
        """
        self._refresh_min_radius_label()
        self._update_turn_radius_check()

    def _apply_min_radius_from_bank(self):
        """
        由目前「最大傾斜角 + 速度」計算最小轉彎半徑，
        並填入 turn_radius_spin（供路徑規劃使用）。

        公式（ArduPlane 同款）：
            R_min = V² / (g × tan(φ))
        """
        import math
        bank_deg = self.parameters.get('fw_max_bank_deg', 45.0)
        speed    = self.parameters.get('speed', 18.0)
        if bank_deg <= 0:
            return
        tan_b = math.tan(math.radians(bank_deg))
        if abs(tan_b) < 1e-9:
            return
        min_r = speed ** 2 / (9.81 * tan_b)
        if hasattr(self, 'turn_radius_spin'):
            self.turn_radius_spin.setValue(round(min_r, 1))
        logger.info(
            f"由傾斜角套用最小轉彎半徑: 傾斜角={bank_deg:.1f}°, "
            f"速度={speed:.1f}m/s → 最小半徑={min_r:.0f}m"
        )

    def create_survey_parameters(self):
        """創建測繪參數群組"""
        group = QGroupBox("測繪參數")
        layout = QFormLayout(group)
        
        # 掃描角度
        angle_layout = QHBoxLayout()
        self.angle_slider = QSlider(Qt.Orientation.Horizontal)
        self.angle_slider.setRange(-180, 180)
        self.angle_slider.setValue(int(self.parameters['angle']))
        self.angle_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.angle_slider.setTickInterval(30)
        self.angle_label = QLabel(f"{self.parameters['angle']:.0f}°")
        self.angle_slider.valueChanged.connect(self.on_angle_changed)
        angle_layout.addWidget(self.angle_slider)
        angle_layout.addWidget(self.angle_label)
        layout.addRow("掃描角度:", angle_layout)
        
        # 航線間距
        self.spacing_spin = QDoubleSpinBox()
        self.spacing_spin.setRange(settings.safety.min_spacing_m, 
                                settings.safety.max_spacing_m)
        self.spacing_spin.setValue(self.parameters['spacing'])
        self.spacing_spin.setSuffix(" m")
        self.spacing_spin.setDecimals(1)
        self.spacing_spin.valueChanged.connect(lambda v: self.update_parameter('spacing', v))
        layout.addRow("航線間距:", self.spacing_spin)
        
        # 子區域分割
        self.subdivision_combo = QComboBox()
        self.subdivision_combo.addItems([
            "1 (不分割)", 
            "2 區域", 
            "3 區域", 
            "4 區域 (2x2)", 
            "5 區域",
            "6 區域 (2x3)"
        ])
        self.subdivision_combo.setCurrentIndex(0)
        self.subdivision_combo.currentIndexChanged.connect(self.on_subdivision_changed)
        layout.addRow("區域分割:", self.subdivision_combo)
        
        return group
    
    def create_advanced_parameters(self):
        """創建進階參數群組"""
        group = QGroupBox("進階設定")
        layout = QVBoxLayout(group)
        
        # 減少重疊
        self.reduce_overlap_check = QCheckBox("減少重疊（互補掃描）")
        self.reduce_overlap_check.setChecked(self.parameters['reduce_overlap'])
        self.reduce_overlap_check.stateChanged.connect(
            lambda state: self.update_parameter('reduce_overlap', state == Qt.CheckState.Checked)
        )
        layout.addWidget(self.reduce_overlap_check)
        
        # 飛行模式
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("飛行模式:"))
        self.flight_mode_combo = QComboBox()
        self.flight_mode_combo.addItems(["同步飛行", "智能避撞"])
        self.flight_mode_combo.setCurrentIndex(1)  # 預設智能避撞
        self.flight_mode_combo.currentTextChanged.connect(self.on_flight_mode_changed)
        mode_layout.addWidget(self.flight_mode_combo)
        layout.addLayout(mode_layout)
        
        # 安全距離顯示（只讀）
        safety_layout = QHBoxLayout()
        safety_layout.addWidget(QLabel("安全距離:"))
        safety_label = QLabel(f"{settings.safety.default_safety_distance_m} m")
        safety_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        safety_layout.addWidget(safety_label)
        safety_layout.addStretch()
        layout.addLayout(safety_layout)
        
        return group
    
    def on_angle_changed(self, value):
        """處理角度變更"""
        self.angle_label.setText(f"{value}°")
        self.update_parameter('angle', float(value))
    
    def on_subdivision_changed(self, index):
        """處理分割數量變更"""
        subdivisions = index + 1  # 1, 2, 3, 4
        self.update_parameter('subdivisions', subdivisions)
        self._rebuild_region_gaps(subdivisions)
    
    def on_flight_mode_changed(self, text):
        """處理飛行模式變更"""
        mode = 'smart_collision' if text == "智能避撞" else 'synchronous'
        self.update_parameter('flight_mode', mode)

    @staticmethod
    def _heading_to_compass(angle: int) -> str:
        """將角度轉換為羅盤方向文字"""
        dirs = ['北', '東北', '東', '東南', '南', '西南', '西', '西北']
        return dirs[round(angle / 45) % 8]

    def on_lock_heading_changed(self, state):
        """處理 Heading Lock 勾選狀態變更"""
        locked = (state == Qt.CheckState.Checked.value)
        self.heading_row_label.setVisible(locked)
        self.heading_container.setVisible(locked)
        self.update_parameter('lock_heading', locked)
        logger.info(f"Heading Lock: {'啟用' if locked else '停用'}")

    def on_heading_changed(self, value: int):
        """處理機頭方向滑條變更"""
        compass = self._heading_to_compass(value)
        self.heading_value_label.setText(f"{value}° {compass}")
        self.update_parameter('heading_angle', float(value))
    
    def update_parameter(self, key: str, value):
        """
        更新參數並發送信號
        
        參數:
            key: 參數名稱
            value: 參數值
        """
        self.parameters[key] = value
        self.parameters_changed.emit({key: value})
        logger.debug(f"參數更新: {key} = {value}")
    
    def get_parameters(self):
        """
        獲取所有參數
        
        返回:
            參數字典
        """
        return self.parameters.copy()
    
    def set_parameters(self, params: dict):
        """
        設置參數
        
        參數:
            params: 參數字典
        """
        for key, value in params.items():
            if key in self.parameters:
                self.parameters[key] = value
                
                # 更新 UI
                if key == 'altitude':
                    self.altitude_spin.setValue(value)
                elif key == 'speed':
                    self.speed_spin.setValue(value)
                elif key == 'angle':
                    self.angle_slider.setValue(int(value))
                elif key == 'spacing':
                    self.spacing_spin.setValue(value)
                elif key == 'yaw_speed':
                    self.yaw_speed_spin.setValue(value)
                elif key == 'subdivisions':
                    self.subdivision_combo.setCurrentIndex(value - 1)

                elif key == 'reduce_overlap':
                    self.reduce_overlap_check.setChecked(value)
                elif key == 'flight_mode':
                    index = 1 if value == 'smart_collision' else 0
                    self.flight_mode_combo.setCurrentIndex(index)
                elif key == 'turn_radius':
                    self.turn_radius_spin.setValue(value)
                elif key == 'fw_max_bank_deg':
                    if hasattr(self, 'fw_max_bank_spin'):
                        self.fw_max_bank_spin.setValue(value)
                elif key == 'lock_heading':
                    self.lock_heading_check.setChecked(bool(value))
                elif key == 'heading_angle':
                    self.heading_slider.setValue(int(value))

        logger.info("參數已設置")
    
    def reset_to_default(self):
        """重置為預設參數"""
        default_params = {
            'altitude': 10.0,
            'speed': 3.0,
            'angle': 0.0,
            'spacing': 20.0,
            'yaw_speed': 60.0,
            'subdivisions': 1,
            'region_spacing': 3.0,
            'reduce_overlap': True,
            'flight_mode': 'smart_collision',
            'turn_radius': 50.0,
            'lock_heading': False,
            'heading_angle': 0.0,
        }

        self.set_parameters(default_params)
        self.parameters_changed.emit(default_params)
        logger.info("參數已重置為預設值")
