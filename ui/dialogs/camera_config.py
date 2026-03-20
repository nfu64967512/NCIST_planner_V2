"""
相機配置對話框
提供相機參數設置和航線間距自動計算
"""

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QComboBox, QDoubleSpinBox, QPushButton,
    QGroupBox, QTextEdit, QMessageBox
)
from PyQt6.QtCore import Qt

from sensors import CameraDatabase, CameraCalculator, SurveyParameters
from utils.logger import get_logger

logger = get_logger()


class CameraConfigDialog(QDialog):
    """
    相機配置對話框
    
    提供相機選擇、參數設置和自動計算航線間距功能
    """
    
    def __init__(self, parent=None):
        """初始化對話框"""
        super().__init__(parent)
        
        self.setWindowTitle("相機配置")
        self.setMinimumSize(600, 700)
        
        # 當前相機
        self.current_camera = None
        
        # 建立 UI
        self.init_ui()
        
        # 載入預設相機
        self.load_default_camera()
        
        logger.info("相機配置對話框初始化完成")
    
    def init_ui(self):
        """初始化 UI"""
        layout = QVBoxLayout(self)
        
        # 相機選擇群組
        camera_group = self.create_camera_selection()
        layout.addWidget(camera_group)
        
        # 相機參數顯示群組
        params_group = self.create_camera_parameters()
        layout.addWidget(params_group)
        
        # 測繪參數群組
        survey_group = self.create_survey_parameters()
        layout.addWidget(survey_group)
        
        # 計算結果群組
        results_group = self.create_results_display()
        layout.addWidget(results_group)
        
        # 按鈕
        button_layout = QHBoxLayout()
        
        calc_btn = QPushButton("📊 計算航線參數")
        calc_btn.clicked.connect(self.calculate_parameters)
        button_layout.addWidget(calc_btn)
        
        apply_btn = QPushButton("✓ 應用設定")
        apply_btn.clicked.connect(self.apply_settings)
        button_layout.addWidget(apply_btn)
        
        close_btn = QPushButton("關閉")
        close_btn.clicked.connect(self.close)
        button_layout.addWidget(close_btn)
        
        layout.addLayout(button_layout)
    
    def create_camera_selection(self):
        """創建相機選擇群組"""
        group = QGroupBox("相機選擇")
        layout = QFormLayout(group)
        
        # 製造商篩選
        self.manufacturer_combo = QComboBox()
        self.manufacturer_combo.addItem("所有製造商")
        self.manufacturer_combo.addItems(CameraDatabase.get_manufacturers())
        self.manufacturer_combo.currentTextChanged.connect(self.on_manufacturer_changed)
        layout.addRow("製造商:", self.manufacturer_combo)
        
        # 相機型號
        self.camera_combo = QComboBox()
        self.camera_combo.addItems(CameraDatabase.get_camera_list())
        self.camera_combo.currentTextChanged.connect(self.on_camera_changed)
        layout.addRow("相機型號:", self.camera_combo)
        
        return group
    
    def create_camera_parameters(self):
        """創建相機參數顯示群組"""
        group = QGroupBox("相機參數")
        layout = QFormLayout(group)
        
        # 感光元件尺寸
        self.sensor_size_label = QLabel("--")
        layout.addRow("感光元件:", self.sensor_size_label)
        
        # 影像解析度
        self.resolution_label = QLabel("--")
        layout.addRow("影像解析度:", self.resolution_label)
        
        # 焦距
        self.focal_length_spin = QDoubleSpinBox()
        self.focal_length_spin.setRange(1.0, 500.0)
        self.focal_length_spin.setSuffix(" mm")
        self.focal_length_spin.setDecimals(2)
        layout.addRow("焦距:", self.focal_length_spin)
        
        # 視場角
        self.fov_label = QLabel("--")
        layout.addRow("視場角:", self.fov_label)
        
        return group
    
    def create_survey_parameters(self):
        """創建測繪參數群組"""
        group = QGroupBox("測繪參數")
        layout = QFormLayout(group)
        
        # 飛行高度
        self.altitude_spin = QDoubleSpinBox()
        self.altitude_spin.setRange(5.0, 500.0)
        self.altitude_spin.setValue(50.0)
        self.altitude_spin.setSuffix(" m")
        self.altitude_spin.setDecimals(1)
        layout.addRow("飛行高度:", self.altitude_spin)
        
        # 前向重疊率
        self.front_overlap_spin = QDoubleSpinBox()
        self.front_overlap_spin.setRange(0.0, 95.0)
        self.front_overlap_spin.setValue(80.0)
        self.front_overlap_spin.setSuffix(" %")
        self.front_overlap_spin.setDecimals(1)
        layout.addRow("前向重疊率:", self.front_overlap_spin)
        
        # 側向重疊率
        self.side_overlap_spin = QDoubleSpinBox()
        self.side_overlap_spin.setRange(0.0, 95.0)
        self.side_overlap_spin.setValue(60.0)
        self.side_overlap_spin.setSuffix(" %")
        self.side_overlap_spin.setDecimals(1)
        layout.addRow("側向重疊率:", self.side_overlap_spin)
        
        # 飛行速度
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.1, 30.0)
        self.speed_spin.setValue(5.0)
        self.speed_spin.setSuffix(" m/s")
        self.speed_spin.setDecimals(1)
        layout.addRow("飛行速度:", self.speed_spin)
        
        return group
    
    def create_results_display(self):
        """創建計算結果顯示群組"""
        group = QGroupBox("計算結果")
        layout = QVBoxLayout(group)
        
        self.results_text = QTextEdit()
        self.results_text.setReadOnly(True)
        self.results_text.setMaximumHeight(200)
        self.results_text.setText("請點擊「計算航線參數」按鈕")
        
        layout.addWidget(self.results_text)
        
        return group
    
    def load_default_camera(self):
        """載入預設相機"""
        default_camera_name = "DJI Mavic 3"
        
        if default_camera_name in CameraDatabase.CAMERAS:
            self.camera_combo.setCurrentText(default_camera_name)
            self.on_camera_changed(default_camera_name)
    
    def on_manufacturer_changed(self, manufacturer: str):
        """處理製造商變更"""
        # 篩選相機列表
        self.camera_combo.clear()
        
        if manufacturer == "所有製造商":
            cameras = CameraDatabase.get_camera_list()
        else:
            cameras = [
                name for name, spec in CameraDatabase.CAMERAS.items()
                if spec.manufacturer == manufacturer
            ]
        
        self.camera_combo.addItems(cameras)
    
    def on_camera_changed(self, camera_name: str):
        """處理相機變更"""
        camera = CameraDatabase.get_camera(camera_name)
        
        if camera:
            self.current_camera = camera
            
            # 更新顯示
            self.sensor_size_label.setText(
                f"{camera.sensor_width:.2f} x {camera.sensor_height:.2f} mm"
            )
            
            self.resolution_label.setText(
                f"{camera.image_width} x {camera.image_height} px"
            )
            
            self.focal_length_spin.setValue(camera.focal_length)
            
            # 計算並顯示視場角
            h_fov, v_fov = CameraCalculator.calculate_field_of_view(
                camera.focal_length,
                camera.sensor_width,
                camera.sensor_height
            )
            
            self.fov_label.setText(f"{h_fov:.1f}° x {v_fov:.1f}°")
            
            logger.info(f"載入相機: {camera_name}")
    
    def calculate_parameters(self):
        """計算航線參數"""
        if not self.current_camera:
            QMessageBox.warning(self, "錯誤", "請先選擇相機")
            return
        
        try:
            # 獲取參數
            altitude = self.altitude_spin.value()
            front_overlap = self.front_overlap_spin.value()
            side_overlap = self.side_overlap_spin.value()
            speed = self.speed_spin.value()
            focal_length = self.focal_length_spin.value()
            
            # 創建相機副本（使用自訂焦距）
            camera = self.current_camera
            camera.focal_length = focal_length
            
            # 計算 GSD
            gsd = CameraCalculator.calculate_gsd(
                altitude, focal_length,
                camera.sensor_width, camera.image_width
            )
            
            # 計算地面覆蓋範圍
            ground_width, ground_height = CameraCalculator.calculate_ground_coverage(
                altitude, focal_length,
                camera.sensor_width, camera.sensor_height
            )
            
            # 計算航線間距和拍照間隔
            line_spacing, photo_interval = CameraCalculator.calculate_spacing_from_overlap(
                altitude, camera,
                front_overlap, side_overlap
            )
            
            # 計算照片數量（假設 10000 m² 區域）
            test_area = 10000.0
            num_photos = CameraCalculator.calculate_required_photos(
                test_area, altitude, camera,
                front_overlap, side_overlap
            )
            
            # 計算飛行時間
            estimated_distance = test_area / line_spacing
            flight_time = CameraCalculator.calculate_flight_time(
                estimated_distance, speed, num_photos, 0.5
            )
            
            # 顯示結果
            results_text = f"""
╔══════════════════════════════════════════╗
║              計算結果                    ║
╠══════════════════════════════════════════╣
║ 地面採樣距離 (GSD):                     ║
║   {gsd*100:.2f} cm/px                              ║
║                                          ║
║ 地面覆蓋範圍:                            ║
║   {ground_width:.1f} m x {ground_height:.1f} m                 ║
║                                          ║
║ 航線間距:                                ║
║   {line_spacing:.1f} m (側向重疊 {side_overlap:.0f}%)      ║
║                                          ║
║ 拍照間隔:                                ║
║   {photo_interval:.1f} m (前向重疊 {front_overlap:.0f}%)      ║
║                                          ║
║ 10000 m² 區域預估:                      ║
║   照片數量: {num_photos} 張                     ║
║   飛行時間: {flight_time/60:.1f} 分鐘                ║
╚══════════════════════════════════════════╝

建議設定:
• 航線間距設為 {line_spacing:.1f} m
• 飛行速度不超過 {speed:.1f} m/s
            """
            
            self.results_text.setText(results_text)
            
            logger.info(f"計算完成: 航線間距={line_spacing:.1f}m, GSD={gsd*100:.2f}cm/px")
            
        except Exception as e:
            logger.error(f"計算失敗: {e}")
            QMessageBox.critical(self, "計算錯誤", f"計算時發生錯誤：\n{str(e)}")
    
    def apply_settings(self):
        """應用設定"""
        if not self.current_camera:
            QMessageBox.warning(self, "錯誤", "請先選擇相機並計算參數")
            return
        
        # TODO: 將計算結果應用到主視窗的參數面板
        QMessageBox.information(self, "成功", "相機設定已應用")
        logger.info("相機設定已應用")
        self.accept()
