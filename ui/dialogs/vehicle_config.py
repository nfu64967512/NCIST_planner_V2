"""
飛行器配置對話框
提供飛行器參數選擇和設定
"""

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QComboBox, QDoubleSpinBox, QPushButton,
    QGroupBox, QTextEdit, QMessageBox, QTabWidget,
    QWidget
)
from PyQt6.QtCore import Qt
from pathlib import Path

from utils.file_io import read_yaml
from utils.logger import get_logger

logger = get_logger()


class VehicleConfigDialog(QDialog):
    """
    飛行器配置對話框
    
    提供飛行器選擇和參數設定
    """
    
    def __init__(self, parent=None):
        """初始化對話框"""
        super().__init__(parent)
        
        self.setWindowTitle("飛行器配置")
        self.setMinimumSize(700, 800)
        
        # 載入飛行器配置檔案
        self.vehicle_profiles = self.load_vehicle_profiles()
        
        # 當前飛行器
        self.current_vehicle = None
        
        # 建立 UI
        self.init_ui()
        
        # 載入預設飛行器
        self.load_default_vehicle()
        
        logger.info("飛行器配置對話框初始化完成")
    
    def load_vehicle_profiles(self):
        """載入飛行器配置檔案"""
        try:
            config_path = Path(__file__).parent.parent.parent / "config" / "vehicle_profiles.yaml"
            
            if config_path.exists():
                profiles = read_yaml(str(config_path))
                logger.info(f"載入飛行器配置: {config_path}")
                return profiles
            else:
                logger.warning(f"飛行器配置檔案不存在: {config_path}")
                return {}
                
        except Exception as e:
            logger.error(f"載入飛行器配置失敗: {e}")
            return {}
    
    def init_ui(self):
        """初始化 UI"""
        layout = QVBoxLayout(self)
        
        # 分頁控制
        tabs = QTabWidget()
        
        # 飛行器選擇頁籤
        selection_tab = self.create_selection_tab()
        tabs.addTab(selection_tab, "飛行器選擇")
        
        # 性能參數頁籤
        performance_tab = self.create_performance_tab()
        tabs.addTab(performance_tab, "性能參數")
        
        # 飛行限制頁籤
        limits_tab = self.create_limits_tab()
        tabs.addTab(limits_tab, "飛行限制")
        
        layout.addWidget(tabs)
        
        # 按鈕
        button_layout = QHBoxLayout()
        
        apply_btn = QPushButton("✓ 應用設定")
        apply_btn.clicked.connect(self.apply_settings)
        button_layout.addWidget(apply_btn)
        
        reset_btn = QPushButton("↻ 重置")
        reset_btn.clicked.connect(self.reset_to_default)
        button_layout.addWidget(reset_btn)
        
        close_btn = QPushButton("關閉")
        close_btn.clicked.connect(self.close)
        button_layout.addWidget(close_btn)
        
        layout.addLayout(button_layout)
    
    def create_selection_tab(self):
        """創建飛行器選擇頁籤"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 飛行器類型選擇
        type_group = QGroupBox("飛行器類型")
        type_layout = QFormLayout(type_group)
        
        self.vehicle_type_combo = QComboBox()
        self.vehicle_type_combo.addItems(["多旋翼", "固定翼", "VTOL"])
        self.vehicle_type_combo.currentTextChanged.connect(self.on_vehicle_type_changed)
        type_layout.addRow("類型:", self.vehicle_type_combo)
        
        layout.addWidget(type_group)
        
        # 飛行器型號選擇
        model_group = QGroupBox("飛行器型號")
        model_layout = QFormLayout(model_group)
        
        self.vehicle_model_combo = QComboBox()
        self.vehicle_model_combo.currentTextChanged.connect(self.on_vehicle_model_changed)
        model_layout.addRow("型號:", self.vehicle_model_combo)
        
        layout.addWidget(model_group)
        
        # 飛行器資訊顯示
        info_group = QGroupBox("飛行器資訊")
        info_layout = QVBoxLayout(info_group)
        
        self.vehicle_info_text = QTextEdit()
        self.vehicle_info_text.setReadOnly(True)
        self.vehicle_info_text.setMaximumHeight(400)
        
        info_layout.addWidget(self.vehicle_info_text)
        
        layout.addWidget(info_group)
        
        layout.addStretch()
        
        return widget
    
    def create_performance_tab(self):
        """創建性能參數頁籤"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 速度性能
        speed_group = QGroupBox("速度性能")
        speed_layout = QFormLayout(speed_group)
        
        self.max_speed_spin = QDoubleSpinBox()
        self.max_speed_spin.setRange(1.0, 50.0)
        self.max_speed_spin.setSuffix(" m/s")
        speed_layout.addRow("最大速度:", self.max_speed_spin)
        
        self.cruise_speed_spin = QDoubleSpinBox()
        self.cruise_speed_spin.setRange(1.0, 50.0)
        self.cruise_speed_spin.setSuffix(" m/s")
        speed_layout.addRow("巡航速度:", self.cruise_speed_spin)
        
        layout.addWidget(speed_group)
        
        # 動力性能
        dynamics_group = QGroupBox("動力性能")
        dynamics_layout = QFormLayout(dynamics_group)
        
        self.max_accel_spin = QDoubleSpinBox()
        self.max_accel_spin.setRange(0.5, 10.0)
        self.max_accel_spin.setSuffix(" m/s²")
        dynamics_layout.addRow("最大加速度:", self.max_accel_spin)
        
        self.max_yaw_rate_spin = QDoubleSpinBox()
        self.max_yaw_rate_spin.setRange(10.0, 360.0)
        self.max_yaw_rate_spin.setSuffix(" °/s")
        dynamics_layout.addRow("最大轉向速度:", self.max_yaw_rate_spin)
        
        layout.addWidget(dynamics_group)
        
        # 續航力
        endurance_group = QGroupBox("續航力")
        endurance_layout = QFormLayout(endurance_group)
        
        self.flight_time_spin = QDoubleSpinBox()
        self.flight_time_spin.setRange(5.0, 300.0)
        self.flight_time_spin.setSuffix(" 分鐘")
        endurance_layout.addRow("最大飛行時間:", self.flight_time_spin)
        
        self.battery_capacity_spin = QDoubleSpinBox()
        self.battery_capacity_spin.setRange(1000.0, 50000.0)
        self.battery_capacity_spin.setSuffix(" mAh")
        endurance_layout.addRow("電池容量:", self.battery_capacity_spin)
        
        layout.addWidget(endurance_group)
        
        layout.addStretch()
        
        return widget
    
    def create_limits_tab(self):
        """創建飛行限制頁籤"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 高度限制
        altitude_group = QGroupBox("高度限制")
        altitude_layout = QFormLayout(altitude_group)
        
        self.max_altitude_spin = QDoubleSpinBox()
        self.max_altitude_spin.setRange(10.0, 10000.0)
        self.max_altitude_spin.setSuffix(" m")
        altitude_layout.addRow("最大飛行高度:", self.max_altitude_spin)
        
        self.service_ceiling_spin = QDoubleSpinBox()
        self.service_ceiling_spin.setRange(10.0, 10000.0)
        self.service_ceiling_spin.setSuffix(" m")
        altitude_layout.addRow("實用升限:", self.service_ceiling_spin)
        
        layout.addWidget(altitude_group)
        
        # 環境限制
        environment_group = QGroupBox("環境限制")
        environment_layout = QFormLayout(environment_group)
        
        self.max_wind_spin = QDoubleSpinBox()
        self.max_wind_spin.setRange(0.0, 30.0)
        self.max_wind_spin.setSuffix(" m/s")
        environment_layout.addRow("最大抗風能力:", self.max_wind_spin)
        
        self.temp_min_spin = QDoubleSpinBox()
        self.temp_min_spin.setRange(-50.0, 50.0)
        self.temp_min_spin.setSuffix(" °C")
        environment_layout.addRow("最低工作溫度:", self.temp_min_spin)
        
        self.temp_max_spin = QDoubleSpinBox()
        self.temp_max_spin.setRange(-50.0, 50.0)
        self.temp_max_spin.setSuffix(" °C")
        environment_layout.addRow("最高工作溫度:", self.temp_max_spin)
        
        layout.addWidget(environment_group)
        
        layout.addStretch()
        
        return widget
    
    def load_default_vehicle(self):
        """載入預設飛行器"""
        # 預設選擇多旋翼
        self.vehicle_type_combo.setCurrentText("多旋翼")
        self.on_vehicle_type_changed("多旋翼")
    
    def on_vehicle_type_changed(self, vehicle_type: str):
        """處理飛行器類型變更"""
        # 清空型號列表
        self.vehicle_model_combo.clear()
        
        # 根據類型載入對應型號
        type_key_map = {
            "多旋翼": "multirotor",
            "固定翼": "fixed_wing",
            "VTOL": "vtol"
        }
        
        type_key = type_key_map.get(vehicle_type)
        
        if type_key and type_key in self.vehicle_profiles:
            vehicles = self.vehicle_profiles[type_key]
            
            # 添加型號到下拉選單
            for vehicle_key, vehicle_data in vehicles.items():
                name = vehicle_data.get('name', vehicle_key)
                self.vehicle_model_combo.addItem(name, vehicle_key)
            
            logger.info(f"載入飛行器類型: {vehicle_type}")
    
    def on_vehicle_model_changed(self, model_name: str):
        """處理飛行器型號變更"""
        # 獲取當前飛行器類型
        vehicle_type = self.vehicle_type_combo.currentText()
        type_key_map = {
            "多旋翼": "multirotor",
            "固定翼": "fixed_wing",
            "VTOL": "vtol"
        }
        
        type_key = type_key_map.get(vehicle_type)
        
        if not type_key or type_key not in self.vehicle_profiles:
            return
        
        # 獲取飛行器配置鍵
        vehicle_key = self.vehicle_model_combo.currentData()
        
        if not vehicle_key:
            return
        
        # 載入飛行器配置
        vehicle_data = self.vehicle_profiles[type_key].get(vehicle_key)
        
        if vehicle_data:
            self.current_vehicle = vehicle_data
            self.display_vehicle_info(vehicle_data)
            self.load_vehicle_parameters(vehicle_data)
            
            logger.info(f"載入飛行器: {model_name}")
    
    def display_vehicle_info(self, vehicle_data: dict):
        """顯示飛行器資訊"""
        name = vehicle_data.get('name', 'Unknown')
        manufacturer = vehicle_data.get('manufacturer', 'Unknown')
        vehicle_type = vehicle_data.get('type', 'Unknown')
        
        info_text = f"""
╔══════════════════════════════════════════╗
║           飛行器資訊                     ║
╠══════════════════════════════════════════╣
║ 型號: {name:30} ║
║ 製造商: {manufacturer:28} ║
║ 類型: {vehicle_type:30} ║
╚══════════════════════════════════════════╝

基本參數:
  質量: {vehicle_data.get('mass_kg', '--')} kg
  最大載重: {vehicle_data.get('max_payload_kg', '--')} kg

性能參數:
  最大速度: {vehicle_data.get('max_speed_mps', '--')} m/s
  巡航速度: {vehicle_data.get('cruise_speed_mps', '--')} m/s
  最大飛行時間: {vehicle_data.get('max_flight_time_min', '--')} 分鐘

飛行限制:
  最大高度: {vehicle_data.get('max_altitude_m', '--')} m
  最大抗風: {vehicle_data.get('max_wind_speed_mps', '--')} m/s
        """
        
        self.vehicle_info_text.setText(info_text)
    
    def load_vehicle_parameters(self, vehicle_data: dict):
        """載入飛行器參數到控制項"""
        # 性能參數
        self.max_speed_spin.setValue(vehicle_data.get('max_speed_mps', 15.0))
        self.cruise_speed_spin.setValue(vehicle_data.get('cruise_speed_mps', 10.0))
        self.max_accel_spin.setValue(vehicle_data.get('max_acceleration_mps2', 3.0))
        self.max_yaw_rate_spin.setValue(vehicle_data.get('max_yaw_rate_dps', 120.0))
        
        # 續航力
        self.flight_time_spin.setValue(vehicle_data.get('max_flight_time_min', 30.0))
        self.battery_capacity_spin.setValue(vehicle_data.get('battery_capacity_mah', 5000.0))
        
        # 飛行限制
        self.max_altitude_spin.setValue(vehicle_data.get('max_altitude_m', 500.0))
        self.service_ceiling_spin.setValue(vehicle_data.get('service_ceiling_m', 1000.0))
        self.max_wind_spin.setValue(vehicle_data.get('max_wind_speed_mps', 10.0))
        
        # 溫度限制（如果有）
        self.temp_min_spin.setValue(vehicle_data.get('operating_temp_min_c', -10.0))
        self.temp_max_spin.setValue(vehicle_data.get('operating_temp_max_c', 40.0))
    
    def apply_settings(self):
        """應用設定"""
        if not self.current_vehicle:
            QMessageBox.warning(self, "錯誤", "請先選擇飛行器")
            return
        
        # TODO: 將設定應用到主視窗
        QMessageBox.information(self, "成功", "飛行器設定已應用")
        logger.info("飛行器設定已應用")
        self.accept()
    
    def reset_to_default(self):
        """重置為預設值"""
        if self.current_vehicle:
            self.load_vehicle_parameters(self.current_vehicle)
            logger.info("已重置為預設值")
