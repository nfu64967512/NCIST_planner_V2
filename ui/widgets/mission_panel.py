"""
任務面板模組
提供任務預覽、匯出、清除等操作界面
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QTextEdit, QProgressBar
)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont

from utils.logger import get_logger

# 獲取日誌實例
logger = get_logger()


class MissionPanel(QWidget):
    """
    任務面板
    
    提供任務相關操作的控制界面
    """
    
    # 信號定義
    preview_requested = pyqtSignal()  # 預覽請求
    export_requested = pyqtSignal()   # 匯出請求
    clear_requested = pyqtSignal()    # 清除請求
    
    def __init__(self, parent=None):
        """初始化任務面板"""
        super().__init__(parent)
        
        # 初始化變數
        self.mission_stats = {}
        
        # 建立 UI
        self.init_ui()
        
        logger.info("任務面板初始化完成")
    
    def init_ui(self):
        """初始化 UI"""
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        
        # 操作按鈕群組
        operation_group = self.create_operation_buttons()
        layout.addWidget(operation_group)
        
        # 任務資訊群組
        info_group = self.create_mission_info()
        layout.addWidget(info_group)
        
        # 添加彈性空間
        layout.addStretch()
    
    def create_operation_buttons(self):
        """創建操作按鈕群組"""
        group = QGroupBox("任務操作")
        layout = QVBoxLayout(group)
        layout.setSpacing(5)
        
        # 預覽路徑按鈕
        self.preview_btn = QPushButton("👁 預覽路徑")
        self.preview_btn.setMinimumHeight(40)
        self.preview_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
        """)
        self.preview_btn.clicked.connect(self.on_preview_clicked)
        layout.addWidget(self.preview_btn)
        
        # 匯出航點按鈕
        self.export_btn = QPushButton("📤 匯出航點")
        self.export_btn.setMinimumHeight(40)
        self.export_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #388E3C;
            }
            QPushButton:pressed {
                background-color: #1B5E20;
            }
        """)
        self.export_btn.clicked.connect(self.on_export_clicked)
        self.export_btn.setEnabled(False)  # 預設停用
        layout.addWidget(self.export_btn)
        
        # 分隔線
        layout.addSpacing(10)
        
        # 清除操作按鈕組
        clear_layout = QHBoxLayout()
        
        self.clear_paths_btn = QPushButton("清除路徑")
        self.clear_paths_btn.clicked.connect(lambda: self.on_clear_clicked('paths'))
        clear_layout.addWidget(self.clear_paths_btn)
        
        self.clear_corners_btn = QPushButton("清除邊界")
        self.clear_corners_btn.clicked.connect(lambda: self.on_clear_clicked('corners'))
        clear_layout.addWidget(self.clear_corners_btn)
        
        layout.addLayout(clear_layout)
        
        # 清除全部按鈕
        self.clear_all_btn = QPushButton("🗑 清除全部")
        self.clear_all_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF5722;
                color: white;
                border: none;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #E64A19;
            }
        """)
        self.clear_all_btn.clicked.connect(lambda: self.on_clear_clicked('all'))
        layout.addWidget(self.clear_all_btn)
        
        return group
    
    def create_mission_info(self):
        """創建任務資訊群組"""
        group = QGroupBox("任務資訊")
        layout = QVBoxLayout(group)
        
        # 資訊文字區域
        self.info_text = QTextEdit()
        self.info_text.setReadOnly(True)
        self.info_text.setMaximumHeight(200)
        
        # 設置字體
        font = QFont("Consolas", 9)
        self.info_text.setFont(font)
        
        # 初始資訊
        self.update_info_display()
        
        layout.addWidget(self.info_text)
        
        # 進度條（用於顯示任務進度）
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)  # 預設隱藏
        layout.addWidget(self.progress_bar)
        
        return group
    
    def on_preview_clicked(self):
        """處理預覽按鈕點擊"""
        self.preview_requested.emit()
        logger.info("請求預覽路徑")
    
    def on_export_clicked(self):
        """處理匯出按鈕點擊"""
        self.export_requested.emit()
        logger.info("請求匯出航點")
    
    def on_clear_clicked(self, clear_type: str):
        """
        處理清除按鈕點擊
        
        參數:
            clear_type: 清除類型 ('paths', 'corners', 'all')
        """
        self.clear_requested.emit()
        logger.info(f"請求清除: {clear_type}")
    
    def update_mission_stats(self, stats: dict):
        """
        更新任務統計資訊
        
        參數:
            stats: 統計資訊字典
        """
        self.mission_stats = stats
        self.update_info_display()
        
        # 如果有統計資訊，啟用匯出按鈕
        if stats:
            self.export_btn.setEnabled(True)
        
        logger.debug(f"更新任務統計: {stats}")
    
    def update_info_display(self):
        """更新資訊顯示"""
        if not self.mission_stats:
            info_text = """
╔══════════════════════════════════╗
║         尚無任務資訊             ║
╠══════════════════════════════════╣
║ 1. 在地圖上點擊設置邊界點        ║
║ 2. 調整飛行參數                  ║
║ 3. 點擊"預覽路徑"生成任務       ║
║ 4. 點擊"匯出航點"儲存檔案       ║
╚══════════════════════════════════╝
            """
        else:
            # 格式化統計資訊
            waypoint_count = self.mission_stats.get('waypoint_count', 0)
            total_distance = self.mission_stats.get('total_distance', 0.0)
            estimated_time = self.mission_stats.get('estimated_time', 0.0)
            area = self.mission_stats.get('area', 0.0)
            regions = self.mission_stats.get('regions', 1)
            
            info_text = f"""
╔══════════════════════════════════╗
║           任務統計               ║
╠══════════════════════════════════╣
║ 航點數量: {waypoint_count:>4} 個           ║
║ 總飛行距離: {total_distance:>8.1f} m       ║
║ 預估時間: {estimated_time/60:>6.1f} 分鐘    ║
║ 測繪面積: {area:>8.1f} m²       ║
║ 子區域數: {regions:>2} 個              ║
╚══════════════════════════════════╝
            """
        
        self.info_text.setText(info_text)
    
    def show_progress(self, visible: bool, value: int = 0, text: str = ""):
        """
        顯示/隱藏進度條
        
        參數:
            visible: 是否顯示
            value: 進度值 (0-100)
            text: 進度文字
        """
        self.progress_bar.setVisible(visible)
        
        if visible:
            self.progress_bar.setValue(value)
            if text:
                self.progress_bar.setFormat(text + " %p%")
        
        logger.debug(f"進度條: visible={visible}, value={value}, text={text}")
    
    def set_buttons_enabled(self, enabled: bool):
        """
        設置按鈕啟用狀態
        
        參數:
            enabled: 是否啟用
        """
        self.preview_btn.setEnabled(enabled)
        self.clear_paths_btn.setEnabled(enabled)
        self.clear_corners_btn.setEnabled(enabled)
        self.clear_all_btn.setEnabled(enabled)
        
        logger.debug(f"按鈕啟用狀態: {enabled}")
    
    def reset(self):
        """重置面板"""
        self.mission_stats = {}
        self.update_info_display()
        self.export_btn.setEnabled(False)
        self.progress_bar.setVisible(False)
        logger.info("任務面板已重置")
