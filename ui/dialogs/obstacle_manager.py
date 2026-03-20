"""
障礙物管理對話框
提供障礙物的新增、編輯、刪除功能
"""

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QDoubleSpinBox, QPushButton, QGroupBox,
    QListWidget, QListWidgetItem, QMessageBox, QComboBox
)
from PyQt6.QtCore import Qt, pyqtSignal

from utils.logger import get_logger

logger = get_logger()


class ObstacleManagerDialog(QDialog):
    """
    障礙物管理對話框

    提供障礙物列表管理、新增、編輯、刪除功能
    """

    # 信號定義
    obstacles_changed = pyqtSignal(list)  # 障礙物變更信號

    def __init__(self, parent=None, obstacles=None):
        """
        初始化對話框

        參數:
            parent: 父視窗
            obstacles: 現有障礙物列表
        """
        super().__init__(parent)

        self.setWindowTitle("障礙物管理")
        self.setMinimumSize(500, 600)

        # 障礙物列表
        self.obstacles = obstacles.copy() if obstacles else []

        # 建立 UI
        self.init_ui()

        # 載入現有障礙物
        self.load_obstacles()

        logger.info("障礙物管理對話框初始化完成")

    def init_ui(self):
        """初始化 UI"""
        layout = QVBoxLayout(self)

        # 障礙物列表群組
        list_group = QGroupBox("障礙物列表")
        list_layout = QVBoxLayout(list_group)

        self.obstacle_list = QListWidget()
        self.obstacle_list.itemSelectionChanged.connect(self.on_selection_changed)
        list_layout.addWidget(self.obstacle_list)

        # 列表操作按鈕
        list_btn_layout = QHBoxLayout()

        self.edit_btn = QPushButton("編輯")
        self.edit_btn.setEnabled(False)
        self.edit_btn.clicked.connect(self.on_edit_obstacle)
        list_btn_layout.addWidget(self.edit_btn)

        self.delete_btn = QPushButton("刪除")
        self.delete_btn.setEnabled(False)
        self.delete_btn.clicked.connect(self.on_delete_obstacle)
        list_btn_layout.addWidget(self.delete_btn)

        self.clear_btn = QPushButton("清除全部")
        self.clear_btn.clicked.connect(self.on_clear_all)
        list_btn_layout.addWidget(self.clear_btn)

        list_layout.addLayout(list_btn_layout)
        layout.addWidget(list_group)

        # 新增障礙物群組
        add_group = QGroupBox("新增障礙物")
        add_layout = QFormLayout(add_group)

        # 障礙物類型
        self.type_combo = QComboBox()
        self.type_combo.addItems(["圓形", "矩形", "多邊形"])
        self.type_combo.currentIndexChanged.connect(self.on_type_changed)
        add_layout.addRow("類型:", self.type_combo)

        # 中心緯度
        self.lat_spin = QDoubleSpinBox()
        self.lat_spin.setRange(-90.0, 90.0)
        self.lat_spin.setDecimals(6)
        self.lat_spin.setValue(23.7027)
        add_layout.addRow("中心緯度:", self.lat_spin)

        # 中心經度
        self.lon_spin = QDoubleSpinBox()
        self.lon_spin.setRange(-180.0, 180.0)
        self.lon_spin.setDecimals(6)
        self.lon_spin.setValue(120.4193)
        add_layout.addRow("中心經度:", self.lon_spin)

        # 半徑（圓形）/ 寬度（矩形）
        self.radius_spin = QDoubleSpinBox()
        self.radius_spin.setRange(1.0, 1000.0)
        self.radius_spin.setValue(50.0)
        self.radius_spin.setSuffix(" m")
        self.radius_label = QLabel("半徑:")
        add_layout.addRow(self.radius_label, self.radius_spin)

        # 高度（矩形）
        self.height_spin = QDoubleSpinBox()
        self.height_spin.setRange(1.0, 1000.0)
        self.height_spin.setValue(50.0)
        self.height_spin.setSuffix(" m")
        self.height_label = QLabel("高度:")
        self.height_spin.setVisible(False)
        self.height_label.setVisible(False)
        add_layout.addRow(self.height_label, self.height_spin)

        # 障礙物高度（垂直）
        self.alt_spin = QDoubleSpinBox()
        self.alt_spin.setRange(0.0, 500.0)
        self.alt_spin.setValue(100.0)
        self.alt_spin.setSuffix(" m")
        add_layout.addRow("障礙物高度:", self.alt_spin)

        # 新增按鈕
        self.add_btn = QPushButton("➕ 新增障礙物")
        self.add_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.add_btn.clicked.connect(self.on_add_obstacle)
        add_layout.addRow("", self.add_btn)

        layout.addWidget(add_group)

        # 對話框按鈕
        btn_layout = QHBoxLayout()

        apply_btn = QPushButton("套用")
        apply_btn.clicked.connect(self.on_apply)
        btn_layout.addWidget(apply_btn)

        close_btn = QPushButton("關閉")
        close_btn.clicked.connect(self.close)
        btn_layout.addWidget(close_btn)

        layout.addLayout(btn_layout)

    def on_type_changed(self, index):
        """處理類型變更"""
        if index == 0:  # 圓形
            self.radius_label.setText("半徑:")
            self.height_spin.setVisible(False)
            self.height_label.setVisible(False)
        elif index == 1:  # 矩形
            self.radius_label.setText("寬度:")
            self.height_spin.setVisible(True)
            self.height_label.setVisible(True)
        else:  # 多邊形
            self.radius_label.setText("半徑:")
            self.height_spin.setVisible(False)
            self.height_label.setVisible(False)

    def load_obstacles(self):
        """載入障礙物到列表"""
        self.obstacle_list.clear()

        for i, obs in enumerate(self.obstacles):
            item_text = self._format_obstacle_text(i, obs)
            item = QListWidgetItem(item_text)
            item.setData(Qt.ItemDataRole.UserRole, i)
            self.obstacle_list.addItem(item)

    def _format_obstacle_text(self, index: int, obstacle: dict) -> str:
        """格式化障礙物文字"""
        obs_type = obstacle.get('type', 'circle')
        lat = obstacle.get('lat', 0)
        lon = obstacle.get('lon', 0)
        radius = obstacle.get('radius', 0)

        type_names = {'circle': '圓形', 'rectangle': '矩形', 'polygon': '多邊形'}
        type_name = type_names.get(obs_type, obs_type)

        return f"#{index+1} {type_name} - ({lat:.4f}, {lon:.4f}) R={radius}m"

    def on_selection_changed(self):
        """處理選擇變更"""
        has_selection = len(self.obstacle_list.selectedItems()) > 0
        self.edit_btn.setEnabled(has_selection)
        self.delete_btn.setEnabled(has_selection)

    def on_add_obstacle(self):
        """新增障礙物"""
        obs_types = ['circle', 'rectangle', 'polygon']
        obs_type = obs_types[self.type_combo.currentIndex()]

        obstacle = {
            'type': obs_type,
            'lat': self.lat_spin.value(),
            'lon': self.lon_spin.value(),
            'radius': self.radius_spin.value(),
            'height': self.height_spin.value() if obs_type == 'rectangle' else 0,
            'altitude': self.alt_spin.value()
        }

        self.obstacles.append(obstacle)
        self.load_obstacles()

        logger.info(f"新增障礙物: {obstacle}")

    def on_edit_obstacle(self):
        """編輯障礙物"""
        selected_items = self.obstacle_list.selectedItems()
        if not selected_items:
            return

        index = selected_items[0].data(Qt.ItemDataRole.UserRole)
        obstacle = self.obstacles[index]

        # 載入到輸入欄位
        type_map = {'circle': 0, 'rectangle': 1, 'polygon': 2}
        self.type_combo.setCurrentIndex(type_map.get(obstacle.get('type', 'circle'), 0))
        self.lat_spin.setValue(obstacle.get('lat', 0))
        self.lon_spin.setValue(obstacle.get('lon', 0))
        self.radius_spin.setValue(obstacle.get('radius', 50))
        self.height_spin.setValue(obstacle.get('height', 50))
        self.alt_spin.setValue(obstacle.get('altitude', 100))

        # 刪除舊的並等待用戶新增更新的
        self.obstacles.pop(index)
        self.load_obstacles()

        QMessageBox.information(self, "編輯模式", "已載入障礙物參數，修改後點擊「新增」以更新")

    def on_delete_obstacle(self):
        """刪除障礙物"""
        selected_items = self.obstacle_list.selectedItems()
        if not selected_items:
            return

        reply = QMessageBox.question(
            self, "確認刪除",
            "確定要刪除選取的障礙物嗎？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )

        if reply == QMessageBox.StandardButton.Yes:
            index = selected_items[0].data(Qt.ItemDataRole.UserRole)
            self.obstacles.pop(index)
            self.load_obstacles()
            logger.info(f"刪除障礙物 #{index+1}")

    def on_clear_all(self):
        """清除所有障礙物"""
        if not self.obstacles:
            return

        reply = QMessageBox.question(
            self, "確認清除",
            "確定要清除所有障礙物嗎？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )

        if reply == QMessageBox.StandardButton.Yes:
            self.obstacles.clear()
            self.load_obstacles()
            logger.info("清除所有障礙物")

    def on_apply(self):
        """套用變更"""
        self.obstacles_changed.emit(self.obstacles)
        QMessageBox.information(self, "已套用", f"已套用 {len(self.obstacles)} 個障礙物設定")
        logger.info(f"套用障礙物設定: {len(self.obstacles)} 個")

    def get_obstacles(self) -> list:
        """獲取障礙物列表"""
        return self.obstacles.copy()
