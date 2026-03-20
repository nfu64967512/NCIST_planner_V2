"""
禁航區管理對話框
提供固定翼任務中多邊形/圓形禁航區的新增、刪除、清除功能。
"""

from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QFormLayout,
    QLabel, QDoubleSpinBox, QPushButton, QGroupBox,
    QListWidget, QListWidgetItem, QMessageBox,
    QTabWidget, QWidget, QTextEdit, QLineEdit,
)
from PyQt6.QtCore import Qt, pyqtSignal

from utils.logger import get_logger

logger = get_logger()


class NFZManagerDialog(QDialog):
    """
    禁航區管理對話框

    支援新增多邊形/圓形禁航區，顯示目前清單，
    並在關閉時通知主視窗更新。

    nfz_changed 信號攜帶最新禁航區清單：
        [{'type': 'polygon', 'vertices': [(lat, lon), ...], 'name': str}, ...]
        [{'type': 'circle',  'center':   (lat, lon),       'radius': float, 'name': str}, ...]
    """

    nfz_changed = pyqtSignal(list)

    def __init__(self, parent=None, nfz_zones=None):
        super().__init__(parent)
        self.setWindowTitle("禁航區管理 (NFZ)")
        self.setMinimumSize(520, 560)

        self.nfz_zones = list(nfz_zones) if nfz_zones else []

        self._init_ui()
        self._refresh_list()
        logger.info("禁航區管理對話框初始化完成")

    # ─────────────────────────────────────
    # UI 建立
    # ─────────────────────────────────────
    def _init_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(8)

        # 說明
        hint = QLabel(
            "在此定義固定翼任務的禁航區（NFZ）。\n"
            "路徑生成時會自動繞行所有啟用的禁航區。\n"
            "座標格式：緯度, 經度（十進位度數）"
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #FF9800; font-size: 11px; padding: 4px;")
        root.addWidget(hint)

        # 分頁：多邊形 / 圓形
        tabs = QTabWidget()
        tabs.addTab(self._build_polygon_tab(), "多邊形 NFZ")
        tabs.addTab(self._build_circle_tab(), "圓形 NFZ")
        root.addWidget(tabs)

        # 現有禁航區清單
        list_group = QGroupBox("目前禁航區清單")
        list_layout = QVBoxLayout(list_group)
        self.nfz_list_widget = QListWidget()
        self.nfz_list_widget.setMinimumHeight(120)
        list_layout.addWidget(self.nfz_list_widget)

        btn_row = QHBoxLayout()
        self.del_btn = QPushButton("刪除選取")
        self.del_btn.setStyleSheet(
            "background-color: #E53935; color: white; padding: 5px; border-radius: 3px;"
        )
        self.del_btn.clicked.connect(self._on_delete)
        self.clear_btn = QPushButton("清除全部")
        self.clear_btn.setStyleSheet(
            "background-color: #78909C; color: white; padding: 5px; border-radius: 3px;"
        )
        self.clear_btn.clicked.connect(self._on_clear_all)
        btn_row.addWidget(self.del_btn)
        btn_row.addWidget(self.clear_btn)
        list_layout.addLayout(btn_row)
        root.addWidget(list_group)

        # 確定 / 取消
        footer = QHBoxLayout()
        ok_btn = QPushButton("確定")
        ok_btn.setStyleSheet(
            "background-color: #1976D2; color: white; padding: 6px; border-radius: 3px;"
        )
        ok_btn.clicked.connect(self._on_ok)
        cancel_btn = QPushButton("取消")
        cancel_btn.clicked.connect(self.reject)
        footer.addStretch()
        footer.addWidget(ok_btn)
        footer.addWidget(cancel_btn)
        root.addLayout(footer)

    def _build_polygon_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setSpacing(6)

        form = QFormLayout()
        self.poly_name_edit = QLineEdit()
        self.poly_name_edit.setPlaceholderText("選填，如 NFZ_機場")
        form.addRow("名稱:", self.poly_name_edit)

        coord_hint = QLabel(
            "每行一個頂點，格式：緯度, 經度\n"
            "例如：\n"
            "  24.123456, 121.654321\n"
            "  24.124000, 121.655000\n"
            "  24.122000, 121.655000"
        )
        coord_hint.setStyleSheet("color: #888; font-size: 10px;")
        coord_hint.setWordWrap(True)
        layout.addLayout(form)
        layout.addWidget(coord_hint)

        self.poly_coords_edit = QTextEdit()
        self.poly_coords_edit.setPlaceholderText(
            "24.123456, 121.654321\n24.124000, 121.655000\n24.122000, 121.655000"
        )
        self.poly_coords_edit.setMaximumHeight(120)
        layout.addWidget(self.poly_coords_edit)

        add_btn = QPushButton("新增多邊形 NFZ")
        add_btn.setStyleSheet(
            "background-color: #388E3C; color: white; padding: 6px; border-radius: 3px;"
        )
        add_btn.clicked.connect(self._on_add_polygon)
        layout.addWidget(add_btn)
        layout.addStretch()
        return tab

    def _build_circle_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)
        layout.setSpacing(6)

        form = QFormLayout()

        self.circle_name_edit = QLineEdit()
        self.circle_name_edit.setPlaceholderText("選填，如 NFZ_禁飛圓")
        form.addRow("名稱:", self.circle_name_edit)

        self.circle_lat_spin = QDoubleSpinBox()
        self.circle_lat_spin.setRange(-90.0, 90.0)
        self.circle_lat_spin.setDecimals(6)
        self.circle_lat_spin.setValue(24.0)
        form.addRow("圓心緯度:", self.circle_lat_spin)

        self.circle_lon_spin = QDoubleSpinBox()
        self.circle_lon_spin.setRange(-180.0, 180.0)
        self.circle_lon_spin.setDecimals(6)
        self.circle_lon_spin.setValue(121.0)
        form.addRow("圓心經度:", self.circle_lon_spin)

        self.circle_radius_spin = QDoubleSpinBox()
        self.circle_radius_spin.setRange(10.0, 50000.0)
        self.circle_radius_spin.setDecimals(0)
        self.circle_radius_spin.setValue(500.0)
        self.circle_radius_spin.setSuffix(" m")
        form.addRow("半徑:", self.circle_radius_spin)

        layout.addLayout(form)

        add_btn = QPushButton("新增圓形 NFZ")
        add_btn.setStyleSheet(
            "background-color: #388E3C; color: white; padding: 6px; border-radius: 3px;"
        )
        add_btn.clicked.connect(self._on_add_circle)
        layout.addWidget(add_btn)
        layout.addStretch()
        return tab

    # ─────────────────────────────────────
    # 動作
    # ─────────────────────────────────────
    def _on_add_polygon(self):
        text = self.poly_coords_edit.toPlainText().strip()
        if not text:
            QMessageBox.warning(self, "缺少資料", "請輸入至少 3 個頂點座標。")
            return

        vertices = []
        for line in text.splitlines():
            line = line.strip()
            if not line:
                continue
            parts = line.replace(',', ' ').split()
            if len(parts) < 2:
                QMessageBox.warning(self, "格式錯誤", f"無法解析: {line}\n格式應為：緯度, 經度")
                return
            try:
                lat, lon = float(parts[0]), float(parts[1])
            except ValueError:
                QMessageBox.warning(self, "格式錯誤", f"數值無效: {line}")
                return
            vertices.append((lat, lon))

        if len(vertices) < 3:
            QMessageBox.warning(self, "頂點不足", "多邊形 NFZ 至少需要 3 個頂點。")
            return

        name = self.poly_name_edit.text().strip() or f"NFZ_Poly_{len(self.nfz_zones)}"
        zone = {'type': 'polygon', 'vertices': vertices, 'name': name}
        self.nfz_zones.append(zone)
        self._refresh_list()
        self.poly_coords_edit.clear()
        self.poly_name_edit.clear()
        logger.info(f"新增多邊形 NFZ: {name}, {len(vertices)} 頂點")

    def _on_add_circle(self):
        lat = self.circle_lat_spin.value()
        lon = self.circle_lon_spin.value()
        radius = self.circle_radius_spin.value()
        name = self.circle_name_edit.text().strip() or f"NFZ_Circle_{len(self.nfz_zones)}"
        zone = {'type': 'circle', 'center': (lat, lon), 'radius': radius, 'name': name}
        self.nfz_zones.append(zone)
        self._refresh_list()
        self.circle_name_edit.clear()
        logger.info(f"新增圓形 NFZ: {name}, 圓心=({lat:.6f},{lon:.6f}), r={radius:.0f}m")

    def _on_delete(self):
        row = self.nfz_list_widget.currentRow()
        if row < 0:
            return
        name = self.nfz_zones[row]['name']
        reply = QMessageBox.question(
            self, "確認刪除", f"刪除禁航區「{name}」？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.nfz_zones.pop(row)
            self._refresh_list()

    def _on_clear_all(self):
        if not self.nfz_zones:
            return
        reply = QMessageBox.question(
            self, "清除全部", "確定清除所有禁航區？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            self.nfz_zones.clear()
            self._refresh_list()

    def _on_ok(self):
        self.nfz_changed.emit(list(self.nfz_zones))
        self.accept()

    def _refresh_list(self):
        self.nfz_list_widget.clear()
        for zone in self.nfz_zones:
            if zone['type'] == 'polygon':
                text = f"🟥 多邊形  {zone['name']}  ({len(zone['vertices'])} 頂點)"
            else:
                lat, lon = zone['center']
                text = (
                    f"🔴 圓形  {zone['name']}  "
                    f"圓心({lat:.5f},{lon:.5f})  r={zone['radius']:.0f}m"
                )
            self.nfz_list_widget.addItem(QListWidgetItem(text))

        count = len(self.nfz_zones)
        self.del_btn.setEnabled(count > 0)
        self.clear_btn.setEnabled(count > 0)
