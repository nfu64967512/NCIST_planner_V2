#!/usr/bin/env python3
"""
UAV 飛行區域多邊形編輯器 - 獨立啟動程式

功能：
- 點擊地圖添加角點（最多 100 個）
- 支援 Google 衛星圖/地圖/OpenStreetMap
- 匯入/匯出 JSON 格式角點資料
- 自動計算面積
- 可與主程式整合使用

使用方式：
    python polygon_editor_app.py
"""

import sys
from pathlib import Path

# 添加專案根目錄到路徑
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))


def main():
    """主函式"""
    try:
        # 初始化配置（可選）
        try:
            from config import init_settings
            init_settings()
        except ImportError:
            pass

        # 檢查 PyQt6 是否可用
        try:
            from PyQt6.QtWidgets import QApplication
            from PyQt6.QtCore import Qt
        except ImportError:
            print("錯誤: PyQt6 未安裝")
            print("請執行: pip install PyQt6 PyQt6-WebEngine")
            return 1

        # 檢查 PyQt6-WebEngine 是否可用
        try:
            from PyQt6.QtWebEngineWidgets import QWebEngineView
        except ImportError:
            print("錯誤: PyQt6-WebEngine 未安裝")
            print("請執行: pip install PyQt6-WebEngine")
            return 1

        # 檢查 folium 是否可用
        try:
            import folium
        except ImportError:
            print("錯誤: folium 未安裝")
            print("請執行: pip install folium")
            return 1

        # 設置 OpenGL 共享上下文（WebEngine 需要）
        QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)

        # 創建應用程式
        app = QApplication(sys.argv)
        app.setApplicationName("UAV 飛行區域編輯器")
        app.setOrganizationName("UAV Team")

        # 導入並創建編輯器視窗
        from ui.widgets.polygon_editor import PolygonEditorWindow

        window = PolygonEditorWindow(max_corners=100)
        window.show()

        print("=" * 50)
        print("  UAV 飛行區域多邊形編輯器")
        print("=" * 50)
        print("  - 點擊地圖添加角點（最多 100 個）")
        print("  - 使用滾輪縮放地圖")
        print("  - Ctrl+Z: 撤銷上一個角點")
        print("  - Ctrl+E: 匯出角點")
        print("  - Ctrl+I: 匯入角點")
        print("=" * 50)

        return app.exec()

    except Exception as e:
        print(f"啟動失敗: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())
