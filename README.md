# UAV Path Planner - 無人機路徑規劃系統

**版本**: 2.1.0
**授權**: MIT
**Python**: >= 3.10

## 專案概述

基於 Mission Planner 演算法核心，採用現代化 Python 架構設計的專業級無人機路徑規劃系統。支援多旋翼、固定翼與 VTOL 載具，整合多種全域規劃演算法，提供互動式地圖操作介面。

### 主要特性

- **互動式地圖**: 基於 Folium + Leaflet 的衛星地圖，左鍵點擊添加角點，右鍵工具選單
- **多種規劃演算法**: Grid / Spiral / Circular / A* / RRT / RRT* / Dijkstra / DWA
- **多飛行器支援**: 多旋翼（DJI Mavic 3、Phantom 4 Pro、Mini 3 Pro）、固定翼（衝浪者、Generic）、VTOL
- **固定翼三階段任務**: 起飛路徑 → 螺旋/同心圓掃描（轉彎半徑強制合規）→ 五邊進場降落
- **障礙物避讓**: 碰撞檢測與智能避障
- **MAVLink 匯出**: QGC WPL 110 航點檔案匯出（Mission Planner / QGroundControl 相容）
- **即時路徑預覽**: 參數調整後自動重新生成路徑
- **座標系轉換**: WGS84 / UTM / 本地 ENU 座標互轉
- **地圖拖曳定圓**: 直接在地圖上拖曳定義螺旋/同心圓中心與半徑
- **多邊形編輯器**: 精確輸入或編輯角點座標

## 快速開始

### 1. 安裝依賴

```bash
# 創建虛擬環境（推薦）
python -m venv venv
venv\Scripts\activate        # Windows
# source venv/bin/activate   # Linux/Mac

# 安裝依賴套件
pip install -r requirements.txt
```

核心依賴：

| 套件 | 用途 |
|------|------|
| `PyQt6` + `PyQt6-WebEngine` | GUI 與地圖顯示 |
| `folium` | 互動式地圖生成 |
| `numpy` / `scipy` | 數值計算 |
| `shapely` / `pyproj` | 幾何與座標投影 |
| `PyYAML` | 配置檔讀取 |
| `pymavlink` | MAVLink 航點匯出 |

### 2. 啟動程式

```bash
# GUI 模式（預設）
python main.py

# 指定飛行器配置
python main.py --vehicle surfer

# 設定日誌等級
python main.py --log-level DEBUG

# 無介面模式（開發中）
python main.py --no-ui
```

### 3. 基本操作流程

1. **定義飛行區域** — 在地圖上左鍵點擊添加角點（至少 3 個），形成多邊形邊界
2. **選擇飛行器** — 右側面板選擇載具類型與型號
3. **選擇演算法** — 選擇路徑演算法（Grid、Spiral、Circular 等）
4. **調整參數** — 設定飛行高度、速度、航線間距、掃描角度等
5. **預覽路徑** — 點擊「預覽」按鈕或按 Enter 生成路徑
6. **匯出航點** — 匯出為 QGC WPL 110 (.waypoints) 或 CSV 格式

## 路徑規劃演算法

### 覆蓋任務（區域掃描）

| 演算法 | 說明 | 適用場景 |
|--------|------|----------|
| **Grid** | 網格掃描（之字形） | 農業噴灑、航拍測繪 |
| **Spiral** | 阿基米德螺線掃描（支援曲率係數、高度漸變） | 從外圍向中心搜索 |
| **Circular** | 同心圓擴張掃描（自適應密度過渡弧） | 環繞目標、逐圈上升拍攝 |

### 點對點路徑

| 演算法 | 說明 | 適用場景 |
|--------|------|----------|
| **A*** | 啟發式最短路徑搜索 | 已知環境中的最優路徑 |
| **RRT** | 快速探索隨機樹 | 複雜障礙物環境 |
| **RRT*** | RRT 最優化版本 | 需要路徑品質保證 |
| **Dijkstra** | 保證最短路徑 | 簡單環境最短路徑 |
| **DWA** | 動態窗口法 | 即時避障（需配合飛控） |

### 固定翼任務規劃

固定翼模式採用三階段任務規劃，確保與 ArduPlane 飛控相容：

| 階段 | 說明 |
|------|------|
| **Phase 1 — 起飛** | MAVLink TAKEOFF 指令 + 爬升轉場至掃描高度 |
| **Phase 2 — 掃描** | 螺旋掃描 / 同心圓掃描（含轉彎半徑強制合規檢查） |
| **Phase 3 — 降落** | 五邊進場（Crosswind → Downwind → Base → Final）|

**轉彎半徑合規性檢查**：根據機體最大傾斜角計算所需傾斜角，回傳 `ok / warning / critical` 三段狀態。

## 飛行器配置

### 多旋翼

| 載具 | 最大速度 | 飛行時間 | 抗風能力 |
|------|----------|----------|----------|
| DJI Mavic 3 | 19 m/s | 46 min | 12 m/s |
| DJI Phantom 4 Pro | 20 m/s | 30 min | 10 m/s |
| DJI Mini 3 Pro | 16 m/s | 34 min | 10.7 m/s |
| Generic Quadcopter | 15 m/s | 25 min | 10 m/s |

### 固定翼

| 載具 | 巡航速度 | 飛行時間 | 最小轉彎半徑 |
|------|----------|----------|--------------|
| 衝浪者 (Surfer) | 18 m/s | 90 min | 30 m |
| Generic Fixed Wing | 18 m/s | 120 min | 50 m |

### VTOL

| 載具 | 多旋翼巡航 | 固定翼巡航 | 飛行時間 |
|------|-----------|-----------|---------|
| Generic VTOL | 5 m/s | 20 m/s | 90 min（固定翼模式）|

## 專案結構

```
DWA_path_planner-main/
├── main.py                          # 程式進入點（GUI / CLI 模式）
├── requirements.txt                 # 依賴套件
│
├── config/                          # 配置模組
│   ├── settings.py                  # 全局配置（地圖、安全、UI 參數）
│   └── vehicle_profiles.yaml        # 飛行器參數檔（多旋翼/固定翼/VTOL）
│
├── core/                            # 核心演算法
│   ├── base/                        # 基礎抽象類別
│   │   ├── vehicle_base.py          # 飛行器基類
│   │   ├── planner_base.py          # 規劃器基類
│   │   └── constraint_base.py       # 約束基類
│   │
│   ├── geometry/                    # 幾何與座標轉換
│   │   ├── coordinate.py            # WGS84/UTM/ENU 座標轉換
│   │   ├── transform.py             # 2D 仿射變換、旋轉座標系
│   │   ├── polygon.py               # 多邊形運算
│   │   ├── intersection.py          # 交點計算
│   │   └── region_divider.py        # 飛行區域分割
│   │
│   ├── global_planner/              # 全域路徑規劃
│   │   ├── coverage_planner.py      # 覆蓋規劃（Grid / Spiral / Circular）
│   │   ├── fixed_wing_planner.py    # 固定翼三階段任務規劃
│   │   ├── astar.py                 # A* 路徑規劃
│   │   ├── rrt.py                   # RRT / RRT* 路徑規劃
│   │   ├── dijkstra.py              # Dijkstra 路徑規劃
│   │   └── grid_generator.py        # 柵格地圖生成
│   │
│   ├── collision/                   # 碰撞檢測
│   │   ├── collision_checker.py     # 碰撞檢測器
│   │   ├── avoidance.py             # 避障策略
│   │   └── obstacle_manager.py      # 障礙物管理
│   │
│   ├── local_planner/               # 局域規劃
│   │   ├── dwa.py                   # 動態窗口法
│   │   ├── apf.py                   # 人工勢場法
│   │   └── mpc.py                   # 模型預測控制
│   │
│   ├── trajectory/                  # 軌跡優化
│   │   ├── smoother.py              # 路徑平滑
│   │   ├── spline.py                # 樣條插值
│   │   └── time_optimal.py          # 時間最優軌跡
│   │
│   └── vehicles/                    # 飛行器模型
│       ├── multirotor.py            # 多旋翼模型
│       └── fixed_wing.py            # 固定翼模型
│
├── mission/                         # 任務管理
│   ├── mission_manager.py           # 任務管理器
│   ├── waypoint.py                  # 航點定義
│   ├── survey_mission.py            # Survey 測繪任務
│   ├── mavlink_exporter.py          # MAVLink / QGC WPL 110 匯出
│   └── swarm_coordinator.py         # 群飛協調
│
├── sensors/                         # 感測器模組
│   ├── camera_model.py              # 相機模型（GSD / 覆蓋率計算）
│   ├── sensor_fusion.py             # 感測器融合
│   └── terrain_manager.py           # 地形管理
│
├── ui/                              # PyQt6 GUI
│   ├── main_window.py               # 主視窗（路徑生成協調）
│   ├── widgets/
│   │   ├── map_widget.py            # 地圖組件（Folium + WebEngine + JS 事件）
│   │   ├── parameter_panel.py       # 參數面板（含固定翼 / 螺旋專用區段）
│   │   ├── mission_panel.py         # 任務面板
│   │   └── polygon_editor.py        # 多邊形編輯器
│   ├── dialogs/
│   │   ├── camera_config.py         # 相機配置
│   │   ├── vehicle_config.py        # 載具配置
│   │   ├── export_dialog.py         # 匯出對話框
│   │   └── obstacle_manager.py      # 障礙物管理
│   └── resources/
│       └── styles/                  # QSS 主題樣式（modern / light）
│
├── utils/                           # 工具模組
│   ├── logger.py                    # 日誌系統
│   ├── math_utils.py                # 數學工具
│   └── file_io.py                   # 檔案讀寫（YAML、航點匯出）
│
└── data/
    └── logs/                        # 執行日誌（依日期命名）
```

## 配置說明

### 系統配置 (settings.py)

| 配置類別 | 說明 |
|----------|------|
| `PathSettings` | 路徑與目錄配置 |
| `MapSettings` | 地圖預設位置與縮放 |
| `SafetySettings` | 飛行高度/速度/間距限制 |
| `ExportSettings` | 匯出格式設定 |
| `UISettings` | 視窗大小與標題 |

### 螺旋掃描進階參數

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `spiral_curvature` | 曲率係數（控制每圈緊密程度） | 1.0 |
| `spiral_alt_step` | 每圈高度增量 (m) | 0.0 |
| `spiral_base_altitude` | 螺旋基準高度 (m) | 同任務高度 |
| `circle_max_radius` | 最大半徑 (m) | 200.0 |
| `circle_min_radius` | 最小半徑 (m) | 0.0 |

### 固定翼進場參數

| 參數 | 說明 | 預設值 |
|------|------|--------|
| `fw_pattern_alt` | 五邊飛行高度 (m) | 80.0 |
| `fw_downwind_offset` | 下風邊側偏距離 (m) | 200.0 |
| `fw_pattern_leg` | 標準腿長 (m) | 300.0 |
| `fw_final_dist` | 最終進場距離 (m) | 400.0 |
| `fw_takeoff_bearing` | 起飛方向 (°，0=北) | 0.0 |
| `fw_landing_bearing` | 降落進場方向 (°) | 180.0 |

## 快捷鍵

| 按鍵 | 功能 |
|------|------|
| Enter / Space | 生成路徑 |
| Escape | 清除路徑 |
| Delete / Backspace | 刪除最後一個角點 |
| Ctrl+N | 新建任務 |
| Ctrl+O | 開啟任務 |
| Ctrl+S | 儲存任務 |
| Ctrl+E | 匯出航點 |
| Ctrl+R | 清除全部 |
| Ctrl+M | 開啟多邊形編輯器 |

## 技術棧

| 類別 | 技術 |
|------|------|
| 語言 | Python 3.10+ |
| GUI | PyQt6 |
| 地圖 | Folium + Leaflet + PyQt6-WebEngine |
| 地圖互動 | JS click/drag → `pyqt://` URL scheme → Python signal |
| 幾何 | NumPy, SciPy, Shapely |
| 座標投影 | pyproj + 自製 WGS84/UTM/ENU 轉換器 |
| 通訊 | pymavlink（QGC WPL 110） |
| 配置 | PyYAML |

## 致謝

- [Mission Planner](https://ardupilot.org/planner/) — 演算法參考
- [ArduPilot](https://ardupilot.org/) — MAVLink 協議
- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) — 路徑規劃演算法參考
- [QGroundControl](http://qgroundcontrol.com/) — 航點格式參考
- [Folium](https://python-visualization.github.io/folium/) — 互動式地圖
