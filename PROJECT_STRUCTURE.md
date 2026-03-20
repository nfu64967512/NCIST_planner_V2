# UAV Path Planner - 無人機路徑規劃系統 V2.0

## 專案概述

基於 Mission Planner 演算法核心，採用現代化 Python 架構重新設計的無人機路徑規劃系統。
支援多旋翼與固定翼（未來），整合全域規劃與 DWA 局域規劃功能。

## 設計原則

1. **策略模式 (Strategy Pattern)**: 飛行器類型可插拔切換
2. **工廠模式 (Factory Pattern)**: 路徑規劃器動態創建
3. **觀察者模式 (Observer Pattern)**: UI 與核心演算法解耦
4. **分層架構**: 全域規劃 → 局域規劃 → 軌跡優化

## 專案結構

```
uav_path_planner/
├── main.py                          # 程式進入點
├── requirements.txt                 # 依賴套件
├── config/
│   ├── __init__.py
│   ├── settings.py                  # 全局配置
│   └── vehicle_profiles.yaml        # 飛行器參數配置
│
├── core/                            # 核心演算法層
│   ├── __init__.py
│   ├── base/
│   │   ├── __init__.py
│   │   ├── vehicle_base.py          # 飛行器基類（抽象）
│   │   ├── planner_base.py          # 規劃器基類（抽象）
│   │   └── constraint_base.py       # 約束基類
│   │
│   ├── vehicles/                    # 飛行器模型
│   │   ├── __init__.py
│   │   ├── multirotor.py            # 多旋翼模型
│   │   └── fixed_wing.py            # 固定翼模型（預留）
│   │
│   ├── geometry/                    # 幾何計算模組
│   │   ├── __init__.py
│   │   ├── coordinate.py            # 座標轉換 (WGS84/UTM/Local)
│   │   ├── polygon.py               # 多邊形運算
│   │   ├── intersection.py          # 交點計算
│   │   └── transform.py             # 幾何變換
│   │
│   ├── global_planner/              # 全域路徑規劃
│   │   ├── __init__.py
│   │   ├── grid_generator.py        # Survey Grid 生成器（Mission Planner 核心）
│   │   ├── astar.py                 # A* 搜尋
│   │   ├── rrt.py                   # RRT/RRT* 演算法
│   │   ├── dijkstra.py              # Dijkstra 演算法
│   │   └── coverage_planner.py      # 覆蓋路徑規劃
│   │
│   ├── local_planner/               # 局域路徑規劃
│   │   ├── __init__.py
│   │   ├── dwa.py                   # Dynamic Window Approach
│   │   ├── apf.py                   # Artificial Potential Field
│   │   └── mpc.py                   # Model Predictive Control（預留）
│   │
│   ├── trajectory/                  # 軌跡優化
│   │   ├── __init__.py
│   │   ├── smoother.py              # 路徑平滑化
│   │   ├── time_optimal.py          # 時間最優化
│   │   └── spline.py                # 樣條曲線生成
│   │
│   └── collision/                   # 碰撞檢測與避障
│       ├── __init__.py
│       ├── obstacle_manager.py      # 障礙物管理
│       ├── collision_checker.py     # 碰撞檢測
│       └── avoidance.py             # 避障策略
│
├── mission/                         # 任務管理層
│   ├── __init__.py
│   ├── mission_manager.py           # 任務管理器
│   ├── waypoint.py                  # 航點資料結構
│   ├── survey_mission.py            # Survey 任務
│   ├── swarm_coordinator.py         # 群飛協調
│   └── mavlink_exporter.py          # MAVLink 匯出
│
├── sensors/                         # 感測器整合層
│   ├── __init__.py
│   ├── camera_model.py              # 相機模型與 FOV 計算
│   ├── terrain_manager.py           # 地形管理（SRTM）
│   └── sensor_fusion.py             # 感測器融合（預留 Kimera VIO）
│
├── ui/                              # PyQt UI 層
│   ├── __init__.py
│   ├── main_window.py               # 主視窗
│   ├── widgets/
│   │   ├── __init__.py
│   │   ├── map_widget.py            # 地圖組件
│   │   ├── parameter_panel.py       # 參數面板
│   │   ├── mission_panel.py         # 任務面板
│   │   ├── planner_panel.py         # 規劃器面板
│   │   └── status_bar.py            # 狀態列
│   │
│   ├── dialogs/
│   │   ├── __init__.py
│   │   ├── vehicle_config.py        # 飛行器配置對話框
│   │   ├── camera_config.py         # 相機配置對話框
│   │   └── export_dialog.py         # 匯出對話框
│   │
│   └── resources/
│       ├── icons/
│       └── styles/
│           └── dark_theme.qss
│
├── utils/                           # 工具層
│   ├── __init__.py
│   ├── logger.py                    # 日誌工具
│   ├── math_utils.py                # 數學工具
│   └── file_io.py                   # 檔案讀寫
│
└── tests/                           # 測試
    ├── __init__.py
    ├── test_geometry.py
    ├── test_planners.py
    └── test_dwa.py
```

## 核心類別關係圖

```
                    ┌─────────────────┐
                    │   MissionManager │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
    ┌─────────────────┐ ┌────────────┐ ┌──────────────┐
    │  GlobalPlanner  │ │LocalPlanner│ │TrajectoryOpt │
    │  (A*/RRT/Grid)  │ │   (DWA)    │ │  (Smoother)  │
    └────────┬────────┘ └─────┬──────┘ └──────┬───────┘
             │                │               │
             └────────────────┼───────────────┘
                              ▼
                    ┌─────────────────┐
                    │  VehicleModel   │
                    │ ┌─────────────┐ │
                    │ │ Multirotor  │ │
                    │ │ FixedWing   │ │
                    │ └─────────────┘ │
                    └─────────────────┘
```

## 技術棧

- **UI**: PyQt6 / PyQt5
- **地圖**: folium + PyQtWebEngine 或 QGraphicsView
- **幾何**: Shapely, NumPy
- **路徑規劃**: 自實現 + PythonRobotics 參考
- **加速**: Numba JIT
- **座標轉換**: pyproj, utm
- **MAVLink**: pymavlink
