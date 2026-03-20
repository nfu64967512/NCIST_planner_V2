"""
路徑規劃器基類模組
定義全域規劃與局域規劃的統一介面
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Tuple, Optional, Callable, Dict, Any
import numpy as np
import time


class PlannerType(Enum):
    """規劃器類型枚舉"""
    # 全域規劃器
    ASTAR = auto()
    DIJKSTRA = auto()
    RRT = auto()
    RRT_STAR = auto()
    GRID_SURVEY = auto()      # Mission Planner 風格的 Survey Grid
    COVERAGE = auto()          # 覆蓋路徑規劃
    
    # 局域規劃器
    DWA = auto()
    APF = auto()               # Artificial Potential Field
    MPC = auto()               # Model Predictive Control


class PlannerStatus(Enum):
    """規劃器狀態"""
    IDLE = auto()
    PLANNING = auto()
    SUCCESS = auto()
    FAILED = auto()
    CANCELLED = auto()


@dataclass
class PlannerResult:
    """規劃結果資料類"""
    status: PlannerStatus = PlannerStatus.IDLE
    path: List[np.ndarray] = field(default_factory=list)
    waypoints: List[np.ndarray] = field(default_factory=list)
    cost: float = float('inf')
    planning_time: float = 0.0
    iterations: int = 0
    message: str = ""
    
    # 額外資訊
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    @property
    def is_success(self) -> bool:
        return self.status == PlannerStatus.SUCCESS
    
    @property
    def path_length(self) -> float:
        """計算路徑總長度"""
        if len(self.path) < 2:
            return 0.0
        
        total = 0.0
        for i in range(len(self.path) - 1):
            total += np.linalg.norm(self.path[i + 1] - self.path[i])
        return total


@dataclass
class PlannerConfig:
    """規劃器配置基類"""
    max_iterations: int = 10000
    timeout: float = 30.0          # 超時時間 (s)
    goal_tolerance: float = 0.5    # 目標容差 (m)
    path_resolution: float = 0.5   # 路徑解析度 (m)
    
    # 回調函數
    progress_callback: Optional[Callable[[float, str], None]] = None
    
    def on_progress(self, progress: float, message: str = ""):
        """進度回調"""
        if self.progress_callback:
            self.progress_callback(progress, message)


class BasePlanner(ABC):
    """
    路徑規劃器抽象基類
    
    定義所有規劃器必須實現的介面
    """
    
    def __init__(self, config: PlannerConfig = None):
        self.config = config or PlannerConfig()
        self._status = PlannerStatus.IDLE
        self._cancel_requested = False
    
    @property
    @abstractmethod
    def planner_type(self) -> PlannerType:
        """獲取規劃器類型"""
        pass
    
    @property
    def status(self) -> PlannerStatus:
        """獲取規劃器狀態"""
        return self._status
    
    @abstractmethod
    def plan(self, start: np.ndarray, goal: np.ndarray,
             obstacles: List[Any] = None) -> PlannerResult:
        """
        執行路徑規劃
        
        Args:
            start: 起點位置
            goal: 目標位置
            obstacles: 障礙物列表
            
        Returns:
            規劃結果
        """
        pass
    
    def cancel(self):
        """取消規劃"""
        self._cancel_requested = True
    
    def reset(self):
        """重設規劃器"""
        self._status = PlannerStatus.IDLE
        self._cancel_requested = False
    
    def _check_timeout(self, start_time: float) -> bool:
        """檢查是否超時"""
        return time.time() - start_time > self.config.timeout
    
    def _check_cancelled(self) -> bool:
        """檢查是否被取消"""
        return self._cancel_requested


@dataclass
class GlobalPlannerConfig(PlannerConfig):
    """全域規劃器配置"""
    # 網格參數
    grid_resolution: float = 1.0       # 網格解析度 (m)
    diagonal_movement: bool = True     # 是否允許對角移動
    
    # 啟發式權重
    heuristic_weight: float = 1.0      # A* 啟發式權重
    
    # 路徑優化
    smooth_path: bool = True           # 是否平滑路徑
    shortcut_path: bool = True         # 是否優化捷徑


class GlobalPlanner(BasePlanner):
    """
    全域路徑規劃器基類
    
    用於在已知地圖中規劃從起點到終點的路徑
    """
    
    def __init__(self, config: GlobalPlannerConfig = None):
        super().__init__(config or GlobalPlannerConfig())
    
    @abstractmethod
    def set_map(self, occupancy_grid: np.ndarray, 
                resolution: float,
                origin: np.ndarray):
        """
        設置佔用格地圖
        
        Args:
            occupancy_grid: 佔用格地圖 (0=free, 1=occupied)
            resolution: 解析度 (m/cell)
            origin: 地圖原點 [x, y]
        """
        pass
    
    def world_to_grid(self, position: np.ndarray) -> Tuple[int, int]:
        """世界座標轉網格座標"""
        pass
    
    def grid_to_world(self, grid_pos: Tuple[int, int]) -> np.ndarray:
        """網格座標轉世界座標"""
        pass


@dataclass  
class LocalPlannerConfig(PlannerConfig):
    """局域規劃器配置"""
    # 時間參數
    dt: float = 0.1                    # 時間步長 (s)
    prediction_horizon: float = 3.0    # 預測時間範圍 (s)
    
    # 重規劃參數
    replan_frequency: float = 10.0     # 重規劃頻率 (Hz)
    
    # 安全參數
    min_obstacle_distance: float = 1.0 # 最小障礙物距離 (m)


class LocalPlanner(BasePlanner):
    """
    局域路徑規劃器基類
    
    用於實時避障和軌跡跟蹤
    """
    
    def __init__(self, config: LocalPlannerConfig = None):
        super().__init__(config or LocalPlannerConfig())
        self._global_path: List[np.ndarray] = []
        self._current_waypoint_idx: int = 0
    
    def set_global_path(self, path: List[np.ndarray]):
        """
        設置全域路徑
        
        Args:
            path: 全域路徑點列表
        """
        self._global_path = path
        self._current_waypoint_idx = 0
    
    @abstractmethod
    def compute_velocity(self, current_state: Any,
                        obstacles: List[Any] = None) -> Tuple[float, float]:
        """
        計算控制速度
        
        Args:
            current_state: 當前飛行器狀態
            obstacles: 障礙物列表
            
        Returns:
            (linear_velocity, angular_velocity)
        """
        pass
    
    def get_current_target(self) -> Optional[np.ndarray]:
        """獲取當前目標點"""
        if self._current_waypoint_idx < len(self._global_path):
            return self._global_path[self._current_waypoint_idx]
        return None
    
    def advance_waypoint(self):
        """前進到下一個航點"""
        if self._current_waypoint_idx < len(self._global_path) - 1:
            self._current_waypoint_idx += 1
    
    def is_path_complete(self) -> bool:
        """檢查路徑是否完成"""
        return self._current_waypoint_idx >= len(self._global_path) - 1


class HybridPlanner:
    """
    混合路徑規劃器
    
    結合全域規劃與局域規劃的分層架構
    """
    
    def __init__(self, global_planner: GlobalPlanner,
                 local_planner: LocalPlanner):
        self.global_planner = global_planner
        self.local_planner = local_planner
        
        self._global_path: List[np.ndarray] = []
        self._is_active = False
    
    def plan_mission(self, start: np.ndarray, goal: np.ndarray,
                    obstacles: List[Any] = None) -> PlannerResult:
        """
        規劃完整任務
        
        1. 使用全域規劃器生成初始路徑
        2. 設置局域規劃器跟蹤路徑
        
        Args:
            start: 起點
            goal: 終點
            obstacles: 障礙物列表
            
        Returns:
            全域規劃結果
        """
        # 全域規劃
        result = self.global_planner.plan(start, goal, obstacles)
        
        if result.is_success:
            self._global_path = result.path
            self.local_planner.set_global_path(result.path)
            self._is_active = True
        
        return result
    
    def update(self, current_state: Any,
              dynamic_obstacles: List[Any] = None) -> Tuple[float, float]:
        """
        更新控制輸出
        
        Args:
            current_state: 當前狀態
            dynamic_obstacles: 動態障礙物
            
        Returns:
            控制速度 (v, w)
        """
        if not self._is_active:
            return (0.0, 0.0)
        
        return self.local_planner.compute_velocity(
            current_state, dynamic_obstacles)
    
    def replan(self, current_position: np.ndarray,
              obstacles: List[Any] = None) -> bool:
        """
        重新規劃路徑
        
        當檢測到新障礙物或路徑不可行時調用
        
        Args:
            current_position: 當前位置
            obstacles: 更新後的障礙物列表
            
        Returns:
            重規劃是否成功
        """
        if not self._global_path:
            return False
        
        goal = self._global_path[-1]
        result = self.global_planner.plan(current_position, goal, obstacles)
        
        if result.is_success:
            self._global_path = result.path
            self.local_planner.set_global_path(result.path)
            return True
        
        return False
    
    def stop(self):
        """停止規劃"""
        self._is_active = False
    
    def is_mission_complete(self) -> bool:
        """檢查任務是否完成"""
        return self.local_planner.is_path_complete()


class PlannerFactory:
    """規劃器工廠類"""
    
    _registry: Dict[PlannerType, type] = {}
    
    @classmethod
    def register(cls, planner_type: PlannerType):
        """註冊規劃器類型"""
        def decorator(planner_class: type):
            cls._registry[planner_type] = planner_class
            return planner_class
        return decorator
    
    @classmethod
    def create(cls, planner_type: PlannerType, 
              config: PlannerConfig = None) -> BasePlanner:
        """創建規劃器實例"""
        planner_class = cls._registry.get(planner_type)
        if planner_class is None:
            raise ValueError(f"未註冊的規劃器類型: {planner_type}")
        return planner_class(config)
    
    @classmethod
    def get_available_types(cls) -> List[PlannerType]:
        """獲取可用的規劃器類型"""
        return list(cls._registry.keys())
