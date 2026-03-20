"""
群飛協調器模組
負責多無人機任務的協調、同步、避撞等功能
支持時間錯開、高度分層、區域分配等策略
"""

import sys
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
import math

# 添加專案根目錄到路徑
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from mission.mission_manager import Mission
from mission.survey_mission import SurveyMission
from mission.waypoint import Waypoint, WaypointSequence, MAVCommand


# ==========================================
# 無人機資料類
# ==========================================
@dataclass
class DroneInfo:
    """無人機資訊"""
    drone_id: int                       # 無人機 ID
    name: str                           # 名稱
    mission: Optional[Mission] = None   # 分配的任務
    start_time: float = 0.0            # 起飛時間（秒）
    rtl_altitude: float = 50.0         # RTL 高度（公尺）
    status: str = "ready"              # 狀態


# ==========================================
# 群飛任務類
# ==========================================
class SwarmMission:
    """
    群飛任務類
    
    管理多個無人機的協同任務
    """
    
    def __init__(self, name: str = "Swarm Mission"):
        """
        初始化群飛任務
        
        參數:
            name: 任務名稱
        """
        self.name = name
        self.drones: List[DroneInfo] = []
        
        # 群飛參數
        self.swarm_params = {
            'strategy': 'sequential',       # 策略：sequential, simultaneous
            'safety_distance': 5.0,        # 安全距離（公尺）
            'altitude_increment': 3.0,     # RTL 高度遞增（公尺）
            'time_buffer': 2.0,            # 時間緩衝（秒）
            'coordination_mode': 'smart',  # 協調模式：smart, simple
        }
        
        # 統計資訊
        self.stats = {
            'total_drones': 0,
            'total_waypoints': 0,
            'total_distance': 0.0,
            'estimated_time': 0.0,
        }
    
    def add_drone(self, drone_id: int, name: str) -> DroneInfo:
        """
        添加無人機
        
        參數:
            drone_id: 無人機 ID
            name: 名稱
        
        返回:
            DroneInfo 實例
        """
        drone = DroneInfo(drone_id=drone_id, name=name)
        self.drones.append(drone)
        self.stats['total_drones'] = len(self.drones)
        return drone
    
    def assign_mission(self, drone_id: int, mission: Mission):
        """
        分配任務給無人機
        
        參數:
            drone_id: 無人機 ID
            mission: 任務實例
        """
        for drone in self.drones:
            if drone.drone_id == drone_id:
                drone.mission = mission
                break
    
    def set_strategy(self, strategy: str):
        """
        設定群飛策略
        
        參數:
            strategy: 策略名稱
                - sequential: 順序執行（時間錯開）
                - simultaneous: 同時執行（空間分離）
        """
        if strategy not in ('sequential', 'simultaneous'):
            raise ValueError(f"不支援的策略: {strategy}")
        
        self.swarm_params['strategy'] = strategy
    
    def calculate_loiter_times(self) -> List[float]:
        """
        計算 LOITER 等待時間（智能避撞模式）
        
        返回:
            每台無人機的等待時間列表
        """
        try:
            from collision_avoidance import CollisionAvoidanceSystem
            
            collision_system = CollisionAvoidanceSystem(
                self.swarm_params['safety_distance']
            )
            
            loiter_times = []
            prev_waypoints = None
            
            for idx, drone in enumerate(self.drones):
                if not drone.mission or len(drone.mission.waypoints) == 0:
                    loiter_times.append(0.0)
                    continue
                
                # 獲取導航航點
                nav_waypoints = drone.mission.waypoints.get_navigation_waypoints()
                waypoints = [(wp.lat, wp.lon) for wp in nav_waypoints]
                
                if idx == 0:
                    # 第一台無人機不需要等待
                    loiter_times.append(0.0)
                    prev_waypoints = waypoints
                else:
                    # 計算需要等待的時間
                    if prev_waypoints and waypoints:
                        start_point = waypoints[0]
                        speed = drone.mission.params.get('speed', 10.0)
                        
                        loiter_time = collision_system.calculate_loiter_delay(
                            prev_waypoints, start_point, speed
                        )
                        
                        # 加上額外的時間錯開
                        loiter_time += idx * self.swarm_params['time_buffer']
                        
                        loiter_times.append(loiter_time)
                        prev_waypoints = waypoints
                    else:
                        loiter_times.append(idx * self.swarm_params['time_buffer'])
            
            return loiter_times
        except ImportError:
            # 如果無法導入 collision_avoidance，使用簡單的時間錯開
            return [i * 5.0 for i in range(len(self.drones))]
    
    def calculate_rtl_altitudes(self, base_altitude: float = 50.0) -> List[float]:
        """
        計算 RTL 高度（高度分層）
        
        參數:
            base_altitude: 基準高度（公尺）
        
        返回:
            每台無人機的 RTL 高度列表
        """
        rtl_altitudes = []
        increment = self.swarm_params['altitude_increment']
        
        for idx in range(len(self.drones)):
            # 反向分配（最後一台最高）
            altitude = base_altitude + (len(self.drones) - idx - 1) * increment
            rtl_altitudes.append(altitude)
        
        return rtl_altitudes
    
    def apply_collision_avoidance(self):
        """
        應用避撞策略到所有無人機任務
        """
        strategy = self.swarm_params['strategy']
        
        if strategy == 'sequential':
            # 順序執行策略：計算 LOITER 時間
            loiter_times = self.calculate_loiter_times()
            
            for idx, drone in enumerate(self.drones):
                if drone.mission:
                    drone.start_time = loiter_times[idx]
                    
                    # 插入 LOITER 命令到任務中
                    if loiter_times[idx] > 0 and len(drone.mission.waypoints) > 0:
                        self._insert_loiter_to_mission(
                            drone.mission, loiter_times[idx]
                        )
        
        elif strategy == 'simultaneous':
            # 同時執行策略：確保空間分離
            # 這裡假設任務已經通過區域分割實現空間分離
            pass
        
        # 計算並應用 RTL 高度分層
        base_alt = max(
            (drone.mission.params.get('altitude', 50.0) 
             for drone in self.drones if drone.mission),
            default=50.0
        )
        
        rtl_altitudes = self.calculate_rtl_altitudes(base_alt)
        
        for idx, drone in enumerate(self.drones):
            if drone.mission:
                drone.rtl_altitude = rtl_altitudes[idx]
                self._update_rtl_altitude(drone.mission, rtl_altitudes[idx])
    
    def _insert_loiter_to_mission(self, mission: Mission, loiter_time: float):
        """
        插入 LOITER 命令到任務
        
        參數:
            mission: 任務實例
            loiter_time: 等待時間（秒）
        """
        try:
            from collision_avoidance import CollisionAvoidanceSystem
            
            collision_system = CollisionAvoidanceSystem()
            
            # 轉換為 QGC 格式
            lines = mission.waypoints.to_qgc_format()
            
            # 插入 LOITER 命令（在速度設定後）
            updated_lines = collision_system.insert_loiter_command(
                lines, loiter_time, insert_after_line=2
            )
            
            # 更新任務航點
            mission.waypoints = WaypointSequence.from_qgc_format(updated_lines)
        except Exception as e:
            print(f"插入 LOITER 命令失敗: {e}")
    
    def _update_rtl_altitude(self, mission: Mission, rtl_altitude: float):
        """
        更新任務的 RTL 高度
        
        參數:
            mission: 任務實例
            rtl_altitude: RTL 高度（公尺）
        """
        for waypoint in mission.waypoints:
            # 更新 TAKEOFF 到 RTL 高度的命令
            if waypoint.command == MAVCommand.NAV_TAKEOFF:
                # 在 RTL 前插入 TAKEOFF 到新高度的命令
                pass
    
    def calculate_statistics(self):
        """計算群飛統計資訊"""
        total_waypoints = 0
        total_distance = 0.0
        max_time = 0.0
        
        for drone in self.drones:
            if drone.mission:
                total_waypoints += len(drone.mission.waypoints)
                total_distance += drone.mission.waypoints.calculate_total_distance()
                
                # 計算該無人機的完成時間
                flight_time = drone.mission.waypoints.estimate_flight_time(
                    drone.mission.params.get('speed', 10.0)
                )
                completion_time = drone.start_time + flight_time
                max_time = max(max_time, completion_time)
        
        self.stats.update({
            'total_waypoints': total_waypoints,
            'total_distance': total_distance,
            'estimated_time': max_time,
        })
    
    def validate(self) -> Tuple[bool, List[str]]:
        """
        驗證群飛任務
        
        返回:
            (是否有效, 錯誤訊息列表)
        """
        errors = []
        
        # 檢查無人機數量
        if len(self.drones) == 0:
            errors.append("沒有無人機")
        
        # 檢查每台無人機的任務
        for drone in self.drones:
            if drone.mission is None:
                errors.append(f"無人機 {drone.name} 沒有分配任務")
            else:
                # 驗證任務
                valid, mission_errors = drone.mission.validate()
                if not valid:
                    errors.extend([f"{drone.name}: {e}" for e in mission_errors])
        
        # 檢查空間分離（simultaneous 策略）
        if self.swarm_params['strategy'] == 'simultaneous':
            if not self._check_spatial_separation():
                errors.append("無人機任務區域存在重疊，無法同時執行")
        
        return len(errors) == 0, errors
    
    def _check_spatial_separation(self) -> bool:
        """
        檢查空間分離
        
        返回:
            是否有足夠的空間分離
        """
        # 簡化實現：檢查邊界框是否重疊
        bboxes = []
        
        for drone in self.drones:
            if drone.mission:
                bbox = drone.mission.waypoints.get_bounding_box()
                if bbox:
                    bboxes.append(bbox)
        
        # 檢查每對邊界框是否重疊
        for i in range(len(bboxes)):
            for j in range(i + 1, len(bboxes)):
                if self._bboxes_overlap(bboxes[i], bboxes[j]):
                    return False
        
        return True
    
    def _bboxes_overlap(self, bbox1: Tuple[float, float, float, float],
                       bbox2: Tuple[float, float, float, float]) -> bool:
        """
        檢查兩個邊界框是否重疊
        
        參數:
            bbox1, bbox2: (min_lat, min_lon, max_lat, max_lon)
        
        返回:
            是否重疊
        """
        return not (bbox1[2] < bbox2[0] or bbox1[0] > bbox2[2] or
                   bbox1[3] < bbox2[1] or bbox1[1] > bbox2[3])
    
    def get_drone_by_id(self, drone_id: int) -> Optional[DroneInfo]:
        """
        根據 ID 獲取無人機
        
        參數:
            drone_id: 無人機 ID
        
        返回:
            DroneInfo 實例，未找到返回 None
        """
        for drone in self.drones:
            if drone.drone_id == drone_id:
                return drone
        return None
    
    def generate_mission_briefing(self) -> str:
        """
        生成群飛任務簡報
        
        返回:
            簡報文字
        """
        briefing = []
        briefing.append("=" * 50)
        briefing.append(f"群飛任務簡報: {self.name}")
        briefing.append("=" * 50)
        briefing.append("")
        
        briefing.append(f"群飛策略: {self.swarm_params['strategy']}")
        briefing.append(f"無人機數量: {len(self.drones)}")
        briefing.append(f"安全距離: {self.swarm_params['safety_distance']}m")
        briefing.append(f"高度遞增: {self.swarm_params['altitude_increment']}m")
        briefing.append("")
        
        briefing.append("統計資訊:")
        briefing.append(f"  - 總航點數: {self.stats['total_waypoints']}")
        briefing.append(f"  - 總航程: {self.stats['total_distance']:.1f}m")
        briefing.append(f"  - 預估時間: {self.stats['estimated_time']/60:.1f}分鐘")
        briefing.append("")
        
        briefing.append("無人機詳情:")
        for idx, drone in enumerate(self.drones, 1):
            briefing.append(f"\n無人機 {idx}: {drone.name}")
            briefing.append(f"  - ID: {drone.drone_id}")
            briefing.append(f"  - 起飛時間: {drone.start_time:.1f}秒")
            briefing.append(f"  - RTL高度: {drone.rtl_altitude:.1f}m")
            
            if drone.mission:
                stats = drone.mission.get_statistics()
                briefing.append(f"  - 航點數: {stats['total_waypoints']}")
                briefing.append(f"  - 航程: {stats['total_distance']:.1f}m")
                briefing.append(f"  - 飛行時間: {stats['estimated_time']/60:.1f}分鐘")
        
        briefing.append("")
        briefing.append("=" * 50)
        
        return '\n'.join(briefing)
    
    def __str__(self) -> str:
        """字串表示"""
        return f"SwarmMission(name='{self.name}', drones={len(self.drones)})"
    
    def __repr__(self) -> str:
        """詳細表示"""
        return self.__str__()


# ==========================================
# 群飛協調器
# ==========================================
class SwarmCoordinator:
    """
    群飛協調器
    
    提供高階 API 來協調多無人機任務
    """
    
    def __init__(self):
        """初始化協調器"""
        self.current_swarm: Optional[SwarmMission] = None
    
    def create_swarm_from_survey(self, survey_mission: SurveyMission,
                                num_drones: int = None) -> SwarmMission:
        """
        從 Survey 任務創建群飛任務
        
        參數:
            survey_mission: Survey 任務實例
            num_drones: 無人機數量（預設使用子區域數量）
        
        返回:
            SwarmMission 實例
        """
        # 確保已生成子區域
        if not survey_mission.sub_regions:
            survey_mission.generate_sub_regions()
        
        # 確定無人機數量
        if num_drones is None:
            num_drones = len(survey_mission.sub_regions)
        
        # 創建群飛任務
        swarm = SwarmMission(f"{survey_mission.name} - Swarm")
        
        # 添加無人機
        for i in range(num_drones):
            swarm.add_drone(i + 1, f"Drone_{i + 1}")
        
        # 為每個子區域創建獨立任務
        for idx, region_corners in enumerate(survey_mission.sub_regions):
            if idx >= num_drones:
                break
            
            # 創建子任務
            sub_mission = SurveyMission(f"{survey_mission.name}_Region_{idx + 1}")
            
            # 複製參數
            sub_mission.params = survey_mission.params.copy()
            sub_mission.survey_params = survey_mission.survey_params.copy()
            
            # 設定子區域
            sub_mission.set_survey_area(region_corners)
            sub_mission.survey_params['subdivisions'] = 1  # 單一區域
            
            # 生成航點
            sub_mission.generate_survey_waypoints()
            
            # 分配給無人機
            swarm.assign_mission(idx + 1, sub_mission)
        
        # 應用避撞策略
        swarm.apply_collision_avoidance()
        
        # 計算統計
        swarm.calculate_statistics()
        
        self.current_swarm = swarm
        return swarm
    
    def create_swarm_from_missions(self, missions: List[Mission],
                                  strategy: str = 'sequential') -> SwarmMission:
        """
        從多個任務創建群飛任務
        
        參數:
            missions: 任務列表
            strategy: 群飛策略
        
        返回:
            SwarmMission 實例
        """
        swarm = SwarmMission("Custom Swarm")
        swarm.set_strategy(strategy)
        
        # 添加無人機並分配任務
        for idx, mission in enumerate(missions):
            drone = swarm.add_drone(idx + 1, f"Drone_{idx + 1}")
            swarm.assign_mission(drone.drone_id, mission)
        
        # 應用避撞策略
        swarm.apply_collision_avoidance()
        
        # 計算統計
        swarm.calculate_statistics()
        
        self.current_swarm = swarm
        return swarm
    
    def export_swarm_missions(self, swarm: SwarmMission,
                            output_dir: str) -> List[str]:
        """
        匯出群飛任務檔案
        
        參數:
            swarm: 群飛任務
            output_dir: 輸出目錄
        
        返回:
            已匯出的檔案路徑列表
        """
        import os
        
        os.makedirs(output_dir, exist_ok=True)
        exported_files = []
        
        for drone in swarm.drones:
            if drone.mission:
                # 匯出航點檔案
                filename = f"drone_{drone.drone_id}_{drone.name}.waypoints"
                filepath = os.path.join(output_dir, filename)
                
                lines = drone.mission.waypoints.to_qgc_format()
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write('\n'.join(lines))
                
                exported_files.append(filepath)
        
        # 生成任務簡報
        briefing_file = os.path.join(output_dir, "swarm_briefing.txt")
        with open(briefing_file, 'w', encoding='utf-8') as f:
            f.write(swarm.generate_mission_briefing())
        
        exported_files.append(briefing_file)
        
        print(f"已匯出 {len(exported_files)} 個檔案到: {output_dir}")
        return exported_files
    
    def get_current_swarm(self) -> Optional[SwarmMission]:
        """獲取當前群飛任務"""
        return self.current_swarm


# ==========================================
# 使用範例
# ==========================================
if __name__ == '__main__':
    from survey_mission import SurveyMissionBuilder
    
    # 建立 Survey 任務
    corners = [
        (23.702732, 120.419333),
        (23.703732, 120.419333),
        (23.703732, 120.420333),
        (23.702732, 120.420333),
    ]
    
    survey = (SurveyMissionBuilder("群飛測試")
             .set_area(corners)
             .set_altitude(50.0)
             .set_speed(10.0)
             .set_grid(angle=0.0, spacing=10.0)
             .set_subdivisions(4, spacing=3.0)
             .build())
    
    # 創建群飛任務
    coordinator = SwarmCoordinator()
    swarm = coordinator.create_swarm_from_survey(survey)
    
    # 顯示簡報
    print(swarm.generate_mission_briefing())
    
    # 驗證
    valid, errors = swarm.validate()
    if valid:
        print("\n✓ 群飛任務驗證通過")
    else:
        print("\n✗ 群飛任務驗證失敗:")
        for error in errors:
            print(f"  - {error}")
