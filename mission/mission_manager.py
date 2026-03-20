"""
任務管理器模組
負責任務的創建、編輯、儲存、載入等功能
整合航點生成器、避撞系統、障礙物管理等核心組件
"""

import os
import json
from typing import List, Tuple, Optional, Dict, Any
from datetime import datetime
from pathlib import Path

from mission.waypoint import (
    Waypoint, WaypointSequence, MAVCommand,
    create_home_waypoint, create_takeoff_waypoint,
    create_rtl_waypoint, create_change_speed_command
)


# ==========================================
# 任務基類
# ==========================================
class Mission:
    """
    任務基類
    
    表示一個完整的無人機任務，包含航點、參數、元數據等
    """
    
    def __init__(self, name: str = "Untitled Mission"):
        """
        初始化任務
        
        參數:
            name: 任務名稱
        """
        self.name = name
        self.waypoints = WaypointSequence()
        self.created_time = datetime.now()
        self.modified_time = datetime.now()
        
        # 任務參數
        self.params = {
            'altitude': 50.0,          # 飛行高度（公尺）
            'speed': 10.0,             # 飛行速度（公尺/秒）
            'home_lat': 0.0,           # HOME 緯度
            'home_lon': 0.0,           # HOME 經度
            'home_alt': 0.0,           # HOME 高度
        }
        
        # 元數據
        self.metadata = {
            'description': '',         # 任務描述
            'mission_type': 'generic', # 任務類型
            'vehicle_type': 'multirotor', # 飛行器類型
            'version': '1.0',          # 版本
        }
    
    def set_home(self, lat: float, lon: float, alt: float = 0.0):
        """
        設定 HOME 位置
        
        參數:
            lat, lon: 座標
            alt: 高度
        """
        self.params['home_lat'] = lat
        self.params['home_lon'] = lon
        self.params['home_alt'] = alt
    
    def validate(self) -> Tuple[bool, List[str]]:
        """
        驗證任務有效性
        
        返回:
            (是否有效, 錯誤訊息列表)
        """
        errors = []
        
        # 驗證 HOME 點
        if self.params['home_lat'] == 0.0 and self.params['home_lon'] == 0.0:
            errors.append("HOME 位置未設定")
        
        # 驗證航點序列
        valid, wp_errors = self.waypoints.validate()
        errors.extend(wp_errors)
        
        return len(errors) == 0, errors
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        獲取任務統計資訊
        
        返回:
            統計資訊字典
        """
        stats = {
            'name': self.name,
            'total_waypoints': len(self.waypoints),
            'navigation_waypoints': len(self.waypoints.get_navigation_waypoints()),
            'total_distance': self.waypoints.calculate_total_distance(),
            'estimated_time': self.waypoints.estimate_flight_time(self.params['speed']),
            'altitude': self.params['altitude'],
            'speed': self.params['speed'],
        }
        
        # 獲取邊界框
        bbox = self.waypoints.get_bounding_box()
        if bbox:
            stats['bounding_box'] = {
                'min_lat': bbox[0],
                'min_lon': bbox[1],
                'max_lat': bbox[2],
                'max_lon': bbox[3]
            }
        
        return stats
    
    def to_dict(self) -> Dict[str, Any]:
        """
        轉換為字典（用於序列化）
        
        返回:
            任務字典
        """
        return {
            'name': self.name,
            'created_time': self.created_time.isoformat(),
            'modified_time': self.modified_time.isoformat(),
            'params': self.params,
            'metadata': self.metadata,
            'waypoints': self.waypoints.to_qgc_format(),
            'statistics': self.get_statistics()
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Mission':
        """
        從字典創建任務
        
        參數:
            data: 任務字典
        
        返回:
            Mission 實例
        """
        mission = cls(data.get('name', 'Untitled Mission'))
        
        # 載入時間
        if 'created_time' in data:
            mission.created_time = datetime.fromisoformat(data['created_time'])
        if 'modified_time' in data:
            mission.modified_time = datetime.fromisoformat(data['modified_time'])
        
        # 載入參數和元數據
        mission.params.update(data.get('params', {}))
        mission.metadata.update(data.get('metadata', {}))
        
        # 載入航點
        if 'waypoints' in data:
            mission.waypoints = WaypointSequence.from_qgc_format(data['waypoints'])
        
        return mission
    
    def __str__(self) -> str:
        """字串表示"""
        return f"Mission(name='{self.name}', waypoints={len(self.waypoints)})"
    
    def __repr__(self) -> str:
        """詳細表示"""
        return self.__str__()


# ==========================================
# 任務管理器
# ==========================================
class MissionManager:
    """
    任務管理器
    
    管理任務的創建、編輯、儲存、載入等操作
    提供與其他系統組件的整合介面
    """
    
    def __init__(self, missions_dir: Optional[str] = None):
        """
        初始化任務管理器
        
        參數:
            missions_dir: 任務儲存目錄
        """
        if missions_dir is None:
            # 使用預設目錄
            project_root = Path(__file__).parent.parent
            missions_dir = project_root / 'data' / 'missions'
        
        self.missions_dir = Path(missions_dir)
        self.missions_dir.mkdir(parents=True, exist_ok=True)
        
        self.current_mission: Optional[Mission] = None
        self.missions: Dict[str, Mission] = {}
    
    def create_mission(self, name: str, mission_type: str = 'generic') -> Mission:
        """
        創建新任務
        
        參數:
            name: 任務名稱
            mission_type: 任務類型
        
        返回:
            Mission 實例
        """
        mission = Mission(name)
        mission.metadata['mission_type'] = mission_type
        self.missions[name] = mission
        self.current_mission = mission
        return mission
    
    def load_mission(self, filepath: str) -> Optional[Mission]:
        """
        載入任務檔案
        
        參數:
            filepath: 檔案路徑
        
        返回:
            Mission 實例，失敗返回 None
        """
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            mission = Mission.from_dict(data)
            self.missions[mission.name] = mission
            self.current_mission = mission
            return mission
        except Exception as e:
            print(f"載入任務失敗: {e}")
            return None
    
    def save_mission(self, mission: Optional[Mission] = None,
                    filepath: Optional[str] = None) -> Optional[str]:
        """
        儲存任務
        
        參數:
            mission: 要儲存的任務（預設為當前任務）
            filepath: 儲存路徑（預設為任務目錄）
        
        返回:
            儲存的檔案路徑，失敗返回 None
        """
        if mission is None:
            mission = self.current_mission
        
        if mission is None:
            print("沒有任務可儲存")
            return None
        
        try:
            # 更新修改時間
            mission.modified_time = datetime.now()
            
            # 確定儲存路徑
            if filepath is None:
                filename = f"{mission.name}_{mission.created_time.strftime('%Y%m%d_%H%M%S')}.json"
                filepath = self.missions_dir / filename
            
            # 轉換為字串路徑
            filepath = str(filepath)
            
            # 儲存為 JSON
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(mission.to_dict(), f, indent=4, ensure_ascii=False)
            
            print(f"任務已儲存至: {filepath}")
            return filepath
        except Exception as e:
            print(f"儲存任務失敗: {e}")
            return None
    
    def export_waypoints(self, mission: Optional[Mission] = None,
                        filepath: Optional[str] = None,
                        format: str = 'qgc') -> bool:
        """
        匯出航點檔案
        
        參數:
            mission: 要匯出的任務（預設為當前任務）
            filepath: 匯出路徑
            format: 檔案格式（qgc, mission_planner）
        
        返回:
            是否成功
        """
        if mission is None:
            mission = self.current_mission
        
        if mission is None or len(mission.waypoints) == 0:
            print("沒有航點可匯出")
            return False
        
        try:
            if format == 'qgc':
                # QGC WPL 110 格式
                lines = mission.waypoints.to_qgc_format()
                
                if filepath is None:
                    filepath = self.missions_dir / f"{mission.name}.waypoints"
                
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write('\n'.join(lines))
                
                print(f"航點已匯出至: {filepath}")
                return True
            else:
                print(f"不支援的格式: {format}")
                return False
        except Exception as e:
            print(f"匯出航點失敗: {e}")
            return False
    
    def import_waypoints(self, filepath: str, mission_name: Optional[str] = None) -> Optional[Mission]:
        """
        匯入航點檔案
        
        參數:
            filepath: 檔案路徑
            mission_name: 任務名稱（預設使用檔案名）
        
        返回:
            Mission 實例，失敗返回 None
        """
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                lines = [line.strip() for line in f if line.strip()]
            
            # 創建任務
            if mission_name is None:
                mission_name = Path(filepath).stem
            
            mission = self.create_mission(mission_name)
            
            # 載入航點
            mission.waypoints = WaypointSequence.from_qgc_format(lines)
            
            print(f"成功匯入 {len(mission.waypoints)} 個航點")
            return mission
        except Exception as e:
            print(f"匯入航點失敗: {e}")
            return None
    
    def list_missions(self) -> List[str]:
        """
        列出已儲存的任務
        
        返回:
            任務名稱列表
        """
        missions = []
        for filepath in self.missions_dir.glob('*.json'):
            missions.append(filepath.stem)
        return sorted(missions)
    
    def delete_mission(self, name: str) -> bool:
        """
        刪除任務
        
        參數:
            name: 任務名稱
        
        返回:
            是否成功
        """
        # 從記憶體中刪除
        if name in self.missions:
            del self.missions[name]
        
        # 從磁碟中刪除
        for filepath in self.missions_dir.glob(f"{name}*.json"):
            try:
                filepath.unlink()
                print(f"已刪除任務檔案: {filepath}")
                return True
            except Exception as e:
                print(f"刪除任務失敗: {e}")
                return False
        
        return False
    
    def get_current_mission(self) -> Optional[Mission]:
        """
        獲取當前任務
        
        返回:
            當前任務，無則返回 None
        """
        return self.current_mission
    
    def set_current_mission(self, mission: Mission):
        """
        設定當前任務
        
        參數:
            mission: 任務實例
        """
        self.current_mission = mission
        if mission.name not in self.missions:
            self.missions[mission.name] = mission
    
    def build_mission_from_generator(self, corners: List[Tuple[float, float]],
                                    params: Dict[str, Any],
                                    mission_name: str = "Generated Mission") -> Optional[Mission]:
        """
        從航點生成器建立任務
        
        這個方法整合現有的 waypoint_generator
        
        參數:
            corners: 區域邊界點
            params: 飛行參數
            mission_name: 任務名稱
        
        返回:
            Mission 實例，失敗返回 None
        """
        try:
            # 這裡需要導入並使用現有的 waypoint_generator
            # 由於 waypoint_generator 在專案根目錄，需要調整導入路徑
            import sys
            from pathlib import Path
            project_root = Path(__file__).parent.parent
            sys.path.insert(0, str(project_root))
            
            from waypoint_generator import OptimizedWaypointGenerator
            from config import FlightParameters
            
            # 創建任務
            mission = self.create_mission(mission_name, 'survey')
            
            # 設定 HOME
            if corners:
                mission.set_home(corners[0][0], corners[0][1], params.get('altitude', 50.0))
            
            # 創建航點生成器
            generator = OptimizedWaypointGenerator()
            
            # 建立飛行參數
            flight_params = FlightParameters(
                altitude=params.get('altitude', 50.0),
                angle=params.get('angle', 0.0),
                spacing=params.get('spacing', 10.0),
                speed=params.get('speed', 10.0),
                yaw_speed=params.get('yaw_speed', 60.0),
                safety_distance=params.get('safety_distance', 5.0)
            )
            
            # 生成航點
            lines, waypoints = generator.generate_complete_mission(
                corners, flight_params, 0, 1
            )
            
            # 轉換為 WaypointSequence
            mission.waypoints = WaypointSequence.from_qgc_format(lines)
            
            # 更新任務參數
            mission.params.update(params)
            
            print(f"成功生成任務: {len(mission.waypoints)} 個航點")
            return mission
        except Exception as e:
            print(f"從生成器建立任務失敗: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def generate_mission_briefing(self, mission: Optional[Mission] = None) -> str:
        """
        生成任務簡報
        
        參數:
            mission: 要生成簡報的任務（預設為當前任務）
        
        返回:
            簡報文字
        """
        if mission is None:
            mission = self.current_mission
        
        if mission is None:
            return "沒有任務"
        
        stats = mission.get_statistics()
        
        briefing = []
        briefing.append("=" * 50)
        briefing.append(f"任務簡報: {mission.name}")
        briefing.append("=" * 50)
        briefing.append("")
        
        briefing.append(f"任務類型: {mission.metadata.get('mission_type', 'unknown')}")
        briefing.append(f"飛行器類型: {mission.metadata.get('vehicle_type', 'unknown')}")
        briefing.append(f"創建時間: {mission.created_time.strftime('%Y-%m-%d %H:%M:%S')}")
        briefing.append("")
        
        briefing.append("飛行參數:")
        briefing.append(f"  - 飛行高度: {stats['altitude']:.1f} 公尺")
        briefing.append(f"  - 飛行速度: {stats['speed']:.1f} 公尺/秒")
        briefing.append("")
        
        briefing.append("任務統計:")
        briefing.append(f"  - 總航點數: {stats['total_waypoints']}")
        briefing.append(f"  - 導航航點: {stats['navigation_waypoints']}")
        briefing.append(f"  - 總航程: {stats['total_distance']:.1f} 公尺")
        briefing.append(f"  - 預估時間: {stats['estimated_time']/60:.1f} 分鐘")
        briefing.append("")
        
        if 'bounding_box' in stats:
            bbox = stats['bounding_box']
            briefing.append("區域範圍:")
            briefing.append(f"  - 緯度: {bbox['min_lat']:.6f} ~ {bbox['max_lat']:.6f}")
            briefing.append(f"  - 經度: {bbox['min_lon']:.6f} ~ {bbox['max_lon']:.6f}")
            briefing.append("")
        
        briefing.append("=" * 50)
        
        return '\n'.join(briefing)
    
    def __str__(self) -> str:
        """字串表示"""
        return f"MissionManager(missions={len(self.missions)}, current={self.current_mission.name if self.current_mission else 'None'})"
    
    def __repr__(self) -> str:
        """詳細表示"""
        return self.__str__()
