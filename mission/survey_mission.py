"""
Survey 任務模組
專門處理網格掃描（Survey）任務
整合相機配置、區域分割、航點生成等功能
"""

import sys
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any

# 添加專案根目錄到路徑
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from mission.mission_manager import Mission
from mission.waypoint import WaypointSequence


# ==========================================
# Survey 任務類
# ==========================================
class SurveyMission(Mission):
    """
    Survey 任務類
    
    專門處理網格掃描任務，支持相機配置、重疊率設定、
    多區域分割等專業測繪功能
    """
    
    def __init__(self, name: str = "Survey Mission"):
        """
        初始化 Survey 任務
        
        參數:
            name: 任務名稱
        """
        super().__init__(name)
        
        # 更新任務類型
        self.metadata['mission_type'] = 'survey'
        
        # Survey 專用參數
        self.survey_params = {
            # 區域設定
            'corners': [],                  # 區域邊界點
            'subdivisions': 1,              # 子區域數量
            'region_spacing': 0.0,          # 子區域間距（公尺）
            
            # 網格設定
            'scan_angle': 0.0,              # 掃描角度（度）
            'line_spacing': 10.0,           # 航線間距（公尺）
            'reduce_overlap': True,         # 減少重疊（之字形）
            
            # 相機設定
            'camera_name': None,            # 相機名稱
            'use_camera_spacing': False,    # 使用相機自動間距
            'front_overlap': 80.0,          # 前向重疊率（%）
            'side_overlap': 60.0,           # 側向重疊率（%）
            
            # 高級選項
            'terrain_following': False,     # 地形跟隨
            'start_from_left': True,        # 從左開始
        }
        
        # 子區域資訊
        self.sub_regions: List[List[Tuple[float, float]]] = []
    
    def set_survey_area(self, corners: List[Tuple[float, float]]):
        """
        設定測繪區域
        
        參數:
            corners: 區域邊界點列表 [(lat, lon), ...]
        """
        self.survey_params['corners'] = corners
        
        # 自動設定 HOME 為第一個點
        if corners:
            self.set_home(corners[0][0], corners[0][1], self.params['altitude'])
    
    def set_camera(self, camera_name: str, front_overlap: float = 80.0,
                   side_overlap: float = 60.0):
        """
        設定相機參數
        
        參數:
            camera_name: 相機名稱（從資料庫選擇）
            front_overlap: 前向重疊率（%）
            side_overlap: 側向重疊率（%）
        """
        self.survey_params['camera_name'] = camera_name
        self.survey_params['use_camera_spacing'] = True
        self.survey_params['front_overlap'] = front_overlap
        self.survey_params['side_overlap'] = side_overlap
        
        # 計算基於相機的航線間距
        try:
            from sensors import CameraDatabase, CameraCalculator, SurveyParameters

            camera = CameraDatabase.get_camera(camera_name)
            if camera:
                # 創建測繪參數
                survey_params = SurveyParameters(
                    camera=camera,
                    altitude_m=self.params['altitude'],
                    front_overlap_percent=front_overlap,
                    side_overlap_percent=side_overlap,
                    speed_mps=self.params['speed']
                )
                
                # 計算間距
                line_spacing, photo_interval = survey_params.get_auto_spacing()
                
                # 更新參數
                self.survey_params['line_spacing'] = line_spacing
                self.metadata['photo_interval'] = photo_interval
                self.metadata['gsd'] = survey_params.get_gsd()
                
                print(f"相機: {camera}")
                print(f"航線間距: {line_spacing:.2f}m")
                print(f"拍照間隔: {photo_interval:.2f}m")
                print(f"GSD: {survey_params.get_gsd():.4f}m/px")
        except ImportError:
            print("警告: 無法導入 camera_config 模組")
    
    def set_grid_parameters(self, angle: float = 0.0, spacing: float = 10.0,
                          reduce_overlap: bool = True):
        """
        設定網格參數
        
        參數:
            angle: 掃描角度（度）
            spacing: 航線間距（公尺）
            reduce_overlap: 是否減少重疊（之字形）
        """
        self.survey_params['scan_angle'] = angle
        self.survey_params['line_spacing'] = spacing
        self.survey_params['reduce_overlap'] = reduce_overlap
    
    def set_subdivisions(self, count: int, spacing: float = 0.0):
        """
        設定子區域分割
        
        參數:
            count: 子區域數量（1-4）
            spacing: 子區域間距（公尺）
        """
        if not 1 <= count <= 4:
            raise ValueError("子區域數量必須在 1-4 之間")
        
        self.survey_params['subdivisions'] = count
        self.survey_params['region_spacing'] = spacing
    
    def generate_sub_regions(self) -> List[List[Tuple[float, float]]]:
        """
        生成子區域
        
        返回:
            子區域列表
        """
        try:
            from region_divider import RegionDivider
            
            corners = self.survey_params['corners']
            if not corners or len(corners) < 3:
                raise ValueError("區域邊界點不足")
            
            sub_count = self.survey_params['subdivisions']
            spacing = self.survey_params['region_spacing']
            
            # 根據邊界點數量選擇分割方法
            if len(corners) == 4:
                # 四邊形分割
                self.sub_regions = RegionDivider.subdivide_rectangle(
                    corners, sub_count, spacing
                )
            else:
                # 多邊形分割
                self.sub_regions = RegionDivider.subdivide_polygon(
                    corners, sub_count, spacing
                )
            
            print(f"成功生成 {len(self.sub_regions)} 個子區域")
            return self.sub_regions
        except ImportError:
            print("警告: 無法導入 region_divider 模組")
            return [self.survey_params['corners']]
    
    def generate_survey_waypoints(self) -> bool:
        """
        生成 Survey 航點
        
        返回:
            是否成功
        """
        try:
            from waypoint_generator import OptimizedWaypointGenerator
            from config import FlightParameters
            from collision_avoidance import CollisionAvoidanceSystem
            
            # 生成子區域
            if not self.sub_regions:
                self.generate_sub_regions()
            
            if not self.sub_regions:
                print("無法生成子區域")
                return False
            
            # 創建航點生成器
            generator = OptimizedWaypointGenerator()
            
            # 建立飛行參數
            flight_params = FlightParameters(
                altitude=self.params['altitude'],
                angle=self.survey_params['scan_angle'],
                spacing=self.survey_params['line_spacing'],
                speed=self.params['speed'],
                yaw_speed=self.params.get('yaw_speed', 60.0),
                safety_distance=self.params.get('safety_distance', 5.0)
            )
            
            # 清空現有航點
            self.waypoints.clear()
            
            # 為每個子區域生成航點
            all_lines = []
            total_regions = len(self.sub_regions)
            
            for idx, region_corners in enumerate(self.sub_regions):
                # 確定起始方向
                start_from_left = (
                    (idx % 2 == 0) if self.survey_params['reduce_overlap']
                    else self.survey_params['start_from_left']
                )
                
                # 生成完整任務
                lines, waypoints = generator.generate_complete_mission(
                    region_corners,
                    flight_params,
                    idx,
                    total_regions,
                    start_from_left,
                    loiter_time=0.0
                )
                
                # 如果是多區域，合併航點
                if total_regions > 1:
                    if idx == 0:
                        all_lines = lines
                    else:
                        # 移除重複的標頭和 HOME 點
                        all_lines.extend(lines[2:])
            
            # 轉換為 WaypointSequence
            if total_regions == 1:
                self.waypoints = WaypointSequence.from_qgc_format(all_lines)
            else:
                # 多區域需要更新序列號
                self.waypoints = WaypointSequence.from_qgc_format(all_lines)
                self.waypoints._update_sequence_numbers()
            
            print(f"成功生成 Survey 航點: {len(self.waypoints)} 個")
            return True
        except Exception as e:
            print(f"生成 Survey 航點失敗: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def calculate_coverage_area(self) -> float:
        """
        計算覆蓋面積
        
        返回:
            面積（平方公尺）
        """
        try:
            corners = self.survey_params['corners']
            if len(corners) < 3:
                return 0.0
            
            # 使用 Shoelace 公式計算多邊形面積
            # 需要先將經緯度轉換為平面座標
            from math_utils import latlon_to_meters
            
            if not corners:
                return 0.0
            
            # 使用第一個點作為參考點
            ref_lat, ref_lon = corners[0]
            
            # 轉換為平面座標
            xy_points = []
            for lat, lon in corners:
                x, y = latlon_to_meters(lat, lon, ref_lat, ref_lon)
                xy_points.append((x, y))
            
            # Shoelace 公式
            area = 0.0
            n = len(xy_points)
            for i in range(n):
                j = (i + 1) % n
                area += xy_points[i][0] * xy_points[j][1]
                area -= xy_points[j][0] * xy_points[i][1]
            
            return abs(area) / 2.0
        except Exception as e:
            print(f"計算覆蓋面積失敗: {e}")
            return 0.0
    
    def estimate_photo_count(self) -> int:
        """
        估算照片數量
        
        返回:
            預估照片數
        """
        try:
            if not self.survey_params['use_camera_spacing']:
                return 0
            
            from sensors import CameraDatabase, CameraCalculator

            camera_name = self.survey_params['camera_name']
            if not camera_name:
                return 0
            
            camera = CameraDatabase.get_camera(camera_name)
            if not camera:
                return 0
            
            area = self.calculate_coverage_area()
            if area <= 0:
                return 0
            
            # 計算需要的照片數
            photo_count = CameraCalculator.calculate_required_photos(
                area,
                self.params['altitude'],
                camera,
                self.survey_params['front_overlap'],
                self.survey_params['side_overlap']
            )
            
            return photo_count
        except ImportError:
            return 0
    
    def get_survey_statistics(self) -> Dict[str, Any]:
        """
        獲取 Survey 統計資訊
        
        返回:
            統計資訊字典
        """
        stats = self.get_statistics()
        
        # 添加 Survey 專用資訊
        stats['survey'] = {
            'area': self.calculate_coverage_area(),
            'subdivisions': self.survey_params['subdivisions'],
            'region_spacing': self.survey_params['region_spacing'],
            'scan_angle': self.survey_params['scan_angle'],
            'line_spacing': self.survey_params['line_spacing'],
        }
        
        # 相機資訊
        if self.survey_params['use_camera_spacing']:
            stats['camera'] = {
                'name': self.survey_params['camera_name'],
                'front_overlap': self.survey_params['front_overlap'],
                'side_overlap': self.survey_params['side_overlap'],
                'estimated_photos': self.estimate_photo_count(),
            }
            
            if 'gsd' in self.metadata:
                stats['camera']['gsd'] = self.metadata['gsd']
            if 'photo_interval' in self.metadata:
                stats['camera']['photo_interval'] = self.metadata['photo_interval']
        
        return stats
    
    def to_dict(self) -> Dict[str, Any]:
        """
        轉換為字典（用於序列化）
        
        返回:
            任務字典
        """
        data = super().to_dict()
        
        # 添加 Survey 專用資料
        data['survey_params'] = self.survey_params
        data['sub_regions'] = self.sub_regions
        data['survey_statistics'] = self.get_survey_statistics()
        
        return data
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'SurveyMission':
        """
        從字典創建 Survey 任務
        
        參數:
            data: 任務字典
        
        返回:
            SurveyMission 實例
        """
        mission = cls(data.get('name', 'Survey Mission'))
        
        # 載入基本資料（使用父類方法）
        base_mission = Mission.from_dict(data)
        mission.created_time = base_mission.created_time
        mission.modified_time = base_mission.modified_time
        mission.params = base_mission.params
        mission.metadata = base_mission.metadata
        mission.waypoints = base_mission.waypoints
        
        # 載入 Survey 專用資料
        if 'survey_params' in data:
            mission.survey_params.update(data['survey_params'])
        
        if 'sub_regions' in data:
            mission.sub_regions = data['sub_regions']
        
        return mission
    
    def __str__(self) -> str:
        """字串表示"""
        area = self.calculate_coverage_area()
        return f"SurveyMission(name='{self.name}', area={area:.1f}m², waypoints={len(self.waypoints)})"


# ==========================================
# Survey 任務建立器
# ==========================================
class SurveyMissionBuilder:
    """
    Survey 任務建立器
    
    提供流暢的 API 來建立 Survey 任務
    """
    
    def __init__(self, name: str = "Survey Mission"):
        """
        初始化建立器
        
        參數:
            name: 任務名稱
        """
        self.mission = SurveyMission(name)
    
    def set_area(self, corners: List[Tuple[float, float]]) -> 'SurveyMissionBuilder':
        """設定區域"""
        self.mission.set_survey_area(corners)
        return self
    
    def set_altitude(self, altitude: float) -> 'SurveyMissionBuilder':
        """設定高度"""
        self.mission.params['altitude'] = altitude
        return self
    
    def set_speed(self, speed: float) -> 'SurveyMissionBuilder':
        """設定速度"""
        self.mission.params['speed'] = speed
        return self
    
    def set_camera(self, camera_name: str, front_overlap: float = 80.0,
                  side_overlap: float = 60.0) -> 'SurveyMissionBuilder':
        """設定相機"""
        self.mission.set_camera(camera_name, front_overlap, side_overlap)
        return self
    
    def set_grid(self, angle: float = 0.0, spacing: float = 10.0,
                reduce_overlap: bool = True) -> 'SurveyMissionBuilder':
        """設定網格"""
        self.mission.set_grid_parameters(angle, spacing, reduce_overlap)
        return self
    
    def set_subdivisions(self, count: int, spacing: float = 0.0) -> 'SurveyMissionBuilder':
        """設定子區域"""
        self.mission.set_subdivisions(count, spacing)
        return self
    
    def build(self) -> SurveyMission:
        """
        建立任務
        
        返回:
            SurveyMission 實例
        """
        # 生成航點
        self.mission.generate_survey_waypoints()
        return self.mission


# ==========================================
# 使用範例
# ==========================================
if __name__ == '__main__':
    # 建立 Survey 任務
    builder = SurveyMissionBuilder("測試 Survey")
    
    # 定義區域（四邊形）
    corners = [
        (23.702732, 120.419333),  # 左下
        (23.703732, 120.419333),  # 右下
        (23.703732, 120.420333),  # 右上
        (23.702732, 120.420333),  # 左上
    ]
    
    # 建立任務
    mission = (builder
              .set_area(corners)
              .set_altitude(50.0)
              .set_speed(10.0)
              .set_grid(angle=0.0, spacing=10.0)
              .set_subdivisions(2, spacing=3.0)
              .build())
    
    # 顯示統計
    print(mission)
    stats = mission.get_survey_statistics()
    print(f"覆蓋面積: {stats['survey']['area']:.1f} m²")
    print(f"總航程: {stats['total_distance']:.1f} m")
    print(f"預估時間: {stats['estimated_time']/60:.1f} 分鐘")
