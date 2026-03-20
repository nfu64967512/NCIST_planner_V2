"""
MAVLink 匯出器模組
提供多種格式的任務檔案匯出功能
支持 QGC、Mission Planner 等格式
"""

import os
import json
from typing import List, Optional, Dict, Any, Tuple
from datetime import datetime
from pathlib import Path

from mission.mission_manager import Mission
from mission.waypoint import Waypoint, WaypointSequence, MAVCommand


# ==========================================
# 匯出格式枚舉
# ==========================================
class ExportFormat:
    """匯出格式"""
    QGC_WPL = 'qgc_wpl'                # QGC WPL 110 格式
    MISSION_PLANNER = 'mission_planner' # Mission Planner 格式
    JSON = 'json'                       # JSON 格式
    KML = 'kml'                         # KML 格式（Google Earth）
    GPX = 'gpx'                         # GPX 格式（GPS 軌跡）


# ==========================================
# MAVLink 匯出器
# ==========================================
class MAVLinkExporter:
    """
    MAVLink 匯出器
    
    提供多種格式的任務匯出功能
    """
    
    def __init__(self):
        """初始化匯出器"""
        pass
    
    def export_mission(self, mission: Mission, filepath: str,
                      format: str = ExportFormat.QGC_WPL,
                      **kwargs) -> bool:
        """
        匯出任務
        
        參數:
            mission: 任務實例
            filepath: 輸出路徑
            format: 匯出格式
            **kwargs: 格式特定參數
        
        返回:
            是否成功
        """
        try:
            # 確保目錄存在
            os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)
            
            # 根據格式選擇匯出方法
            if format == ExportFormat.QGC_WPL:
                return self._export_qgc_wpl(mission, filepath, **kwargs)
            elif format == ExportFormat.MISSION_PLANNER:
                return self._export_mission_planner(mission, filepath, **kwargs)
            elif format == ExportFormat.JSON:
                return self._export_json(mission, filepath, **kwargs)
            elif format == ExportFormat.KML:
                return self._export_kml(mission, filepath, **kwargs)
            elif format == ExportFormat.GPX:
                return self._export_gpx(mission, filepath, **kwargs)
            else:
                print(f"不支援的匯出格式: {format}")
                return False
        except Exception as e:
            print(f"匯出任務失敗: {e}")
            return False
    
    def _export_qgc_wpl(self, mission: Mission, filepath: str, **kwargs) -> bool:
        """
        匯出為 QGC WPL 110 格式
        
        參數:
            mission: 任務實例
            filepath: 輸出路徑
        
        返回:
            是否成功
        """
        try:
            lines = mission.waypoints.to_qgc_format()
            
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write('\n'.join(lines))
            
            print(f"成功匯出 QGC WPL 格式: {filepath}")
            return True
        except Exception as e:
            print(f"匯出 QGC WPL 格式失敗: {e}")
            return False
    
    def _export_mission_planner(self, mission: Mission, filepath: str, **kwargs) -> bool:
        """
        匯出為 Mission Planner 格式
        
        Mission Planner 使用與 QGC 相同的格式，但可能有細微差異
        
        參數:
            mission: 任務實例
            filepath: 輸出路徑
        
        返回:
            是否成功
        """
        # Mission Planner 格式基本與 QGC 相同
        return self._export_qgc_wpl(mission, filepath, **kwargs)
    
    def _export_json(self, mission: Mission, filepath: str,
                    include_metadata: bool = True, **kwargs) -> bool:
        """
        匯出為 JSON 格式
        
        參數:
            mission: 任務實例
            filepath: 輸出路徑
            include_metadata: 是否包含元數據
        
        返回:
            是否成功
        """
        try:
            data = mission.to_dict()
            
            if not include_metadata:
                # 移除元數據
                data.pop('metadata', None)
                data.pop('statistics', None)
            
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=4, ensure_ascii=False)
            
            print(f"成功匯出 JSON 格式: {filepath}")
            return True
        except Exception as e:
            print(f"匯出 JSON 格式失敗: {e}")
            return False
    
    def _export_kml(self, mission: Mission, filepath: str,
                   name: Optional[str] = None, **kwargs) -> bool:
        """
        匯出為 KML 格式（Google Earth）
        
        參數:
            mission: 任務實例
            filepath: 輸出路徑
            name: 顯示名稱
        
        返回:
            是否成功
        """
        try:
            if name is None:
                name = mission.name
            
            # 獲取導航航點
            nav_waypoints = mission.waypoints.get_navigation_waypoints()
            
            # KML 模板
            kml = []
            kml.append('<?xml version="1.0" encoding="UTF-8"?>')
            kml.append('<kml xmlns="http://www.opengis.net/kml/2.2">')
            kml.append('  <Document>')
            kml.append(f'    <name>{name}</name>')
            kml.append('    <description>UAV Mission Path</description>')
            
            # 樣式定義
            kml.append('    <Style id="waypointStyle">')
            kml.append('      <IconStyle>')
            kml.append('        <color>ff00ff00</color>')
            kml.append('        <scale>0.8</scale>')
            kml.append('      </IconStyle>')
            kml.append('    </Style>')
            
            kml.append('    <Style id="lineStyle">')
            kml.append('      <LineStyle>')
            kml.append('        <color>ff0000ff</color>')
            kml.append('        <width>2</width>')
            kml.append('      </LineStyle>')
            kml.append('    </Style>')
            
            # 航點標記
            for wp in nav_waypoints:
                kml.append('    <Placemark>')
                kml.append(f'      <name>WP{wp.seq}</name>')
                kml.append(f'      <description>Alt: {wp.alt:.1f}m</description>')
                kml.append('      <styleUrl>#waypointStyle</styleUrl>')
                kml.append('      <Point>')
                kml.append(f'        <coordinates>{wp.lon},{wp.lat},{wp.alt}</coordinates>')
                kml.append('      </Point>')
                kml.append('    </Placemark>')
            
            # 路徑線
            kml.append('    <Placemark>')
            kml.append('      <name>Flight Path</name>')
            kml.append('      <styleUrl>#lineStyle</styleUrl>')
            kml.append('      <LineString>')
            kml.append('        <tessellate>1</tessellate>')
            kml.append('        <coordinates>')
            
            for wp in nav_waypoints:
                kml.append(f'          {wp.lon},{wp.lat},{wp.alt}')
            
            kml.append('        </coordinates>')
            kml.append('      </LineString>')
            kml.append('    </Placemark>')
            
            kml.append('  </Document>')
            kml.append('</kml>')
            
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write('\n'.join(kml))
            
            print(f"成功匯出 KML 格式: {filepath}")
            return True
        except Exception as e:
            print(f"匯出 KML 格式失敗: {e}")
            return False
    
    def _export_gpx(self, mission: Mission, filepath: str,
                   name: Optional[str] = None, **kwargs) -> bool:
        """
        匯出為 GPX 格式（GPS 軌跡）
        
        參數:
            mission: 任務實例
            filepath: 輸出路徑
            name: 軌跡名稱
        
        返回:
            是否成功
        """
        try:
            if name is None:
                name = mission.name
            
            # 獲取導航航點
            nav_waypoints = mission.waypoints.get_navigation_waypoints()
            
            # GPX 模板
            gpx = []
            gpx.append('<?xml version="1.0" encoding="UTF-8"?>')
            gpx.append('<gpx version="1.1" creator="UAV Path Planner"')
            gpx.append('     xmlns="http://www.topografix.com/GPX/1/1">')
            
            # 元數據
            gpx.append('  <metadata>')
            gpx.append(f'    <name>{name}</name>')
            gpx.append(f'    <time>{datetime.now().isoformat()}</time>')
            gpx.append('  </metadata>')
            
            # 航點
            for wp in nav_waypoints:
                gpx.append(f'  <wpt lat="{wp.lat}" lon="{wp.lon}">')
                gpx.append(f'    <ele>{wp.alt}</ele>')
                gpx.append(f'    <name>WP{wp.seq}</name>')
                gpx.append(f'    <desc>Waypoint {wp.seq}</desc>')
                gpx.append('  </wpt>')
            
            # 軌跡
            gpx.append('  <trk>')
            gpx.append(f'    <name>{name} Track</name>')
            gpx.append('    <trkseg>')
            
            for wp in nav_waypoints:
                gpx.append(f'      <trkpt lat="{wp.lat}" lon="{wp.lon}">')
                gpx.append(f'        <ele>{wp.alt}</ele>')
                gpx.append('      </trkpt>')
            
            gpx.append('    </trkseg>')
            gpx.append('  </trk>')
            gpx.append('</gpx>')
            
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write('\n'.join(gpx))
            
            print(f"成功匯出 GPX 格式: {filepath}")
            return True
        except Exception as e:
            print(f"匯出 GPX 格式失敗: {e}")
            return False
    
    def export_mission_briefing(self, mission: Mission, filepath: str) -> bool:
        """
        匯出任務簡報
        
        參數:
            mission: 任務實例
            filepath: 輸出路徑
        
        返回:
            是否成功
        """
        try:
            from mission.mission_manager import MissionManager
            
            manager = MissionManager()
            briefing = manager.generate_mission_briefing(mission)
            
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(briefing)
            
            print(f"成功匯出任務簡報: {filepath}")
            return True
        except Exception as e:
            print(f"匯出任務簡報失敗: {e}")
            return False
    
    def export_batch(self, missions: List[Mission], output_dir: str,
                    format: str = ExportFormat.QGC_WPL,
                    include_briefing: bool = True) -> List[str]:
        """
        批量匯出任務
        
        參數:
            missions: 任務列表
            output_dir: 輸出目錄
            format: 匯出格式
            include_briefing: 是否包含簡報
        
        返回:
            已匯出的檔案路徑列表
        """
        os.makedirs(output_dir, exist_ok=True)
        exported_files = []
        
        for idx, mission in enumerate(missions, 1):
            # 確定檔案名
            ext = self._get_extension_for_format(format)
            filename = f"{mission.name}_{idx}{ext}"
            filepath = os.path.join(output_dir, filename)
            
            # 匯出任務
            if self.export_mission(mission, filepath, format):
                exported_files.append(filepath)
            
            # 匯出簡報
            if include_briefing:
                briefing_file = os.path.join(output_dir, f"{mission.name}_{idx}_briefing.txt")
                if self.export_mission_briefing(mission, briefing_file):
                    exported_files.append(briefing_file)
        
        print(f"批量匯出完成: {len(exported_files)} 個檔案")
        return exported_files
    
    def _get_extension_for_format(self, format: str) -> str:
        """
        獲取格式對應的副檔名
        
        參數:
            format: 格式名稱
        
        返回:
            副檔名
        """
        extensions = {
            ExportFormat.QGC_WPL: '.waypoints',
            ExportFormat.MISSION_PLANNER: '.waypoints',
            ExportFormat.JSON: '.json',
            ExportFormat.KML: '.kml',
            ExportFormat.GPX: '.gpx',
        }
        return extensions.get(format, '.txt')


# ==========================================
# 匯出助手
# ==========================================
class ExportHelper:
    """
    匯出助手
    
    提供便捷的匯出方法
    """
    
    @staticmethod
    def quick_export_qgc(mission: Mission, output_dir: str = None) -> str:
        """
        快速匯出為 QGC 格式
        
        參數:
            mission: 任務實例
            output_dir: 輸出目錄（預設為當前目錄）
        
        返回:
            輸出檔案路徑
        """
        if output_dir is None:
            output_dir = '.'
        
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"{mission.name}_{timestamp}.waypoints"
        filepath = os.path.join(output_dir, filename)
        
        exporter = MAVLinkExporter()
        exporter.export_mission(mission, filepath, ExportFormat.QGC_WPL)
        
        return filepath
    
    @staticmethod
    def export_with_briefing(mission: Mission, output_dir: str) -> Tuple[str, str]:
        """
        匯出任務和簡報
        
        參數:
            mission: 任務實例
            output_dir: 輸出目錄
        
        返回:
            (任務檔案路徑, 簡報檔案路徑)
        """
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # 匯出任務
        mission_file = os.path.join(output_dir, f"{mission.name}_{timestamp}.waypoints")
        exporter = MAVLinkExporter()
        exporter.export_mission(mission, mission_file, ExportFormat.QGC_WPL)
        
        # 匯出簡報
        briefing_file = os.path.join(output_dir, f"{mission.name}_{timestamp}_briefing.txt")
        exporter.export_mission_briefing(mission, briefing_file)
        
        return mission_file, briefing_file
    
    @staticmethod
    def export_multiple_formats(mission: Mission, output_dir: str,
                               formats: List[str] = None) -> Dict[str, str]:
        """
        匯出多種格式
        
        參數:
            mission: 任務實例
            output_dir: 輸出目錄
            formats: 格式列表（預設為所有格式）
        
        返回:
            {格式: 檔案路徑} 字典
        """
        if formats is None:
            formats = [
                ExportFormat.QGC_WPL,
                ExportFormat.JSON,
                ExportFormat.KML,
                ExportFormat.GPX
            ]
        
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        exporter = MAVLinkExporter()
        
        output_files = {}
        
        for fmt in formats:
            ext = exporter._get_extension_for_format(fmt)
            filename = f"{mission.name}_{timestamp}{ext}"
            filepath = os.path.join(output_dir, filename)
            
            if exporter.export_mission(mission, filepath, fmt):
                output_files[fmt] = filepath
        
        return output_files


# ==========================================
# 使用範例
# ==========================================
if __name__ == '__main__':
    from mission.mission_manager import MissionManager
    
    # 創建任務管理器
    manager = MissionManager()
    
    # 假設我們有一個任務
    # mission = manager.load_mission("example_mission.json")
    
    # 或者創建一個簡單的任務用於測試
    mission = manager.create_mission("Test Mission")
    from mission.waypoint import create_home_waypoint, create_navigation_waypoint
    
    # 添加一些航點
    mission.waypoints.add(create_home_waypoint(23.702732, 120.419333))
    mission.waypoints.add(create_navigation_waypoint(23.703732, 120.419333, 50.0, 1))
    mission.waypoints.add(create_navigation_waypoint(23.703732, 120.420333, 50.0, 2))
    
    # 使用匯出器
    exporter = MAVLinkExporter()
    
    # 匯出為 QGC 格式
    exporter.export_mission(mission, "test_mission.waypoints", ExportFormat.QGC_WPL)
    
    # 匯出為 KML 格式
    exporter.export_mission(mission, "test_mission.kml", ExportFormat.KML)
    
    # 使用助手快速匯出
    from mission.mavlink_exporter import ExportHelper
    
    filepath = ExportHelper.quick_export_qgc(mission, "output")
    print(f"已匯出至: {filepath}")
    
    # 匯出多種格式
    files = ExportHelper.export_multiple_formats(mission, "output")
    print(f"已匯出 {len(files)} 種格式:")
    for fmt, path in files.items():
        print(f"  - {fmt}: {path}")
