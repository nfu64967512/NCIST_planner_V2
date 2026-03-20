"""
航點資料結構模組
定義航點、航點序列等核心資料結構
支持 MAVLink 標準命令
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Any
from enum import IntEnum
import math


# ==========================================
# MAVLink 命令定義
# ==========================================
class MAVCommand(IntEnum):
    """MAVLink 命令枚舉"""
    # 導航命令
    NAV_WAYPOINT = 16           # 導航到航點
    NAV_LOITER_UNLIM = 17       # 無限懸停
    NAV_LOITER_TURNS = 18       # 懸停N圈
    NAV_LOITER_TIME = 19        # 懸停N秒
    NAV_RETURN_TO_LAUNCH = 20   # 返回起點
    NAV_LAND = 21               # 降落
    NAV_TAKEOFF = 22            # 起飛
    
    # 條件命令
    CONDITION_DELAY = 112       # 延遲
    CONDITION_DISTANCE = 114    # 距離條件
    CONDITION_YAW = 115         # 航向條件
    
    # DO 命令
    DO_CHANGE_SPEED = 178       # 改變速度
    DO_SET_HOME = 179           # 設定起點
    DO_SET_RELAY = 181         # 設定繼電器
    DO_REPEAT_RELAY = 182      # 重複繼電器
    DO_SET_SERVO = 183         # 設定伺服馬達
    DO_REPEAT_SERVO = 184      # 重複伺服馬達
    DO_SET_ROI = 201           # 設定興趣區域
    DO_DIGICAM_CONTROL = 203   # 數位相機控制
    DO_MOUNT_CONTROL = 205     # 雲台控制
    DO_SET_CAM_TRIGG_DIST = 206 # 設定相機觸發距離


class CoordinateFrame(IntEnum):
    """座標系枚舉"""
    GLOBAL = 0                  # WGS84，高度為海拔
    LOCAL_NED = 1              # 本地北東地座標系
    MISSION = 2                # 任務座標系
    GLOBAL_RELATIVE_ALT = 3    # WGS84，高度相對起點
    LOCAL_ENU = 4              # 本地東北上座標系
    GLOBAL_INT = 5             # WGS84（整數）
    GLOBAL_RELATIVE_ALT_INT = 6 # WGS84（整數），相對高度
    LOCAL_OFFSET_NED = 7       # 本地偏移NED
    BODY_NED = 8              # 機體NED
    BODY_OFFSET_NED = 9       # 機體偏移NED
    GLOBAL_TERRAIN_ALT = 10    # WGS84，地形高度
    GLOBAL_TERRAIN_ALT_INT = 11 # WGS84（整數），地形高度


# ==========================================
# 航點資料類
# ==========================================
@dataclass
class Waypoint:
    """
    航點資料類
    
    表示單個航點的完整資訊，兼容 MAVLink 協議
    """
    # 基本資訊
    seq: int                                    # 序列號
    command: MAVCommand                         # MAVLink 命令
    
    # 座標資訊
    lat: float = 0.0                           # 緯度（度）
    lon: float = 0.0                           # 經度（度）
    alt: float = 0.0                           # 高度（公尺）
    
    # 座標系
    frame: CoordinateFrame = CoordinateFrame.GLOBAL_RELATIVE_ALT
    
    # 命令參數
    param1: float = 0.0                        # 參數1
    param2: float = 0.0                        # 參數2
    param3: float = 0.0                        # 參數3
    param4: float = 0.0                        # 參數4（通常是航向角）
    
    # 狀態標記
    current: int = 0                           # 是否為當前航點
    autocontinue: int = 1                      # 是否自動繼續
    
    # 元數據
    metadata: Dict[str, Any] = field(default_factory=dict)  # 額外元數據
    
    def __post_init__(self):
        """初始化後驗證"""
        if not isinstance(self.command, MAVCommand):
            self.command = MAVCommand(self.command)
        if not isinstance(self.frame, CoordinateFrame):
            self.frame = CoordinateFrame(self.frame)
    
    def to_qgc_line(self) -> str:
        """
        轉換為 QGC WPL 110 格式的一行
        
        返回:
            QGC 格式字串
        """
        return (f"{self.seq}\t{self.current}\t{self.frame}\t{self.command}\t"
               f"{self.param1}\t{self.param2}\t{self.param3}\t{self.param4}\t"
               f"{self.lat:.6f}\t{self.lon:.6f}\t{self.alt:.2f}\t{self.autocontinue}")
    
    @classmethod
    def from_qgc_line(cls, line: str) -> Optional['Waypoint']:
        """
        從 QGC WPL 110 格式的一行創建航點
        
        參數:
            line: QGC 格式字串
        
        返回:
            Waypoint 實例，失敗返回 None
        """
        try:
            parts = line.strip().split('\t')
            if len(parts) < 12:
                return None
            
            return cls(
                seq=int(parts[0]),
                current=int(parts[1]),
                frame=CoordinateFrame(int(parts[2])),
                command=MAVCommand(int(parts[3])),
                param1=float(parts[4]),
                param2=float(parts[5]),
                param3=float(parts[6]),
                param4=float(parts[7]),
                lat=float(parts[8]),
                lon=float(parts[9]),
                alt=float(parts[10]),
                autocontinue=int(parts[11])
            )
        except (ValueError, IndexError):
            return None
    
    def distance_to(self, other: 'Waypoint') -> float:
        """
        計算到另一個航點的距離
        
        參數:
            other: 另一個航點
        
        返回:
            距離（公尺）
        """
        # 使用 Haversine 公式
        R = 6378137.0  # 地球半徑（公尺）
        
        lat1 = math.radians(self.lat)
        lat2 = math.radians(other.lat)
        dlat = math.radians(other.lat - self.lat)
        dlon = math.radians(other.lon - self.lon)
        
        a = (math.sin(dlat / 2) ** 2 + 
             math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        distance = R * c
        
        # 加上高度差
        dalt = other.alt - self.alt
        distance = math.sqrt(distance ** 2 + dalt ** 2)
        
        return distance
    
    def bearing_to(self, other: 'Waypoint') -> float:
        """
        計算到另一個航點的方位角
        
        參數:
            other: 另一個航點
        
        返回:
            方位角（度，0-360）
        """
        lat1 = math.radians(self.lat)
        lat2 = math.radians(other.lat)
        dlon = math.radians(other.lon - self.lon)
        
        y = math.sin(dlon) * math.cos(lat2)
        x = (math.cos(lat1) * math.sin(lat2) - 
             math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360
    
    def copy(self) -> 'Waypoint':
        """創建航點的深拷貝"""
        return Waypoint(
            seq=self.seq,
            command=self.command,
            lat=self.lat,
            lon=self.lon,
            alt=self.alt,
            frame=self.frame,
            param1=self.param1,
            param2=self.param2,
            param3=self.param3,
            param4=self.param4,
            current=self.current,
            autocontinue=self.autocontinue,
            metadata=self.metadata.copy()
        )
    
    def __str__(self) -> str:
        """字串表示"""
        return f"Waypoint(seq={self.seq}, cmd={self.command.name}, pos=({self.lat:.6f}, {self.lon:.6f}, {self.alt:.2f}))"
    
    def __repr__(self) -> str:
        """詳細表示"""
        return self.__str__()


# ==========================================
# 航點序列類
# ==========================================
class WaypointSequence:
    """
    航點序列類
    
    管理一組航點，提供操作、查詢、統計功能
    """
    
    def __init__(self, waypoints: Optional[List[Waypoint]] = None):
        """
        初始化航點序列
        
        參數:
            waypoints: 初始航點列表
        """
        self.waypoints: List[Waypoint] = waypoints or []
        self._update_sequence_numbers()
    
    def _update_sequence_numbers(self):
        """更新所有航點的序列號"""
        for i, wp in enumerate(self.waypoints):
            wp.seq = i
    
    def add(self, waypoint: Waypoint):
        """
        添加航點
        
        參數:
            waypoint: 要添加的航點
        """
        self.waypoints.append(waypoint)
        self._update_sequence_numbers()
    
    def insert(self, index: int, waypoint: Waypoint):
        """
        在指定位置插入航點
        
        參數:
            index: 插入位置
            waypoint: 要插入的航點
        """
        self.waypoints.insert(index, waypoint)
        self._update_sequence_numbers()
    
    def remove(self, index: int) -> Optional[Waypoint]:
        """
        移除指定位置的航點
        
        參數:
            index: 位置索引
        
        返回:
            被移除的航點，失敗返回 None
        """
        if 0 <= index < len(self.waypoints):
            waypoint = self.waypoints.pop(index)
            self._update_sequence_numbers()
            return waypoint
        return None
    
    def clear(self):
        """清空所有航點"""
        self.waypoints.clear()
    
    def get(self, index: int) -> Optional[Waypoint]:
        """
        獲取指定位置的航點
        
        參數:
            index: 位置索引
        
        返回:
            航點，失敗返回 None
        """
        if 0 <= index < len(self.waypoints):
            return self.waypoints[index]
        return None
    
    def get_navigation_waypoints(self) -> List[Waypoint]:
        """
        獲取所有導航航點（排除 DO/CONDITION 命令）
        
        返回:
            導航航點列表
        """
        return [wp for wp in self.waypoints 
               if wp.command in (MAVCommand.NAV_WAYPOINT,
                               MAVCommand.NAV_TAKEOFF,
                               MAVCommand.NAV_LAND,
                               MAVCommand.NAV_LOITER_TIME,
                               MAVCommand.NAV_RETURN_TO_LAUNCH)]
    
    def calculate_total_distance(self) -> float:
        """
        計算總航程距離
        
        返回:
            總距離（公尺）
        """
        nav_wps = self.get_navigation_waypoints()
        if len(nav_wps) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(len(nav_wps) - 1):
            total_distance += nav_wps[i].distance_to(nav_wps[i + 1])
        
        return total_distance
    
    def estimate_flight_time(self, cruise_speed: float = 10.0) -> float:
        """
        估算飛行時間
        
        參數:
            cruise_speed: 巡航速度（公尺/秒）
        
        返回:
            預估飛行時間（秒）
        """
        distance = self.calculate_total_distance()
        
        # 基本飛行時間
        flight_time = distance / cruise_speed if cruise_speed > 0 else 0
        
        # 加上懸停時間
        for wp in self.waypoints:
            if wp.command == MAVCommand.NAV_LOITER_TIME:
                flight_time += wp.param1
        
        # 加上轉向時間（簡化估算）
        nav_wps = self.get_navigation_waypoints()
        turn_time = (len(nav_wps) - 1) * 2.0  # 每次轉向約2秒
        
        return flight_time + turn_time
    
    def get_bounding_box(self) -> Optional[Tuple[float, float, float, float]]:
        """
        獲取航點的邊界框
        
        返回:
            (min_lat, min_lon, max_lat, max_lon) 或 None
        """
        nav_wps = self.get_navigation_waypoints()
        if not nav_wps:
            return None
        
        lats = [wp.lat for wp in nav_wps]
        lons = [wp.lon for wp in nav_wps]
        
        return (min(lats), min(lons), max(lats), max(lons))
    
    def to_qgc_format(self) -> List[str]:
        """
        轉換為 QGC WPL 110 格式
        
        返回:
            QGC 格式的字串列表
        """
        lines = ["QGC WPL 110"]
        lines.extend([wp.to_qgc_line() for wp in self.waypoints])
        return lines
    
    @classmethod
    def from_qgc_format(cls, lines: List[str]) -> 'WaypointSequence':
        """
        從 QGC WPL 110 格式創建航點序列
        
        參數:
            lines: QGC 格式的字串列表
        
        返回:
            WaypointSequence 實例
        """
        sequence = cls()
        
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#') or line.startswith('QGC'):
                continue
            
            waypoint = Waypoint.from_qgc_line(line)
            if waypoint:
                sequence.add(waypoint)
        
        return sequence
    
    def validate(self) -> Tuple[bool, List[str]]:
        """
        驗證航點序列的有效性
        
        返回:
            (是否有效, 錯誤訊息列表)
        """
        errors = []
        
        # 檢查是否為空
        if not self.waypoints:
            errors.append("航點序列為空")
            return False, errors
        
        # 檢查第一個點是否為 HOME 或 TAKEOFF
        first_wp = self.waypoints[0]
        if first_wp.command not in (MAVCommand.DO_SET_HOME, MAVCommand.NAV_TAKEOFF):
            errors.append("第一個航點應為 HOME 或 TAKEOFF")
        
        # 檢查座標有效性
        for wp in self.waypoints:
            if wp.command in (MAVCommand.NAV_WAYPOINT, MAVCommand.NAV_TAKEOFF, MAVCommand.NAV_LAND):
                if not (-90 <= wp.lat <= 90):
                    errors.append(f"航點 {wp.seq} 的緯度無效: {wp.lat}")
                if not (-180 <= wp.lon <= 180):
                    errors.append(f"航點 {wp.seq} 的經度無效: {wp.lon}")
                if wp.alt < 0:
                    errors.append(f"航點 {wp.seq} 的高度為負值: {wp.alt}")
        
        # 檢查序列號連續性
        for i, wp in enumerate(self.waypoints):
            if wp.seq != i:
                errors.append(f"序列號不連續: 預期 {i}，實際 {wp.seq}")
        
        return len(errors) == 0, errors
    
    def __len__(self) -> int:
        """序列長度"""
        return len(self.waypoints)
    
    def __getitem__(self, index: int) -> Waypoint:
        """索引訪問"""
        return self.waypoints[index]
    
    def __iter__(self):
        """迭代器"""
        return iter(self.waypoints)
    
    def __str__(self) -> str:
        """字串表示"""
        nav_count = len(self.get_navigation_waypoints())
        total_dist = self.calculate_total_distance()
        return f"WaypointSequence(total={len(self)}, nav={nav_count}, distance={total_dist:.1f}m)"
    
    def __repr__(self) -> str:
        """詳細表示"""
        return self.__str__()


# ==========================================
# 航點工廠函數
# ==========================================
def create_home_waypoint(lat: float, lon: float, alt: float = 0.0) -> Waypoint:
    """
    創建 HOME 航點
    
    參數:
        lat, lon: 座標
        alt: 高度
    
    返回:
        HOME 航點
    """
    return Waypoint(
        seq=0,
        command=MAVCommand.DO_SET_HOME,
        lat=lat,
        lon=lon,
        alt=alt,
        current=1
    )


def create_takeoff_waypoint(lat: float, lon: float, alt: float,
                           seq: int = 1) -> Waypoint:
    """
    創建起飛航點
    
    參數:
        lat, lon: 座標
        alt: 起飛高度
        seq: 序列號
    
    返回:
        起飛航點
    """
    return Waypoint(
        seq=seq,
        command=MAVCommand.NAV_TAKEOFF,
        lat=lat,
        lon=lon,
        alt=alt
    )


def create_navigation_waypoint(lat: float, lon: float, alt: float,
                              seq: int) -> Waypoint:
    """
    創建導航航點
    
    參數:
        lat, lon: 座標
        alt: 高度
        seq: 序列號
    
    返回:
        導航航點
    """
    return Waypoint(
        seq=seq,
        command=MAVCommand.NAV_WAYPOINT,
        lat=lat,
        lon=lon,
        alt=alt
    )


def create_loiter_waypoint(lat: float, lon: float, alt: float,
                          duration: float, seq: int) -> Waypoint:
    """
    創建懸停航點
    
    參數:
        lat, lon: 座標
        alt: 高度
        duration: 懸停時間（秒）
        seq: 序列號
    
    返回:
        懸停航點
    """
    return Waypoint(
        seq=seq,
        command=MAVCommand.NAV_LOITER_TIME,
        lat=lat,
        lon=lon,
        alt=alt,
        param1=duration
    )


def create_rtl_waypoint(seq: int) -> Waypoint:
    """
    創建 RTL（返回起點）航點
    
    參數:
        seq: 序列號
    
    返回:
        RTL 航點
    """
    return Waypoint(
        seq=seq,
        command=MAVCommand.NAV_RETURN_TO_LAUNCH
    )


def create_change_speed_command(speed: float, seq: int) -> Waypoint:
    """
    創建改變速度命令
    
    參數:
        speed: 目標速度（公尺/秒）
        seq: 序列號
    
    返回:
        速度命令航點
    """
    return Waypoint(
        seq=seq,
        command=MAVCommand.DO_CHANGE_SPEED,
        param2=speed  # param2 為速度值
    )


def create_condition_yaw_command(heading: float, yaw_speed: float,
                                seq: int) -> Waypoint:
    """
    創建航向命令
    
    參數:
        heading: 目標航向（度）
        yaw_speed: 轉向速度（度/秒）
        seq: 序列號
    
    返回:
        航向命令航點
    """
    return Waypoint(
        seq=seq,
        command=MAVCommand.CONDITION_YAW,
        param1=heading,
        param2=yaw_speed
    )
