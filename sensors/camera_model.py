import math
import numpy as np
from dataclasses import dataclass

@dataclass
class CameraSpecs:
    """相機硬體規格資料結構"""
    sensor_width_mm: float      # 感光元件寬度 (mm)
    sensor_height_mm: float     # 感光元件高度 (mm)
    focal_length_mm: float      # 焦距 (mm)
    image_width_px: int         # 圖片寬度 (pixel)
    image_height_px: int        # 圖片高度 (pixel)
    name: str = "Generic Camera"

class CameraModel:
    """
    相機模型與視場角(FOV)計算核心
    負責計算 GSD、覆蓋範圍以及拍攝間隔
    """
    def __init__(self, specs: CameraSpecs):
        self.specs = specs
        self._validate_specs()

    def _validate_specs(self):
        if self.specs.focal_length_mm <= 0:
            raise ValueError("焦距必須大於 0")

    def calculate_fov(self) -> tuple[float, float]:
        """
        計算水平與垂直視場角 (Field of View)
        Returns:
            (hfov_deg, vfov_deg): 水平與垂直 FOV (度)
        """
        hfov = 2 * math.atan(self.specs.sensor_width_mm / (2 * self.specs.focal_length_mm))
        vfov = 2 * math.atan(self.specs.sensor_height_mm / (2 * self.specs.focal_length_mm))
        return math.degrees(hfov), math.degrees(vfov)

    def calculate_gsd(self, altitude_m: float) -> float:
        """
        計算地面採樣距離 (Ground Sampling Distance)
        Formula: (Sensor Width / Image Width) * (Altitude / Focal Length)
        
        Args:
            altitude_m: 相對地面高度 (AGL)
        Returns:
            gsd_cm_per_pixel: 每個像素代表的公分
        """
        if altitude_m <= 0:
            return 0.0
        
        # 使用感光元件寬度計算 (通常取較大邊或寬度)
        gsd_m = (self.specs.sensor_width_mm / self.specs.image_width_px) * \
                (altitude_m / (self.specs.focal_length_mm / 1000.0))
        
        return gsd_m * 100  # 轉換為 cm

    def calculate_footprint(self, altitude_m: float) -> tuple[float, float]:
        """
        計算在地面的投影範圍 (Footprint)
        
        Returns:
            (width_m, height_m): 地面覆蓋寬度與高度
        """
        if altitude_m <= 0:
            return 0.0, 0.0

        # Width on ground = (Sensor Width * Altitude) / Focal Length
        # 注意單位轉換: sensor_mm / focal_mm = 無單位比例
        width_on_ground = (self.specs.sensor_width_mm * altitude_m) / self.specs.focal_length_mm
        height_on_ground = (self.specs.sensor_height_mm * altitude_m) / self.specs.focal_length_mm
        
        return width_on_ground, height_on_ground

    def calculate_survey_parameters(self, altitude_m: float, overlap_percent: float, sidelap_percent: float):
        """
        根據重疊率計算航線參數 (供 Global Planner 使用)
        
        Args:
            overlap_percent: 航向重疊率 (Front Lap) 0-100
            sidelap_percent: 旁向重疊率 (Side Lap) 0-100
            
        Returns:
            dict: {
                'strip_distance': 航線間距 (m),
                'trigger_distance': 拍照間隔距離 (m),
                'gsd': GSD (cm/px)
            }
        """
        fp_width, fp_height = self.calculate_footprint(altitude_m)
        
        # 旁向重疊決定航線間距 (Side Lap affects distance between lines)
        # 假設 sensor width 對應橫向 (Landscape mode)
        strip_dist = fp_width * (1 - sidelap_percent / 100.0)
        
        # 航向重疊決定拍照觸發距離 (Front Lap affects trigger distance)
        trigger_dist = fp_height * (1 - overlap_percent / 100.0)
        
        return {
            "strip_distance": strip_dist,
            "trigger_distance": trigger_dist,
            "gsd": self.calculate_gsd(altitude_m),
            "footprint_width": fp_width,
            "footprint_height": fp_height
        }

# 範例：定義一個常見的相機 (如 Sony RX1R II)
RX1R_II = CameraSpecs(
    sensor_width_mm=35.9,
    sensor_height_mm=24.0,
    focal_length_mm=35.0,
    image_width_px=7952,
    image_height_px=5304,
    name="Sony RX1R II"
)


# ==========================================
# 相機資料庫
# ==========================================
@dataclass
class CameraInfo:
    """相機資訊（用於 CameraDatabase）"""
    name: str
    manufacturer: str
    sensor_width: float  # mm
    sensor_height: float  # mm
    image_width: int  # px
    image_height: int  # px
    focal_length: float  # mm


class CameraDatabase:
    """
    相機資料庫
    預置常見無人機相機參數
    """

    CAMERAS = {
        # DJI 系列
        "DJI Mavic 3": CameraInfo(
            name="DJI Mavic 3",
            manufacturer="DJI",
            sensor_width=17.3,
            sensor_height=13.0,
            image_width=5280,
            image_height=3956,
            focal_length=24.0
        ),
        "DJI Mavic 3 Cine": CameraInfo(
            name="DJI Mavic 3 Cine",
            manufacturer="DJI",
            sensor_width=17.3,
            sensor_height=13.0,
            image_width=5280,
            image_height=3956,
            focal_length=24.0
        ),
        "DJI Phantom 4 Pro": CameraInfo(
            name="DJI Phantom 4 Pro",
            manufacturer="DJI",
            sensor_width=13.2,
            sensor_height=8.8,
            image_width=5472,
            image_height=3648,
            focal_length=8.8
        ),
        "DJI Mini 3 Pro": CameraInfo(
            name="DJI Mini 3 Pro",
            manufacturer="DJI",
            sensor_width=9.7,
            sensor_height=7.3,
            image_width=4000,
            image_height=3000,
            focal_length=6.72
        ),
        "DJI Air 2S": CameraInfo(
            name="DJI Air 2S",
            manufacturer="DJI",
            sensor_width=13.2,
            sensor_height=8.8,
            image_width=5472,
            image_height=3648,
            focal_length=8.4
        ),
        "DJI Zenmuse P1": CameraInfo(
            name="DJI Zenmuse P1",
            manufacturer="DJI",
            sensor_width=35.9,
            sensor_height=24.0,
            image_width=8192,
            image_height=5460,
            focal_length=35.0
        ),
        # Sony 系列
        "Sony RX1R II": CameraInfo(
            name="Sony RX1R II",
            manufacturer="Sony",
            sensor_width=35.9,
            sensor_height=24.0,
            image_width=7952,
            image_height=5304,
            focal_length=35.0
        ),
        "Sony A7R IV": CameraInfo(
            name="Sony A7R IV",
            manufacturer="Sony",
            sensor_width=35.7,
            sensor_height=23.8,
            image_width=9504,
            image_height=6336,
            focal_length=24.0
        ),
        # 通用相機
        "Generic 1-inch": CameraInfo(
            name="Generic 1-inch Sensor",
            manufacturer="Generic",
            sensor_width=13.2,
            sensor_height=8.8,
            image_width=5472,
            image_height=3648,
            focal_length=9.0
        ),
        "Generic APS-C": CameraInfo(
            name="Generic APS-C Sensor",
            manufacturer="Generic",
            sensor_width=23.5,
            sensor_height=15.6,
            image_width=6000,
            image_height=4000,
            focal_length=18.0
        ),
    }

    @classmethod
    def get_camera(cls, name: str) -> CameraInfo:
        """獲取相機資訊"""
        return cls.CAMERAS.get(name)

    @classmethod
    def get_camera_list(cls) -> list:
        """獲取所有相機名稱"""
        return list(cls.CAMERAS.keys())

    @classmethod
    def get_manufacturers(cls) -> list:
        """獲取所有製造商"""
        manufacturers = set(cam.manufacturer for cam in cls.CAMERAS.values())
        return sorted(list(manufacturers))


class CameraCalculator:
    """
    相機計算器
    提供航測相關的計算功能
    """

    @staticmethod
    def calculate_gsd(altitude: float, focal_length: float,
                      sensor_width: float, image_width: int) -> float:
        """
        計算地面採樣距離 (GSD)

        參數:
            altitude: 飛行高度 (m)
            focal_length: 焦距 (mm)
            sensor_width: 感光元件寬度 (mm)
            image_width: 圖像寬度 (px)

        返回:
            GSD (m/px)
        """
        if focal_length <= 0 or image_width <= 0:
            return 0.0

        # GSD = (sensor_width / image_width) * (altitude / focal_length)
        gsd = (sensor_width / image_width) * (altitude / focal_length)
        return gsd / 1000.0  # mm to m

    @staticmethod
    def calculate_ground_coverage(altitude: float, focal_length: float,
                                   sensor_width: float, sensor_height: float) -> tuple:
        """
        計算地面覆蓋範圍

        返回:
            (width_m, height_m)
        """
        if focal_length <= 0:
            return (0.0, 0.0)

        width = (sensor_width / focal_length) * altitude
        height = (sensor_height / focal_length) * altitude
        return (width, height)

    @staticmethod
    def calculate_field_of_view(focal_length: float,
                                sensor_width: float,
                                sensor_height: float) -> tuple:
        """
        計算視場角 (FOV)

        返回:
            (horizontal_fov_deg, vertical_fov_deg)
        """
        if focal_length <= 0:
            return (0.0, 0.0)

        h_fov = 2 * math.atan(sensor_width / (2 * focal_length))
        v_fov = 2 * math.atan(sensor_height / (2 * focal_length))
        return (math.degrees(h_fov), math.degrees(v_fov))

    @staticmethod
    def calculate_spacing_from_overlap(altitude: float, camera: CameraInfo,
                                       front_overlap: float,
                                       side_overlap: float) -> tuple:
        """
        根據重疊率計算航線間距和拍照間隔

        參數:
            altitude: 飛行高度 (m)
            camera: 相機資訊
            front_overlap: 前向重疊率 (%)
            side_overlap: 側向重疊率 (%)

        返回:
            (line_spacing_m, photo_interval_m)
        """
        ground_width, ground_height = CameraCalculator.calculate_ground_coverage(
            altitude, camera.focal_length, camera.sensor_width, camera.sensor_height
        )

        # 航線間距 = 地面覆蓋寬度 × (1 - 側向重疊率)
        line_spacing = ground_width * (1 - side_overlap / 100.0)

        # 拍照間隔 = 地面覆蓋高度 × (1 - 前向重疊率)
        photo_interval = ground_height * (1 - front_overlap / 100.0)

        return (line_spacing, photo_interval)

    @staticmethod
    def calculate_required_photos(area: float, altitude: float,
                                  camera: CameraInfo,
                                  front_overlap: float,
                                  side_overlap: float) -> int:
        """
        計算所需照片數量

        參數:
            area: 區域面積 (m²)
            altitude: 飛行高度 (m)
            camera: 相機資訊
            front_overlap: 前向重疊率 (%)
            side_overlap: 側向重疊率 (%)

        返回:
            預估照片數量
        """
        line_spacing, photo_interval = CameraCalculator.calculate_spacing_from_overlap(
            altitude, camera, front_overlap, side_overlap
        )

        if line_spacing <= 0 or photo_interval <= 0:
            return 0

        # 有效覆蓋面積 = 間距 × 間隔
        effective_area_per_photo = line_spacing * photo_interval

        if effective_area_per_photo <= 0:
            return 0

        return int(math.ceil(area / effective_area_per_photo))

    @staticmethod
    def calculate_flight_time(distance: float, speed: float,
                              photo_count: int, trigger_delay: float = 0.5) -> float:
        """
        計算飛行時間

        參數:
            distance: 總飛行距離 (m)
            speed: 飛行速度 (m/s)
            photo_count: 照片數量
            trigger_delay: 每張照片的觸發延遲 (s)

        返回:
            預估飛行時間 (s)
        """
        if speed <= 0:
            return 0.0

        flight_time = distance / speed
        photo_time = photo_count * trigger_delay

        return flight_time + photo_time


@dataclass
class SurveyParameters:
    """測繪參數"""
    camera: CameraInfo
    altitude_m: float
    front_overlap_percent: float = 80.0
    side_overlap_percent: float = 60.0
    speed_mps: float = 10.0

    def get_gsd(self) -> float:
        """獲取 GSD (m/px)"""
        return CameraCalculator.calculate_gsd(
            self.altitude_m,
            self.camera.focal_length,
            self.camera.sensor_width,
            self.camera.image_width
        )

    def get_auto_spacing(self) -> tuple:
        """獲取自動計算的間距"""
        return CameraCalculator.calculate_spacing_from_overlap(
            self.altitude_m,
            self.camera,
            self.front_overlap_percent,
            self.side_overlap_percent
        )