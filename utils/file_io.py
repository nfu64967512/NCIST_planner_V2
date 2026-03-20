"""
文件讀寫工具模組
提供 JSON、YAML、航點檔案的讀寫功能
"""

import os
import json
from typing import Any, Dict, List, Optional


# ==========================================
# JSON 文件讀寫
# ==========================================
def read_json(filepath: str) -> Optional[Dict[str, Any]]:
    """
    讀取 JSON 文件
    
    參數:
        filepath: 文件路徑
    
    返回:
        JSON 資料字典，失敗返回 None
    """
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return data
    except FileNotFoundError:
        print(f"文件不存在: {filepath}")
        return None
    except json.JSONDecodeError as e:
        print(f"JSON 解析錯誤: {e}")
        return None
    except Exception as e:
        print(f"讀取 JSON 失敗: {e}")
        return None


def write_json(filepath: str, data: Dict[str, Any], 
              indent: int = 4, ensure_ascii: bool = False) -> bool:
    """
    寫入 JSON 文件
    
    參數:
        filepath: 文件路徑
        data: 要寫入的資料
        indent: 縮排空格數
        ensure_ascii: 是否確保 ASCII 編碼
    
    返回:
        是否成功
    """
    try:
        # 確保目錄存在
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=indent, ensure_ascii=ensure_ascii)
        return True
    except Exception as e:
        print(f"寫入 JSON 失敗: {e}")
        return False


# ==========================================
# YAML 文件讀寫
# ==========================================
def read_yaml(filepath: str) -> Optional[Dict[str, Any]]:
    """
    讀取 YAML 文件
    
    參數:
        filepath: 文件路徑
    
    返回:
        YAML 資料字典，失敗返回 None
    """
    try:
        import yaml
        
        with open(filepath, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return data
    except ImportError:
        print("需要安裝 PyYAML: pip install pyyaml")
        return None
    except FileNotFoundError:
        print(f"文件不存在: {filepath}")
        return None
    except yaml.YAMLError as e:
        print(f"YAML 解析錯誤: {e}")
        return None
    except Exception as e:
        print(f"讀取 YAML 失敗: {e}")
        return None


def write_yaml(filepath: str, data: Dict[str, Any]) -> bool:
    """
    寫入 YAML 文件
    
    參數:
        filepath: 文件路徑
        data: 要寫入的資料
    
    返回:
        是否成功
    """
    try:
        import yaml
        
        # 確保目錄存在
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        with open(filepath, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        return True
    except ImportError:
        print("需要安裝 PyYAML: pip install pyyaml")
        return False
    except Exception as e:
        print(f"寫入 YAML 失敗: {e}")
        return False


# ==========================================
# 航點文件讀寫
# ==========================================
def read_waypoints(filepath: str) -> Optional[List[str]]:
    """
    讀取航點文件（QGC WPL 110 格式）
    
    參數:
        filepath: 文件路徑
    
    返回:
        航點行列表，失敗返回 None
    """
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            lines = [line.strip() for line in f if line.strip()]
        
        # 驗證格式
        if not lines or not lines[0].startswith('QGC WPL'):
            print("無效的航點文件格式")
            return None
        
        return lines
    except FileNotFoundError:
        print(f"文件不存在: {filepath}")
        return None
    except Exception as e:
        print(f"讀取航點文件失敗: {e}")
        return None


def write_waypoints(filepath: str, waypoint_lines: List[str]) -> bool:
    """
    寫入航點文件（QGC WPL 110 格式）
    
    參數:
        filepath: 文件路徑
        waypoint_lines: 航點行列表
    
    返回:
        是否成功
    """
    try:
        # 確保目錄存在
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        # 確保第一行是格式標識
        if not waypoint_lines or not waypoint_lines[0].startswith('QGC WPL'):
            waypoint_lines.insert(0, 'QGC WPL 110')
        
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write('\n'.join(waypoint_lines))
        
        return True
    except Exception as e:
        print(f"寫入航點文件失敗: {e}")
        return False


def parse_waypoint_line(line: str) -> Optional[Dict[str, Any]]:
    """
    解析單行航點資料
    
    參數:
        line: 航點行字串
    
    返回:
        航點資料字典，失敗返回 None
    """
    try:
        parts = line.split('\t')
        
        if len(parts) < 12:
            return None
        
        waypoint = {
            'seq': int(parts[0]),
            'current': int(parts[1]),
            'frame': int(parts[2]),
            'command': int(parts[3]),
            'param1': float(parts[4]),
            'param2': float(parts[5]),
            'param3': float(parts[6]),
            'param4': float(parts[7]),
            'lat': float(parts[8]),
            'lon': float(parts[9]),
            'alt': float(parts[10]),
            'autocontinue': int(parts[11])
        }
        
        return waypoint
    except (ValueError, IndexError) as e:
        print(f"解析航點行失敗: {e}")
        return None


def create_waypoint_line(seq: int, command: int, 
                        lat: float = 0.0, lon: float = 0.0, alt: float = 0.0,
                        param1: float = 0.0, param2: float = 0.0, 
                        param3: float = 0.0, param4: float = 0.0,
                        frame: int = 3, current: int = 0, 
                        autocontinue: int = 1) -> str:
    """
    創建航點行字串
    
    參數:
        seq: 序列號
        command: MAVLink 命令碼
        lat, lon, alt: 座標和高度
        param1-4: 命令參數
        frame: 座標系
        current: 是否為當前航點
        autocontinue: 是否自動繼續
    
    返回:
        航點行字串
    """
    return (f"{seq}\t{current}\t{frame}\t{command}\t"
           f"{param1}\t{param2}\t{param3}\t{param4}\t"
           f"{lat:.6f}\t{lon:.6f}\t{alt:.2f}\t{autocontinue}")


# ==========================================
# 通用文件操作
# ==========================================
def ensure_directory(filepath: str) -> bool:
    """
    確保文件所在目錄存在
    
    參數:
        filepath: 文件路徑
    
    返回:
        是否成功
    """
    try:
        directory = os.path.dirname(filepath)
        if directory:
            os.makedirs(directory, exist_ok=True)
        return True
    except Exception as e:
        print(f"創建目錄失敗: {e}")
        return False


def file_exists(filepath: str) -> bool:
    """
    檢查文件是否存在
    
    參數:
        filepath: 文件路徑
    
    返回:
        是否存在
    """
    return os.path.isfile(filepath)


def get_file_extension(filepath: str) -> str:
    """
    獲取文件副檔名
    
    參數:
        filepath: 文件路徑
    
    返回:
        副檔名（包含點號）
    """
    return os.path.splitext(filepath)[1]


def read_text_file(filepath: str) -> Optional[str]:
    """
    讀取文本文件
    
    參數:
        filepath: 文件路徑
    
    返回:
        文件內容，失敗返回 None
    """
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        return content
    except Exception as e:
        print(f"讀取文本文件失敗: {e}")
        return None


def write_text_file(filepath: str, content: str) -> bool:
    """
    寫入文本文件
    
    參數:
        filepath: 文件路徑
        content: 文件內容
    
    返回:
        是否成功
    """
    try:
        ensure_directory(filepath)
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        return True
    except Exception as e:
        print(f"寫入文本文件失敗: {e}")
        return False


def list_files(directory: str, extension: Optional[str] = None) -> List[str]:
    """
    列出目錄中的文件
    
    參數:
        directory: 目錄路徑
        extension: 副檔名篩選（可選）
    
    返回:
        文件路徑列表
    """
    try:
        if not os.path.isdir(directory):
            return []
        
        files = []
        for filename in os.listdir(directory):
            filepath = os.path.join(directory, filename)
            if os.path.isfile(filepath):
                if extension is None or filepath.endswith(extension):
                    files.append(filepath)
        
        return sorted(files)
    except Exception as e:
        print(f"列出文件失敗: {e}")
        return []


def get_file_size(filepath: str) -> int:
    """
    獲取文件大小
    
    參數:
        filepath: 文件路徑
    
    返回:
        文件大小（位元組），失敗返回 -1
    """
    try:
        return os.path.getsize(filepath)
    except Exception:
        return -1


def delete_file(filepath: str) -> bool:
    """
    刪除文件
    
    參數:
        filepath: 文件路徑
    
    返回:
        是否成功
    """
    try:
        if os.path.isfile(filepath):
            os.remove(filepath)
        return True
    except Exception as e:
        print(f"刪除文件失敗: {e}")
        return False
