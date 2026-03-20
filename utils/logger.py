"""
日誌工具模組
提供統一的日誌管理功能，支援文件輸出和格式化
"""

import logging
import os
import sys
from datetime import datetime
from typing import Optional
from pathlib import Path


# ==========================================
# 日誌等級映射
# ==========================================
LOG_LEVELS = {
    'DEBUG': logging.DEBUG,
    'INFO': logging.INFO,
    'WARNING': logging.WARNING,
    'ERROR': logging.ERROR,
    'CRITICAL': logging.CRITICAL
}


# ==========================================
# 日誌格式定義
# ==========================================
class LogFormatter(logging.Formatter):
    """自訂日誌格式器，支援顏色輸出（終端機）"""
    
    # ANSI 顏色碼
    COLORS = {
        'DEBUG': '\033[36m',      # 青色
        'INFO': '\033[32m',       # 綠色
        'WARNING': '\033[33m',    # 黃色
        'ERROR': '\033[31m',      # 紅色
        'CRITICAL': '\033[35m',   # 紫色
        'RESET': '\033[0m'
    }
    
    def __init__(self, use_color: bool = True):
        """
        初始化格式器
        
        參數:
            use_color: 是否使用顏色
        """
        super().__init__(
            fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        self.use_color = use_color
    
    def format(self, record):
        """格式化日誌記錄"""
        if self.use_color and sys.stdout.isatty():
            # 終端機環境，使用顏色
            levelname = record.levelname
            if levelname in self.COLORS:
                record.levelname = (
                    f"{self.COLORS[levelname]}{levelname}{self.COLORS['RESET']}"
                )
        
        return super().format(record)


# ==========================================
# 日誌管理器
# ==========================================
class Logger:
    """統一的日誌管理器"""
    
    _instances = {}
    
    def __init__(self, name: str = 'UAVPathPlanner',
                level: str = 'INFO',
                log_dir: Optional[str] = None,
                log_to_file: bool = True,
                log_to_console: bool = True):
        """
        初始化日誌管理器
        
        參數:
            name: 日誌名稱
            level: 日誌等級
            log_dir: 日誌目錄
            log_to_file: 是否輸出到文件
            log_to_console: 是否輸出到控制台
        """
        self.logger = logging.getLogger(name)
        self.logger.setLevel(LOG_LEVELS.get(level.upper(), logging.INFO))
        
        # 清除現有處理器
        self.logger.handlers.clear()
        
        # 控制台處理器
        if log_to_console:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setFormatter(LogFormatter(use_color=True))
            self.logger.addHandler(console_handler)
        
        # 文件處理器
        if log_to_file:
            if log_dir is None:
                # 使用預設日誌目錄
                project_root = Path(__file__).parent.parent
                log_dir = os.path.join(project_root, 'data', 'logs')
            
            # 確保日誌目錄存在
            os.makedirs(log_dir, exist_ok=True)
            
            # 建立日誌文件（按日期命名）
            log_filename = f"{name}_{datetime.now().strftime('%Y%m%d')}.log"
            log_filepath = os.path.join(log_dir, log_filename)
            
            file_handler = logging.FileHandler(log_filepath, encoding='utf-8')
            file_handler.setFormatter(LogFormatter(use_color=False))
            self.logger.addHandler(file_handler)
        
        # 防止日誌向上傳播
        self.logger.propagate = False
    
    def debug(self, message: str):
        """輸出 DEBUG 等級日誌"""
        self.logger.debug(message)
    
    def info(self, message: str):
        """輸出 INFO 等級日誌"""
        self.logger.info(message)
    
    def warning(self, message: str):
        """輸出 WARNING 等級日誌"""
        self.logger.warning(message)
    
    def error(self, message: str):
        """輸出 ERROR 等級日誌"""
        self.logger.error(message)
    
    def critical(self, message: str):
        """輸出 CRITICAL 等級日誌"""
        self.logger.critical(message)
    
    def exception(self, message: str):
        """輸出異常日誌（包含堆疊追蹤）"""
        self.logger.exception(message)
    
    def set_level(self, level: str):
        """
        設定日誌等級
        
        參數:
            level: 日誌等級（DEBUG, INFO, WARNING, ERROR, CRITICAL）
        """
        self.logger.setLevel(LOG_LEVELS.get(level.upper(), logging.INFO))
    
    @classmethod
    def get_instance(cls, name: str = 'UAVPathPlanner', **kwargs):
        """
        獲取日誌實例（單例模式）
        
        參數:
            name: 日誌名稱
            **kwargs: 其他初始化參數
        
        返回:
            Logger 實例
        """
        if name not in cls._instances:
            cls._instances[name] = cls(name, **kwargs)
        return cls._instances[name]


# ==========================================
# 便捷函數
# ==========================================
def setup_logger(name: str = 'UAVPathPlanner',
                level: str = 'INFO',
                log_dir: Optional[str] = None,
                log_to_file: bool = True,
                log_to_console: bool = True) -> Logger:
    """
    設定日誌管理器
    
    參數:
        name: 日誌名稱
        level: 日誌等級
        log_dir: 日誌目錄
        log_to_file: 是否輸出到文件
        log_to_console: 是否輸出到控制台
    
    返回:
        Logger 實例
    """
    return Logger(name, level, log_dir, log_to_file, log_to_console)


def get_logger(name: str = 'UAVPathPlanner') -> Logger:
    """
    獲取日誌實例
    
    參數:
        name: 日誌名稱
    
    返回:
        Logger 實例
    """
    return Logger.get_instance(name)


# ==========================================
# 日誌裝飾器
# ==========================================
def log_function_call(logger: Optional[Logger] = None):
    """
    日誌裝飾器，記錄函數調用
    
    參數:
        logger: Logger 實例（可選）
    
    使用範例:
        @log_function_call()
        def my_function(arg1, arg2):
            pass
    """
    def decorator(func):
        def wrapper(*args, **kwargs):
            _logger = logger or get_logger()
            _logger.debug(f"調用函數: {func.__name__}")
            try:
                result = func(*args, **kwargs)
                _logger.debug(f"函數 {func.__name__} 執行成功")
                return result
            except Exception as e:
                _logger.error(f"函數 {func.__name__} 執行失敗: {e}")
                raise
        return wrapper
    return decorator


def log_execution_time(logger: Optional[Logger] = None):
    """
    日誌裝飾器，記錄函數執行時間
    
    參數:
        logger: Logger 實例（可選）
    
    使用範例:
        @log_execution_time()
        def my_function():
            pass
    """
    import time
    
    def decorator(func):
        def wrapper(*args, **kwargs):
            _logger = logger or get_logger()
            start_time = time.time()
            
            try:
                result = func(*args, **kwargs)
                execution_time = time.time() - start_time
                _logger.info(f"函數 {func.__name__} 執行時間: {execution_time:.4f} 秒")
                return result
            except Exception as e:
                execution_time = time.time() - start_time
                _logger.error(
                    f"函數 {func.__name__} 執行失敗 "
                    f"({execution_time:.4f} 秒): {e}"
                )
                raise
        return wrapper
    return decorator


# ==========================================
# 全局日誌實例
# ==========================================
logger = get_logger()
