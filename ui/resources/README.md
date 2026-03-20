# UI Resources 資源模組

## 概述
提供統一的 UI 資源管理，包括圖示、樣式表等。

## 目錄結構
```
ui/resources/
├── __init__.py           # 資源模組入口
├── icons/                # 圖示資源
│   ├── __init__.py
│   ├── icon_manager.py   # 圖示管理器
│   └── *.svg            # SVG 圖示檔案（可選）
└── styles/              # 樣式表資源
    └── dark_theme.qss   # 深色主題樣式表
```

## 使用方式

### 圖示管理器

#### 基本使用
```python
from ui.resources import get_icon, Icons

# 獲取圖示
icon = get_icon(Icons.NEW)

# 獲取指定顏色的圖示
icon = get_icon(Icons.DRONE, "#2196F3")

# 應用到按鈕
button.setIcon(get_icon(Icons.SAVE))
```

#### 內建圖示
圖示管理器提供以下內建圖示（使用 SVG 定義）：

**檔案操作**
- `Icons.NEW` - 新建檔案
- `Icons.OPEN` - 開啟檔案
- `Icons.SAVE` - 儲存檔案

**編輯操作**
- `Icons.EDIT` - 編輯
- `Icons.DELETE` - 刪除
- `Icons.CLEAR` - 清除

**地圖操作**
- `Icons.MAP` - 地圖
- `Icons.MARKER` - 標記

**任務操作**
- `Icons.PREVIEW` - 預覽
- `Icons.EXPORT` - 匯出

**設定**
- `Icons.SETTINGS` - 設定
- `Icons.CAMERA` - 相機
- `Icons.DRONE` - 無人機

**資訊**
- `Icons.INFO` - 資訊
- `Icons.HELP` - 說明
- `Icons.WARNING` - 警告
- `Icons.SUCCESS` - 成功

#### 進階使用
```python
from ui.resources.icons import IconManager

# 獲取管理器實例
manager = IconManager.get_instance()

# 創建有色圖示
icon = manager.create_colored_icon('drone', '#FF5722')

# 清除快取
manager.clear_cache()
```

#### 自訂圖示
可以在 `ui/resources/icons/` 目錄下放置 SVG 或 PNG 檔案：

```
ui/resources/icons/
├── custom_icon.svg      # 自訂 SVG 圖示
└── custom_icon.png      # 自訂 PNG 圖示
```

使用時：
```python
icon = get_icon('custom_icon')
```

### 樣式表

#### 載入樣式表
```python
def load_stylesheet():
    """載入樣式表"""
    style_path = Path(__file__).parent / 'resources' / 'styles' / 'dark_theme.qss'
    
    if style_path.exists():
        with open(style_path, 'r', encoding='utf-8') as f:
            return f.read()
    
    return ""

# 應用到應用程式
app.setStyleSheet(load_stylesheet())
```

## 圖示格式

### SVG 圖示範例
```svg
<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
    <circle cx="12" cy="12" r="10"/>
    <polyline points="9 12 11 14 15 10"/>
</svg>
```

- 使用 `viewBox="0 0 24 24"` 作為標準尺寸
- 使用 `currentColor` 允許動態顏色替換
- `stroke-width="2"` 作為標準線寬

## 顏色代碼

### Material Design 調色板
```python
# 主題色
PRIMARY = "#2196F3"      # 藍色
SUCCESS = "#4CAF50"      # 綠色
WARNING = "#FF9800"      # 橘色
DANGER = "#F44336"       # 紅色
INFO = "#00BCD4"         # 青色

# 中性色
DARK = "#212121"
LIGHT = "#F5F5F5"
MUTED = "#9E9E9E"
```

## 最佳實踐

1. **使用常數**: 使用 `Icons` 類別的常數而非字串
   ```python
   # 好的
   icon = get_icon(Icons.NEW)
   
   # 不好的
   icon = get_icon('new')
   ```

2. **顏色一致性**: 使用主題色系
   ```python
   # 主要動作 - 藍色
   icon = get_icon(Icons.PREVIEW, "#2196F3")
   
   # 成功動作 - 綠色
   icon = get_icon(Icons.SUCCESS, "#4CAF50")
   
   # 危險動作 - 紅色
   icon = get_icon(Icons.DELETE, "#F44336")
   ```

3. **快取管理**: 圖示會自動快取，無需手動管理

4. **SVG 優先**: 優先使用 SVG 格式以獲得更好的縮放品質

## 擴展

### 添加新圖示
1. 創建 SVG 檔案並放置在 `ui/resources/icons/` 目錄
2. 或在 `icon_manager.py` 的 `_get_builtin_icon()` 中添加 SVG 模板
3. 在 `Icons` 類別中添加常數

### 自訂主題
1. 複製 `dark_theme.qss` 並重新命名
2. 修改顏色值和樣式
3. 在主程式中載入自訂主題

## 疑難排解

### 圖示不顯示
- 檢查檔案路徑是否正確
- 確認 SVG 格式是否有效
- 檢查 `PyQt6.QtSvg` 是否已安裝

### 顏色替換無效
- 確保 SVG 使用 `currentColor` 或 `#000000`
- 檢查 `stroke` 和 `fill` 屬性設定

### 效能問題
- 使用 `clear_cache()` 清除不需要的圖示
- 考慮使用較小的圖示尺寸
