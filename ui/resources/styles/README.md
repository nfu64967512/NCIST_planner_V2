# 字體資源目錄

這個目錄用於存放自訂字體檔案，系統會自動載入並註冊這些字體。

## 支援的字體格式

- `.ttf` (TrueType Font)
- `.otf` (OpenType Font)

## 如何使用自訂字體

### 1. 添加字體檔案

將字體檔案放入此目錄：
```
ui/resources/fonts/
├── CustomFont-Regular.ttf
├── CustomFont-Bold.ttf
└── CustomFont-Italic.ttf
```

### 2. 程式中載入字體

```python
from ui.resources import ResourceLoader

# 載入特定字體
font_id = ResourceLoader.load_font('CustomFont-Regular.ttf')
if font_id != -1:
    font = QFont('CustomFont')
    widget.setFont(font)
```

### 3. 在樣式表中使用

```css
QWidget {
    font-family: "CustomFont", "Segoe UI", sans-serif;
}
```

## 推薦字體

### 免費開源字體

1. **思源黑體 (Noto Sans CJK)**
   - 支援中日韓文字
   - Google + Adobe 聯合開發
   - 下載：https://github.com/googlefonts/noto-cjk

2. **Fira Code**
   - 程式碼專用字體
   - 支援連字 (ligatures)
   - 下載：https://github.com/tonsky/FiraCode

3. **Roboto**
   - Material Design 預設字體
   - 現代化、易讀
   - 下載：https://fonts.google.com/specimen/Roboto

4. **Inter**
   - UI 介面專用字體
   - 適合螢幕顯示
   - 下載：https://rsms.me/inter/

### 繁體中文字體

1. **台北黑體 (Taipei Sans TC)**
   - 台北市政府開發
   - 免費商用
   - 下載：https://sites.google.com/view/jtfoundry/

2. **思源宋體 (Noto Serif CJK)**
   - 宋體樣式
   - Google + Adobe
   - 下載：https://github.com/googlefonts/noto-cjk

3. **jf open 粉圓**
   - 圓潤可愛風格
   - 適合親和力介面
   - 下載：https://github.com/justfont/open-huninn-font

## 注意事項

1. **字體授權**：請確認字體授權允許在您的專案中使用
2. **檔案大小**：中文字體通常較大 (10-20MB)，建議只包含需要的字重
3. **效能影響**：載入多個大型字體會增加啟動時間
4. **跨平台**：測試字體在不同作業系統上的顯示效果

## 系統預設字體

如果不提供自訂字體，系統會使用以下優先順序的字體：

1. Windows: `Segoe UI` / `Microsoft YaHei` (微軟正黑體)
2. macOS: `SF Pro` / `PingFang TC` (蘋方-繁)
3. Linux: `Noto Sans` / `Noto Sans CJK TC`

## 字體子集化

如果字體檔案太大，可以使用工具只保留需要的字元：

### 使用 fonttools

```bash
pip install fonttools

# 只保留常用漢字
pyftsubset font.ttf \
  --unicodes=U+4E00-9FFF \
  --output-file=font-subset.ttf
```

### 使用線上工具

- Font Squirrel Webfont Generator: https://www.fontsquirrel.com/tools/webfont-generator
- Transfonter: https://transfonter.org/

## 範例：完整字體配置

```python
from PyQt6.QtGui import QFont, QFontDatabase
from ui.resources import ResourceLoader

# 載入主要字體
ResourceLoader.load_font('NotoSansCJK-Regular.ttf')
ResourceLoader.load_font('NotoSansCJK-Bold.ttf')

# 設定應用程式字體
app_font = QFont('Noto Sans CJK TC')
app_font.setPointSize(9)
app.setFont(app_font)

# 設定等寬字體（用於代碼顯示）
code_font = QFont('Fira Code')
code_font.setPointSize(9)
code_editor.setFont(code_font)
```
