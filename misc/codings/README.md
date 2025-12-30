# WakaTime 统计数据工具

本目录包含用于获取和验证 WakaTime 编码统计数据的工具脚本。

## 文件说明

- `fetch_all_data.py` - **完整历史数据爬取脚本**
  - 从 2022 年开始获取所有历史数据
  - 支持 tqdm 进度条显示
  - 支持断点续传
  - 数据结构清晰（按日期组织）
  - **用途：** 首次获取或重新获取所有历史数据
  
- `fetch_recent_data.py` - **增量更新脚本（每日自动运行）**
  - 只获取最近 3 天的数据
  - 自动合并到现有数据
  - 轻量级，运行快速
  - **用途：** 每日自动更新，保持数据最新
  
- `verify_data.py` - 验证数据文件的完整性和格式

## 使用方法

### 0. 设置 API Key（必需）

在运行脚本之前，需要设置 WakaTime API Key 作为环境变量：

```bash
# 从项目根目录运行
export WAKATIME_API_KEY='your-wakatime-api-key-here'
```

或者在一行中运行：
```bash
WAKATIME_API_KEY='your-wakatime-api-key-here' python3 misc/codings/fetch_all_data.py
```

### 1. 首次获取或重新获取所有历史数据

```bash
# 从项目根目录运行（确保已设置 WAKATIME_API_KEY 环境变量）
python3 misc/codings/fetch_all_data.py
```

**功能特性：**
- ✅ 显示 tqdm 进度条
- ✅ 详细的日志信息
- ✅ 自动断点续传（如果中断，重新运行会从上次停止的地方继续）
- ✅ 每 5 个批次自动保存进度
- ✅ 清晰的数据结构（按日期组织）

### 2. 增量更新（获取最近几天数据）

```bash
# 从项目根目录运行（默认获取最近 3 天，确保已设置 WAKATIME_API_KEY 环境变量）
python3 misc/codings/fetch_recent_data.py

# 或者指定天数（例如获取最近 7 天）
python3 misc/codings/fetch_recent_data.py 7
```

**功能特性：**
- ✅ 只获取最近 N 天的数据（默认 3 天）
- ✅ 自动合并到现有数据
- ✅ 更新已存在的日期数据
- ✅ 快速运行，适合每日自动更新

### 3. 验证数据

```bash
# 从项目根目录运行
python3 misc/codings/verify_data.py
```

### 4. 本地测试

```bash
# 从项目根目录运行
./start_local_server.sh
```

然后在浏览器中打开 `http://localhost:8000/statistics.html`

**注意：** 本地测试需要使用 HTTP 服务器，因为浏览器的安全策略不允许 `file://` 协议直接加载 JSON 文件。部署到 GitHub Pages 后不需要本地服务器。

## 数据文件

数据文件保存在本目录：
- `wakatime_stats.json` - 统计数据（最近 30 天）
- `wakatime_summaries.json` - 每日详细数据（从 2022 年开始）

### 数据格式

`wakatime_summaries.json` 使用清晰的结构：

```json
{
  "metadata": {
    "last_updated": "2025-12-30T...",
    "total_days": 1460,
    "date_range": {
      "start": "2022-01-01",
      "end": "2025-12-30"
    }
  },
  "daily_data": {
    "2022-01-01": {
      "date": "2022-01-01",
      "total_seconds": 3600,
      "total_hours": 1.0,
      "text": "1 hr",
      "languages": [...],
      "editors": [...],
      "projects": [...]
    },
    "2022-01-02": {...}
  }
}
```

这种格式更易读，也更容易查找特定日期的数据。

## GitHub Actions 自动更新

GitHub Actions 工作流会自动运行 `fetch_recent_data.py` 来每日更新数据文件。

### 配置说明

1. **自动运行时间：** 每天北京时间凌晨 0:00（UTC 16:00）
2. **更新范围：** 最近 3 天的数据
3. **工作流文件：** `.github/workflows/update-wakatime.yml`

### 设置 GitHub Secrets（必需）

**重要：** 由于安全原因，API Key 不再硬编码在代码中。你必须设置 GitHub Secret：

1. 在 GitHub 仓库设置中添加 Secret：
   - 进入仓库 → Settings → Secrets and variables → Actions
   - 点击 "New repository secret"
   - Name: `WAKATIME_API_KEY`
   - Value: 你的 WakaTime API Key（例如：`waka_bbfc4972-29b1-47e2-bab0-a822624e7123`）
   - 点击 "Add secret"

2. 工作流会自动使用该 Secret 作为环境变量传递给脚本

**本地运行：** 在本地运行脚本时，需要设置环境变量：
```bash
export WAKATIME_API_KEY='your-api-key-here'
python3 misc/codings/fetch_recent_data.py
```

### 手动触发

你也可以在 GitHub Actions 页面手动触发工作流运行。

## 关于 API 限制

### Premium vs Free 账户

- **Summaries API**：
  - **Premium 账户**：可以获取所有历史数据（无限制）
  - **免费账户**：可以获取最近 14 天的每日详细数据
  - 提供每日详细数据（包括项目、语言、编辑器等）
  
- **Stats API** 是免费的
  - 提供最近 30 天的汇总统计
  - 不提供每日详细数据

### 脚本行为

- **有 Premium 账户时：**
  - `fetch_recent_data.py` 可以正常更新每日详细数据（任意日期范围）
  - `fetch_all_data.py` 可以获取所有历史数据

- **免费账户时：**
  - `fetch_recent_data.py` 会：
    - ✅ 成功更新 Stats 数据（最近 30 天汇总）
    - ✅ **可以更新最近 14 天的每日详细数据**（如果请求的天数 ≤ 14 天）
    - ⚠️ 如果请求超过 14 天，会显示警告但不会失败
    - 脚本仍然会成功完成
  - `fetch_all_data.py` 无法获取超过 14 天的历史数据（需要 Premium）

### 数据格式一致性 ✅

**重要确认：** 两个脚本使用完全相同的数据格式和处理逻辑：

1. **相同的 API 端点**：都使用 `/users/current/summaries`
2. **相同的数据处理函数**：`extract_daily_info()` 和 `save_daily_data()` 完全一致
3. **相同的数据结构**：保存的 JSON 格式完全相同

这意味着：
- ✅ **Premium 账户获取的数据格式 = 免费账户获取的数据格式**（在 14 天内）
- ✅ **每天增量更新获取的数据 = 一次性爬取所有数据获取的数据**（格式完全一致）
- ✅ 每天运行 `fetch_recent_data.py` 更新 3 天数据，和运行 `fetch_all_data.py` 获取所有数据，**在数据格式上完全一样**

**结论：** 取消 Premium 后，每日增量更新的数据格式和 Premium 下获取所有数据的格式**完全一样**，可以无缝合并到现有的 JSON 数据库中。

### 建议

如果你准备取消 Premium 订阅：
1. **在取消前**：运行 `fetch_all_data.py` 获取所有历史数据并保存
2. **取消后**：
   - `fetch_recent_data.py` 仍然可以运行，**可以继续更新最近 14 天的每日详细数据**
   - 默认获取 3 天数据，完全在免费账户的 14 天限制内
   - **数据格式与 Premium 时完全一致**，可以无缝合并
   - 超过 14 天的历史数据无法更新
3. **数据展示**：
   - `statistics.html` 会继续显示已有的历史每日数据
   - **新的最近 14 天数据可以正常更新和显示**
   - 数据格式完全一致，不会有任何兼容性问题

