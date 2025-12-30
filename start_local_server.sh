#!/bin/bash
# 启动本地测试服务器
# 用于在浏览器中测试 statistics.html 页面
# 
# 使用方法：
#   1. 双击运行此脚本（macOS/Linux）
#   2. 或者命令行运行: ./start_local_server.sh
#   3. 然后在浏览器中打开: http://localhost:8000/index.html

echo "=========================================="
echo "🚀 启动本地测试服务器"
echo "=========================================="
echo ""
echo "服务器地址: http://localhost:8000"
echo ""
echo "推荐访问:"
echo "  - 首页: http://localhost:8000/index.html"
echo ""
echo "按 Ctrl+C 停止服务器"
echo "=========================================="
echo ""

python3 -m http.server 8000 2>/dev/null || python -m http.server 8000

