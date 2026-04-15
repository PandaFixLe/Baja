#!/bin/bash

# 1. 激活虚拟环境
echo "📦 激活虚拟环境..."
source venv/bin/activate

# 2. 运行Python脚本生成图片
echo "📊 生成图表..."
python plot_results.py

# 检查是否成功生成
if [ $? -ne 0 ]; then
    echo "❌ Python脚本执行失败！"
    exit 1
fi

# 3. 创建Windows桌面上的目标文件夹
echo "📁 准备复制到Windows桌面..."
DESKTOP_PATH="/mnt/c/Users/PandaFixLe/Desktop/baja_results"
mkdir -p "$DESKTOP_PATH"

# 4. 复制图片到Windows桌面
echo "📤 复制图片到: $DESKTOP_PATH"
if [ -d "docs/figures" ] && [ "$(ls -A docs/figures/*.png 2>/dev/null)" ]; then
    cp docs/figures/*.png "$DESKTOP_PATH/"
    
    # 添加时间戳
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    echo ""
    echo "✅ 完成！生成的文件："
    echo "=================================="
    ls -lh "$DESKTOP_PATH"/*.png
    echo ""
    echo "📂 文件夹位置: $DESKTOP_PATH"
    echo "⏰ 生成时间: $TIMESTAMP"
    echo "=================================="
    echo ""
    echo "💡 提示：图片已自动复制到Windows桌面 'baja_results' 文件夹"
else
    echo "⚠️  警告：未找到生成的图片文件"
    echo "检查 docs/figures/ 目录是否存在或脚本是否成功运行"
fi

