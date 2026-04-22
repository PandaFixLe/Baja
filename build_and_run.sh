#!/bin/bash
echo "🚀 开始编译..."
cd build
make -j8  # 编译

if [ $? -ne 0 ]; then
    echo "❌ 编译失败！"
    exit 1
fi

echo "✅ 编译成功，开始运行轨迹测试..."
cd ..
./run_trajectory.sh  # 运行脚本