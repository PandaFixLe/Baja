#!/usr/bin/env bash
set -e

# 一键生成轨迹图：先编译然后运行仿真，再绘制轨迹
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT_DIR"

# 激活虚拟环境（如果存在）
if [ -d "venv" ]; then
  source "${ROOT_DIR}/venv/bin/activate"
fi

# 保证 build 目录存在
mkdir -p build
cd build

# 如果可执行文件不存在，则重新配置并编译
if [ ! -x "controller" ]; then
  cmake ..
  make controller
fi

# 运行仿真并生成轨迹数据
./controller

# 返回根目录绘制轨迹图
cd "$ROOT_DIR"

# 如果 plot_trajectory 依赖缺失，尝试在虚拟环境内安装
if ! python3 -c "import pandas, matplotlib" >/dev/null 2>&1; then
  python3 -m pip install pandas matplotlib
fi

python3 plot_trajectory.py

echo "轨迹图已生成：trajectory_plot.png"