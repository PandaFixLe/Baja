#!/usr/bin/env python3
"""
轨迹可视化脚本 - 用于展示巴哈赛车控制器轨迹
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def load_trajectory_data(filename):
    """加载轨迹数据"""
    df = pd.read_csv(filename)
    ref_path = df[df['type'] == 'ref']
    car_traj = df[df['type'] == 'car']
    return ref_path, car_traj

def plot_trajectory(ref_path, car_traj, save_path=None):
    """绘制轨迹对比图"""
    plt.figure(figsize=(12, 8))

    # 绘制参考路径
    plt.plot(ref_path['x'], ref_path['y'], 'b--', linewidth=2, label='参考路径', alpha=0.7)

    # 绘制车辆轨迹
    plt.plot(car_traj['x'], car_traj['y'], 'r-', linewidth=1.5, label='车辆轨迹')

    # 标记起点和终点
    plt.plot(ref_path['x'].iloc[0], ref_path['y'].iloc[0], 'go', markersize=8, label='起点')
    plt.plot(car_traj['x'].iloc[0], car_traj['y'].iloc[0], 'ro', markersize=6, label='车辆起点')
    plt.plot(car_traj['x'].iloc[-1], car_traj['y'].iloc[-1], 'rx', markersize=8, label='车辆终点')

    # 计算误差统计
    # 简化版：计算最近点的误差
    errors = []
    for i, car_point in car_traj.iterrows():
        distances = np.sqrt((ref_path['x'] - car_point['x'])**2 + (ref_path['y'] - car_point['y'])**2)
        min_error = distances.min()
        errors.append(min_error)

    avg_error = np.mean(errors)
    max_error = np.max(errors)

    # 设置图表属性
    plt.title(f'巴哈赛车轨迹跟踪\n平均误差: {avg_error:.3f}m, 最大误差: {max_error:.3f}m', fontsize=14)
    plt.xlabel('X 位置 (m)', fontsize=12)
    plt.ylabel('Y 位置 (m)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.legend(fontsize=10)

    # 添加误差信息文本框
    error_text = f'''
平均误差: {avg_error:.3f} m
最大误差: {max_error:.3f} m
轨迹点数: {len(car_traj)}
参考点数: {len(ref_path)}
'''

    plt.text(0.02, 0.98, error_text, transform=plt.gca().transAxes,
             verticalalignment='top', fontsize=10,
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"📊 图表已保存到: {save_path}")

    plt.show()

def main():
    # 加载数据
    ref_path, car_traj = load_trajectory_data('build/trajectory.csv')

    print("📊 轨迹数据统计:")
    print(f"  参考路径点数: {len(ref_path)}")
    print(f"  车辆轨迹点数: {len(car_traj)}")

    # 绘制轨迹
    plot_trajectory(ref_path, car_traj, 'trajectory_plot.png')

if __name__ == "__main__":
    main()
