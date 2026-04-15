import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# ==================== 配置区（只需改这里）====================
CSV_FILE = 'build/parameter_test_results.csv' # 你的CSV文件
OUTPUT_DIR = 'docs/figures'              # 图片保存目录
SHOW_GRID = True                          # 是否显示网格
DPI = 300                                 # 图片分辨率
# ===========================================================

# 创建输出目录
Path(OUTPUT_DIR).mkdir(parents=True, exist_ok=True)

# 读取数据
df = pd.read_csv(CSV_FILE)
print(f"📊 读取到 {len(df)} 组测试数据")
print(df[['algo', 'parameter', 'avg_error', 'steering_jerk']].to_string(index=False))

# ==================== 图1: 平均误差对比 ====================
plt.figure(figsize=(10, 6))
for algo in df['algo'].unique():
    sub = df[df['algo'] == algo]
    plt.plot(sub['parameter'], sub['avg_error'], 'o-', linewidth=2, 
             markersize=8, label=algo)
    
    # 标注最优参数点
    best_idx = sub['avg_error'].idxmin()
    best_param = sub.loc[best_idx, 'parameter']
    best_error = sub.loc[best_idx, 'avg_error']
    plt.annotate(f'最优\n({best_param:.1f}, {best_error:.2f})', 
                 xy=(best_param, best_error), 
                 xytext=(5, 10), textcoords='offset points',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.5),
                 arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))

plt.xlabel('Parameter Value (Lookahead Distance / Kp Gain)', fontsize=12)
plt.ylabel('Average Lateral Error (m)', fontsize=12)
plt.title('Algorithm Comparison: Average Tracking Error', fontsize=14, fontweight='bold')
plt.legend(loc='best', fontsize=10)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(f'{OUTPUT_DIR}/error_comparison.png', dpi=DPI, bbox_inches='tight')
print(f"✅ 已保存: {OUTPUT_DIR}/error_comparison.png")
plt.close()

# ==================== 图2: 平滑度对比（Jerk）====================
if 'steering_jerk' in df.columns:
    plt.figure(figsize=(10, 6))
    for algo in df['algo'].unique():
        sub = df[df['algo'] == algo]
        plt.plot(sub['parameter'], sub['steering_jerk'], 's--', linewidth=2, 
                 markersize=8, label=algo)
    
    plt.xlabel('Parameter Value', fontsize=12)
    plt.ylabel('Average Steering Jerk (rad/frame)', fontsize=12)
    plt.title('Control Smoothness: Steering Angle Variation', fontsize=14, fontweight='bold')
    plt.legend(loc='best', fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{OUTPUT_DIR}/steering_jerk.png', dpi=DPI, bbox_inches='tight')
    print(f"✅ 已保存: {OUTPUT_DIR}/steering_jerk.png")
    plt.close()

# ==================== 图3: 综合评分雷达图 ====================
if 'max_steering_util' in df.columns:
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    for idx, algo in enumerate(df['algo'].unique()):
        sub = df[df['algo'] == algo]
        best_idx = sub['avg_error'].idxmin()
        best_row = sub.loc[best_idx]
        
        # 归一化指标（越小越好）
        error_norm = best_row['avg_error'] / df['avg_error'].max()
        jerk_norm = best_row['steering_jerk'] / df['steering_jerk'].max()
        util_norm = best_row['max_steering_util']
        
        categories = ['Tracking\nError', 'Steering\nJerk', 'Steering\nUtilization']
        values = [error_norm, jerk_norm, util_norm]
        
        # 闭合雷达图
        values += values[:1]
        angles = [n / float(len(categories)) * 2 * np.pi for n in range(len(categories))]
        angles += angles[:1]
        
        ax = axes[idx]
        ax.plot(angles, values, 'o-', linewidth=2, label=algo)
        ax.fill(angles, values, alpha=0.25)
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(categories)
        ax.set_ylim(0, 1)
        ax.set_title(f'Performance Radar Chart (Best Parameter)\n{algo}', fontweight='bold')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{OUTPUT_DIR}/radar_comparison.png', dpi=DPI, bbox_inches='tight')
    print(f"✅ 已保存: {OUTPUT_DIR}/radar_comparison.png")
    plt.close()

# ==================== 图4: 参数扫描热力图 ====================
if len(df['algo'].unique()) > 1:
    pivot_df = df.pivot_table(values='avg_error', index='parameter', 
                              columns='algo', aggfunc='first')
    
    plt.figure(figsize=(10, 8))
    im = plt.imshow(pivot_df.values, cmap='YlGnBu_r', aspect='auto')
    plt.colorbar(im, label='Average Error (m)')
    
    # 设置坐标轴
    plt.xticks(range(len(pivot_df.columns)), pivot_df.columns, rotation=45, ha='right')
    plt.yticks(range(len(pivot_df.index)), [f'{p:.1f}' for p in pivot_df.index])
    
    # 在单元格中标注数值
    for i in range(len(pivot_df.index)):
        for j in range(len(pivot_df.columns)):
            value = pivot_df.values[i, j]
            plt.text(j, i, f'{value:.2f}', ha='center', va='center', 
                    color='red' if value < 0.5 else 'black', fontweight='bold')
    
    plt.title('Parameter Scan Heatmap: Average Error', fontsize=14, fontweight='bold')
    plt.xlabel('Algorithm')
    plt.ylabel('Parameter Value')
    plt.tight_layout()
    plt.savefig(f'{OUTPUT_DIR}/parameter_heatmap.png', dpi=DPI, bbox_inches='tight')
    print(f"✅ 已保存: {OUTPUT_DIR}/parameter_heatmap.png")
    plt.close()

print("\n🎉 所有图表生成完毕！可用于静态答辩 F2.5.2 测试验证报告")