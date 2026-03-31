#!/usr/bin/env python3
"""
标定质量评估报告可视化分析脚本
用于分析 front3_calibration_metrics.txt 文件
"""
import re
import sys
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

def parse_metrics_file(filepath):
    """解析评估报告文件"""
    with open(filepath, 'r') as f:
        content = f.read()

    metrics = {
        'FL-F': {},
        'F-FR': {},
        'overall': 0
    }

    # 解析总体得分
    overall_match = re.search(r'Overall Score:\s*([\d.]+)', content)
    if overall_match:
        metrics['overall'] = float(overall_match.group(1))

    # 解析各相机对指标
    pairs = ['FL-F', 'F-FR']
    for pair in pairs:
        # 查找对应部分
        pattern = rf'{pair}[^:]*:\s*([\d.]+)' + r'.*?Status:\s*(\w+)'
        matches = re.findall(rf'{pair} Photometric Loss:\s*Value:\s*([\d.]+).*?Score:\s*([\d.]+).*?Status:\s*(\w+)', content, re.DOTALL)

        if matches:
            metrics[pair]['photometric'] = {
                'value': float(matches[0][0]),
                'score': float(matches[0][1]),
                'status': matches[0][2]
            }

        # SSIM
        ssim_match = re.search(rf'{pair} SSIM:\s*Value:\s*([\d.]+).*?Score:\s*([\d.]+).*?Status:\s*(\w+)', content, re.DOTALL)
        if ssim_match:
            metrics[pair]['ssim'] = {
                'value': float(ssim_match.group(1)),
                'score': float(ssim_match.group(2)),
                'status': ssim_match.group(3)
            }

        # Edge Alignment
        edge_match = re.search(rf'{pair} Edge Alignment:\s*Value:\s*([\d.]+).*?Score:\s*([\d.]+).*?Status:\s*(\w+)', content, re.DOTALL)
        if edge_match:
            metrics[pair]['edge'] = {
                'value': float(edge_match.group(1)),
                'score': float(edge_match.group(2)),
                'status': edge_match.group(3)
            }

        # Feature Matching
        feat_match = re.search(rf'{pair} Feature Matching:\s*Value:\s*([\d.]+).*?Score:\s*([\d.]+).*?Status:\s*(\w+)', content, re.DOTALL)
        if feat_match:
            metrics[pair]['feature'] = {
                'value': float(feat_match.group(1)),
                'score': float(feat_match.group(2)),
                'status': feat_match.group(3)
            }

    return metrics

def plot_metrics(metrics, output_path=None):
    """绘制评估指标雷达图和柱状图"""
    fig = plt.figure(figsize=(16, 6))

    # 准备数据
    pairs = ['FL-F', 'F-FR']
    metric_names = ['Photometric', 'SSIM', 'Edge Align', 'Feature']

    # 提取分数
    scores = {}
    for pair in pairs:
        scores[pair] = [
            metrics[pair].get('photometric', {}).get('score', 0),
            metrics[pair].get('ssim', {}).get('score', 0),
            metrics[pair].get('edge', {}).get('score', 0),
            metrics[pair].get('feature', {}).get('score', 0)
        ]

    # 子图1: 柱状图对比
    ax1 = fig.add_subplot(131)
    x = np.arange(len(metric_names))
    width = 0.35

    bars1 = ax1.bar(x - width/2, scores['FL-F'], width, label='FL-F', color='skyblue')
    bars2 = ax1.bar(x + width/2, scores['F-FR'], width, label='F-FR', color='lightcoral')

    ax1.set_ylabel('Score (0-100)')
    ax1.set_title('Calibration Quality Metrics Comparison')
    ax1.set_xticks(x)
    ax1.set_xticklabels(metric_names)
    ax1.legend()
    ax1.set_ylim(0, 100)
    ax1.axhline(y=80, color='g', linestyle='--', alpha=0.5, label='Excellent')
    ax1.axhline(y=60, color='y', linestyle='--', alpha=0.5, label='Good')
    ax1.axhline(y=40, color='r', linestyle='--', alpha=0.5, label='Fair')

    # 在柱状图上添加数值
    for bar in bars1:
        height = bar.get_height()
        ax1.annotate(f'{height:.1f}',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 3), textcoords="offset points",
                    ha='center', va='bottom', fontsize=8)
    for bar in bars2:
        height = bar.get_height()
        ax1.annotate(f'{height:.1f}',
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 3), textcoords="offset points",
                    ha='center', va='bottom', fontsize=8)

    # 子图2: FL-F 雷达图
    ax2 = fig.add_subplot(132, projection='polar')
    angles = np.linspace(0, 2 * np.pi, len(metric_names), endpoint=False).tolist()
    angles += angles[:1]  # 闭合

    values_flf = scores['FL-F'] + scores['FL-F'][:1]
    ax2.plot(angles, values_flf, 'o-', linewidth=2, label='FL-F', color='skyblue')
    ax2.fill(angles, values_flf, alpha=0.25, color='skyblue')

    ax2.set_xticks(angles[:-1])
    ax2.set_xticklabels(metric_names)
    ax2.set_ylim(0, 100)
    ax2.set_title(f'FL-F Pair\nOverall: {metrics["overall"]:.1f}/100', pad=20)
    ax2.grid(True)

    # 子图3: F-FR 雷达图
    ax3 = fig.add_subplot(133, projection='polar')
    values_ffr = scores['F-FR'] + scores['F-FR'][:1]
    ax3.plot(angles, values_ffr, 'o-', linewidth=2, label='F-FR', color='lightcoral')
    ax3.fill(angles, values_ffr, alpha=0.25, color='lightcoral')

    ax3.set_xticks(angles[:-1])
    ax3.set_xticklabels(metric_names)
    ax3.set_ylim(0, 100)
    ax3.set_title(f'F-FR Pair\nOverall: {metrics["overall"]:.1f}/100', pad=20)
    ax3.grid(True)

    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {output_path}")
    else:
        plt.show()

def print_summary(metrics):
    """打印摘要信息"""
    print("\n" + "="*60)
    print("        Calibration Quality Summary")
    print("="*60)

    overall = metrics['overall']
    status = "UNKNOWN"
    if overall >= 80: status = "EXCELLENT"
    elif overall >= 60: status = "GOOD"
    elif overall >= 40: status = "FAIR"
    else: status = "POOR"

    print(f"\nOverall Score: {overall:.1f}/100 [{status}]")
    print()

    for pair in ['FL-F', 'F-FR']:
        print(f"\n{pair} Pair:")
        print("-" * 40)
        m = metrics[pair]

        if 'photometric' in m:
            p = m['photometric']
            print(f"  Photometric Loss: {p['value']:.2f} (Score: {p['score']:.1f}) [{p['status']}]")

        if 'ssim' in m:
            s = m['ssim']
            print(f"  SSIM: {s['value']:.3f} (Score: {s['score']:.1f}) [{s['status']}]")

        if 'edge' in m:
            e = m['edge']
            print(f"  Edge Alignment: {e['value']:.3f} (Score: {e['score']:.1f}) [{e['status']}]")

        if 'feature' in m:
            f = m['feature']
            print(f"  Feature Matching: {f['value']:.0f} pts (Score: {f['score']:.1f}) [{f['status']}]")

    print("\n" + "="*60)

    # 建议
    print("\nRecommendations:")
    if overall >= 80:
        print("  ✓ Calibration quality is excellent. No further adjustment needed.")
    elif overall >= 60:
        print("  ~ Good quality, but minor adjustments may improve results.")
        print("    Focus on improving the lowest scoring metric.")
    elif overall >= 40:
        print("  ⚠ Fair quality. Further calibration is recommended.")
        print("    Check the edge alignment and feature matching metrics.")
    else:
        print("  ✗ Poor quality. Significant adjustment is required.")
        print("    Re-examine the intrinsic parameters and initial extrinsics.")

    print()

def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_metrics.py <metrics_file> [output_image.png]")
        print("Example: python analyze_metrics.py front3_calibration_metrics.txt metrics_plot.png")
        sys.exit(1)

    metrics_file = sys.argv[1]
    output_image = sys.argv[2] if len(sys.argv) > 2 else None

    if not Path(metrics_file).exists():
        print(f"Error: File not found: {metrics_file}")
        sys.exit(1)

    # 解析文件
    metrics = parse_metrics_file(metrics_file)

    # 打印摘要
    print_summary(metrics)

    # 绘制图表
    try:
        plot_metrics(metrics, output_image)
    except Exception as e:
        print(f"Warning: Could not generate plot: {e}")
        print("Make sure matplotlib is installed: pip install matplotlib")

if __name__ == '__main__':
    main()
