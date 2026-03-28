#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OCAM去畸变工具 + iKalibr参数导出（优化版）

功能：
1. 基于cpac仓库的正确实现，将鱼眼图像去畸变
2. 支持单张图像或批量目录处理
3. GPU加速和多线程支持
4. 生成iKalibr兼容的Pinhole相机内参文件

作者：基于cpac、mkc、static_sensor_calibration仓库的实现
日期：2026-03-21
"""

import cv2
import numpy as np
import json
import yaml
import argparse
import os
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

# 尝试导入CUDA（GPU加速）
try:
    import cupy as cp
    CUDA_AVAILABLE = True
    print("✓ CUDA可用，启用GPU加速")
except ImportError:
    CUDA_AVAILABLE = False
    import numpy as cp
    print("✗ CUDA不可用，使用CPU处理")


class OCameraModel:
    """
    OCam相机模型实现
    基于cpac仓库的ocam_model.py
    """
    def __init__(self, calib_file):
        """加载OCAM标定参数"""
        with open(calib_file, 'r') as f:
            calib = json.load(f)

        # 基本参数
        self.width = int(calib['intrinsic_param']['camera_width'])
        self.height = int(calib['intrinsic_param']['camera_height'])
        self.cx = calib['intrinsic_param']['principal_point'][0]
        self.cy = calib['intrinsic_param']['principal_point'][1]

        # 仿射变换参数
        self.c = calib['intrinsic_param']['affine_c']
        self.d = calib['intrinsic_param']['affine_d']
        self.e = calib['intrinsic_param']['affine_e']

        # 多项式系数（注意：命名反了！）
        # cam2world用于pixel2cam（2D→3D）
        # world2cam用于cam2pixel（3D→2D）
        self.cam2world_poly = np.array(calib['intrinsic_param']['cam2world'])
        self.world2cam_poly = np.array(calib['intrinsic_param']['world2cam'])

        # 仿射变换矩阵
        self.A = np.array([[1.0, self.e], [self.d, self.c]])
        self.B = np.array([self.cx, self.cy])

        # CUDA加速：将numpy数组转换为cupy数组
        if CUDA_AVAILABLE:
            self.cp = cp  # 保存GPU计算库引用
            self.cam2world_poly = self.cp.asarray(self.cam2world_poly)
            self.world2cam_poly = self.cp.asarray(self.world2cam_poly)
            self.A = self.cp.asarray(self.A)
            self.B = self.cp.asarray(self.B)
        else:
            self.cp = np  # CPU模式下使用numpy作为计算库

    def _evaluate_polynomial(self, coeffs, x):
        """
        计算多项式值：coeffs[0] + coeffs[1]*x + coeffs[2]*x^2 + ...
        使用Horner's method，从高次到低次（从最后一个系数开始）
        """
        result = 0.0
        for coeff in reversed(coeffs):
            result = result * x + coeff
        return result

    def world_to_image_batch(self, rays):
        """
        批量3D射线 → 图像坐标（使用world2cam多项式）
        rays: (N, 3) 归一化的3D方向向量
        返回: (N, 2) 图像坐标
        """
        # 确保self.cp可用（用于CPU/GPU兼容）
        assert hasattr(self, 'cp'), "self.cp未定义，检查CUDA_AVAILABLE逻辑"

        x = rays[:, 0]
        y = rays[:, 1]
        z = rays[:, 2]

        # 标准 → Omni坐标系：z取负
        z_omni = -z

        # 计算xy平面上的距离
        norm = self.cp.sqrt(x*x + y*y)
        norm = self.cp.maximum(norm, 1e-6)

        # 计算入射角theta
        theta = self.cp.arctan(z_omni / norm)

        # 使用world2cam多项式计算rho
        rho = self._evaluate_polynomial(self.world2cam_poly, theta)

        # 投影到归一化平面
        xy = self.cp.stack([x, y], axis=1) * rho[:, self.cp.newaxis] / norm[:, self.cp.newaxis]

        # 应用仿射变换
        p = xy @ self.A.T + self.B[self.cp.newaxis, :]

        # CPU模式：直接返回numpy数组
        # GPU模式：转换为numpy数组
        if self.cp is np:
            return p
        else:
            return self.cp.asnumpy(p)

    def world_to_image_single(self, ray):
        """
        单个3D射线 → 图像坐标（用于FOV计算）
        ray: (3,) 归一化的3D方向向量
        返回: u, v 图像坐标
        """
        x, y, z = ray

        # 标准 → Omni坐标系：z取负
        z_omni = -z

        # 计算xy平面上的距离
        norm = np.sqrt(x*x + y*y)
        if norm < 1e-6:
            norm = 1e-6

        # 计算入射角theta
        theta = np.arctan(z_omni / norm)

        # 使用world2cam多项式计算rho
        rho = self._evaluate_polynomial(self.world2cam_poly, theta)

        # 投影到归一化平面
        xy = np.array([x, y]) * rho / norm

        # 应用仿射变换
        p = self.A @ xy + self.B

        return p[0], p[1]


class Undistorter:
    """
    OCam去畸变器（支持GPU加速和多线程）
    """
    def __init__(self, calib_file, focal_length=1000, fov_scale=1.3,
                 output_scale=1.0, use_fov_limit=True):
        """
        初始化去畸变器

        参数:
        - calib_file: 标定文件路径
        - focal_length: 去畸变后的焦距
        - fov_scale: 视野范围缩放因子（>1扩大视野，<1缩小视野）
        - output_scale: 输出图像尺寸缩放因子
        - use_fov_limit: 是否自动限制FOV以避免边缘黑边
        """
        self.cam_model = OCameraModel(calib_file)

        # 去畸变后的相机参数
        self.focal_length = focal_length
        self.undistort_fx = focal_length
        self.undistort_fy = focal_length

        # 输出图像尺寸
        self.undistort_width = int(self.cam_model.width * output_scale)
        self.undistort_height = int(self.cam_model.height * output_scale)

        # 输出主点（图像中心）
        self.undistort_cx = self.undistort_width / 2
        self.undistort_cy = self.undistort_height / 2

        # FOV参数
        self.fov_scale = fov_scale
        self.use_fov_limit = use_fov_limit

        # 自动计算合适的FOV范围
        if use_fov_limit:
            self.max_valid_theta = self._compute_max_valid_theta()
        else:
            self.max_valid_theta = None

    def _compute_max_valid_theta(self):
        """
        计算最大有效角度，避免映射到原始图像边界外
        使用二分法快速找到最大有效角度，留5%余量
        """
        theta_min = 0.0
        theta_max = np.pi / 2

        # 二分法查找
        for _ in range(30):
            theta_mid = (theta_min + theta_max) / 2

            # 检查四个方向是否都有效
            all_valid = True
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                # 计算射线方向
                ray = np.array([dx, dy, 1.0])
                ray = ray / np.linalg.norm(ray)

                # 映射到原始图像坐标
                u, v = self.cam_model.world_to_image_single(ray)

                # 检查是否在原始图像范围内
                if u < 0 or u >= self.cam_model.width or v < 0 or v >= self.cam_model.height:
                    all_valid = False
                    break

            if all_valid:
                theta_min = theta_mid
            else:
                theta_max = theta_mid

        # 留5%余量
        max_theta = theta_min * 0.95
        return max_theta

    def create_undistortion_maps(self):
        """创建去畸变映射表"""
        height = self.undistort_height
        width = self.undistort_width

        # 使用GPU或CPU创建坐标网格
        if CUDA_AVAILABLE:
            import cupy as cp
            y_indices, x_indices = cp.mgrid[0:height, 0:width]
            y_indices = y_indices.astype(cp.float32)
            x_indices = x_indices.astype(cp.float32)
            xp = cp  # 使用CuPy作为计算库
        else:
            y_indices, x_indices = np.mgrid[0:height, 0:width]
            y_indices = y_indices.astype(np.float32)
            x_indices = x_indices.astype(np.float32)
            xp = np  # 使用NumPy作为计算库

        # 计算归一化坐标
        y_norm = (y_indices - self.undistort_cy) / self.undistort_fy
        x_norm = (x_indices - self.undistort_cx) / self.undistort_fx

        # 应用FOV缩放
        x_norm_scaled = x_norm * self.fov_scale
        y_norm_scaled = y_norm * self.fov_scale

        # 计算归一化半径
        norm_radius = xp.sqrt(x_norm_scaled**2 + y_norm_scaled**2)

        # 应用FOV限制
        if self.max_valid_theta:
            max_allowed_norm_radius = np.tan(self.max_valid_theta)
            # 超出FOV范围，标记为无效
            mask = norm_radius > max_allowed_norm_radius

        # 如果没有FOV限制，所有像素都有效
        else:
            mask = xp.zeros_like(norm_radius, dtype=bool)

        # 归一化为3D射线方向
        norm = xp.sqrt(x_norm_scaled**2 + y_norm_scaled**2 + 1.0)
        rays = xp.stack([x_norm_scaled, y_norm_scaled, xp.ones_like(x_norm_scaled)], axis=2)
        rays = rays / norm[:, :, xp.newaxis]

        # 将3D射线展平为(N, 3)
        rays_flat = rays.reshape(-1, 3)

        # 使用OCAM模型批量映射到原图像坐标
        image_coords = self.cam_model.world_to_image_batch(rays_flat)

        # 重塑为(height, width, 2)
        map_x = image_coords[:, 0].reshape(height, width).astype(np.float32)
        map_y = image_coords[:, 1].reshape(height, width).astype(np.float32)

        # 应用FOV限制（如果有）
        if self.max_valid_theta:
            if CUDA_AVAILABLE:
                mask_np = cp.asnumpy(mask)
            else:
                mask_np = mask
            map_x[mask_np] = -1
            map_y[mask_np] = -1

        return map_x, map_y

    def undistort(self, img, show_grid=False, border_color=(0, 0, 0)):
        """
        对图像进行去畸变

        参数:
        - img: 输入图像
        - show_grid: 是否显示网格
        - border_color: 边界填充颜色
        """
        # 创建映射表
        self.map_x, self.map_y = self.create_undistortion_maps()

        # 使用cv2.remap进行去畸变
        undistorted = cv2.remap(img, self.map_x, self.map_y, cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT,
                               borderValue=border_color)

        # 如果需要，绘制网格
        if show_grid:
            undistorted = self._draw_grid(undistorted)

        return undistorted

    def _draw_grid(self, img):
        """绘制网格线"""
        h, w = img.shape[:2]

        # 绘制垂直线
        for x in range(100, w, 200):
            cv2.line(img, (x, 0), (x, h), (0, 255, 0), 1)

        # 绘制水平线
        for y in range(100, h, 200):
            cv2.line(img, (0, y), (w, y), (0, 255, 0), 1)

        # 绘制中心十字
        cv2.line(img, (w//2 - 50, h//2), (w//2 + 50, h//2), (255, 0, 0), 2)
        cv2.line(img, (w//2, h//2 - 50), (w//2, h//2 + 50), (255, 0, 0), 2)

        return img


def process_single_image(args):
    """处理单张图像（用于多线程）"""
    calib_file, input_path, output_path, focal_length, fov_scale, output_scale, use_fov_limit, show_grid = args

    # 读取图像
    img = cv2.imread(input_path)
    if img is None:
        print(f"✗ 无法读取图像: {input_path}")
        return False, input_path

    # 创建去畸变器
    undistorter = Undistorter(
        calib_file,
        focal_length=focal_length,
        fov_scale=fov_scale,
        output_scale=output_scale,
        use_fov_limit=use_fov_limit
    )

    # 去畸变
    start_time = time.time()
    undistorted_img = undistorter.undistort(img, show_grid=show_grid)
    elapsed = time.time() - start_time

    # 保存结果
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    cv2.imwrite(output_path, undistorted_img)

    return True, input_path


def process_directory(input_dir, output_dir, calib_file, focal_length, fov_scale,
                   output_scale, use_fov_limit, show_grid, num_workers=4):
    """
    批量处理目录中的所有图像

    参数:
    - input_dir: 输入图像目录
    - output_dir: 输出图像目录
    - calib_file: 标定文件路径
    - focal_length: 焦距
    - fov_scale: FOV缩放
    - output_scale: 输出缩放
    - use_fov_limit: 是否启用FOV限制
    - show_grid: 是否绘制网格
    - num_workers: 线程数
    """
    # 支持的图像格式
    image_extensions = ['.png', '.jpg', '.jpeg', '.bmp', '.tiff']

    # 查找所有图像
    image_files = []
    for ext in image_extensions:
        image_files.extend(Path(input_dir).glob(f'*{ext}'))
        image_files.extend(Path(input_dir).glob(f'*{ext.upper()}'))

    if not image_files:
        print(f"✗ 在目录 {input_dir} 中未找到图像文件")
        return

    print(f"找到 {len(image_files)} 张图像")

    # 准备任务
    tasks = []
    for img_path in image_files:
        output_path = Path(output_dir) / img_path.name
        task_args = (calib_file, str(img_path), str(output_path),
                     focal_length, fov_scale, output_scale, use_fov_limit, show_grid)
        tasks.append(task_args)

    # 使用线程池处理
    print(f"\n使用 {num_workers} 个线程批量处理...")
    print("="*70)

    success_count = 0
    fail_count = 0

    with ThreadPoolExecutor(max_workers=num_workers) as executor:
        future_to_path = {
            executor.submit(process_single_image, task): task[1]
            for task in tasks
        }

        for future in as_completed(future_to_path):
            success, input_path = future.result()
            if success:
                success_count += 1
                progress = (success_count + fail_count) / len(tasks) * 100
                print(f"✓ [{success_count + fail_count}/{len(tasks)}] {progress:.1f}% - {input_path}")
            else:
                fail_count += 1
                print(f"✗ [{success_count + fail_count}/{len(tasks)}] - {input_path}")

    print("="*70)
    print(f"处理完成: 成功 {success_count} 张, 失败 {fail_count} 张")


def generate_ikalibr_intrinsics(calib_file, output_file, focal_length):
    """
    生成iKalibr兼容的Pinhole相机内参文件

    参数:
    - calib_file: OCam标定JSON文件
    - output_file: 输出的YAML文件
    - focal_length: 去畸变时使用的焦距
    """
    # 加载OCam校准参数
    with open(calib_file, 'r') as f:
        calib = json.load(f)

    intrinsic = calib['intrinsic_param']
    width = intrinsic['camera_width']
    height = intrinsic['camera_height']
    camera_name = calib.get('name', 'camera')

    # iKalibr使用的针孔内参
    ikalibr_intrinsics = {
        'camera_model': 'pinhole',
        'image_width': width,
        'image_height': height,
        'intrinsic_parameters': {
            'fx': float(focal_length),
            'fy': float(focal_length),
            'cx': float(width / 2.0),
            'cy': float(height / 2.0)
        },
        'distortion_coefficients': {
            'k1': 0.0,
            'k2': 0.0,
            'p1': 0.0,
            'p2': 0.0,
            'k3': 0.0
        },
        'notes': 'Generated from OCam calibration for iKalibr usage'
    }

    # 保存为YAML
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, 'w') as f:
        yaml.dump(ikalibr_intrinsics, f, default_flow_style=False, sort_keys=False)

    return output_file


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='OCAM去畸变工具 + iKalibr参数导出（优化版：GPU+多线程）',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    # 子命令
    subparsers = parser.add_subparsers(dest='command', help='子命令')

    # 去畸变子命令
    undistort_parser = subparsers.add_parser('undistort', help='去畸变图像')

    # 单张图像去畸变
    single_group = undistort_parser.add_mutually_exclusive_group(required=True)
    single_group.add_argument('--image', type=str,
                              help='单张输入图像路径（与--input-dir互斥）')
    single_group.add_argument('--input-dir', type=str,
                              help='输入图像目录路径（批量处理，与--image互斥）')

    # 其他参数
    undistort_parser.add_argument('--calib', type=str, required=True,
                                 help='OCam标定文件路径')
    undistort_parser.add_argument('--output', type=str, required=True,
                                 help='输出图像路径（单张）或输出目录（批量）')
    undistort_parser.add_argument('--focal', type=float, default=1000,
                                 help='去畸变后的焦距（默认：1000）')
    undistort_parser.add_argument('--fov-scale', type=float, default=1.3,
                                 help='FOV缩放因子（默认：1.3）')
    undistort_parser.add_argument('--output-scale', type=float, default=1.0,
                                 help='输出图像尺寸缩放因子（默认：1.0）')
    undistort_parser.add_argument('--no-fov-limit', action='store_true',
                                 help='禁用FOV自动限制（允许边缘黑边）')
    undistort_parser.add_argument('--grid', action='store_true',
                                 help='在输出图像上绘制网格')
    undistort_parser.add_argument('--workers', type=int, default=4,
                                 help='多线程处理时使用的线程数（默认：4，批量处理时生效）')

    # iKalibr参数导出子命令
    export_parser = subparsers.add_parser('export', help='导出iKalibr参数')

    export_parser.add_argument('--calib', type=str, required=True,
                           help='OCam标定文件路径')
    export_parser.add_argument('--output', type=str, required=True,
                           help='输出的iKalibr内参YAML文件路径')
    export_parser.add_argument('--focal', type=float, default=1000,
                           help='去畸变时使用的焦距（默认：1000）')

    args = parser.parse_args()

    # 执行对应的子命令
    if args.command == 'undistort':
        # 去畸变
        print("="*70)
        print("OCAM去畸变工具（GPU加速 + 多线程）")
        print("="*70)
        print(f"GPU加速: {'启用' if CUDA_AVAILABLE else '禁用'}")

        # 检查是单张还是批量
        if args.image:
            # 单张图像
            print(f"\n单张图像模式")
            print(f"标定文件: {args.calib}")
            print(f"输入图像: {args.image}")
            print(f"输出图像: {args.output}")

            img = cv2.imread(args.image)
            if img is None:
                print(f"✗ 错误: 无法读取图像 {args.image}")
                return

            print(f"输入图像尺寸: {img.shape}")

            # 创建去畸变器
            undistorter = Undistorter(
                args.calib,
                focal_length=args.focal,
                fov_scale=args.fov_scale,
                output_scale=args.output_scale,
                use_fov_limit=not args.no_fov_limit
            )

            print(f"\n去畸变参数:")
            print(f"  焦距: fx=fy={args.focal}")
            print(f"  输出尺寸: {undistorter.undistort_width}x{undistorter.undistort_height}")
            print(f"  FOV缩放: {args.fov_scale}")
            print(f"  输出缩放: {args.output_scale}")
            if undistorter.max_valid_theta:
                max_fov_deg = np.rad2deg(undistorter.max_valid_theta)
                print(f"  最大有效角度: {max_fov_deg:.1f}°")
                print(f"  有效FOV: {max_fov_deg*2:.1f}°")

            # 去畸变
            start_time = time.time()
            undistorted_img = undistorter.undistort(img, show_grid=args.grid)
            elapsed = time.time() - start_time

            # 保存结果
            os.makedirs(os.path.dirname(args.output), exist_ok=True)
            cv2.imwrite(args.output, undistorted_img)

            print(f"\n✓ 处理完成，耗时: {elapsed:.2f}秒")
            print(f"✓ 输出图像: {args.output}")

        elif args.input_dir:
            # 批量处理
            print(f"\n批量处理模式")
            print(f"标定文件: {args.calib}")
            print(f"输入目录: {args.input_dir}")
            print(f"输出目录: {args.output}")
            print(f"线程数: {args.workers}")

            print(f"\n去畸变参数:")
            print(f"  焦距: fx=fy={args.focal}")
            print(f"  FOV缩放: {args.fov_scale}")
            print(f"  输出缩放: {args.output_scale}")

            process_directory(
                args.input_dir,
                args.output,
                args.calib,
                args.focal,
                args.fov_scale,
                args.output_scale,
                not args.no_fov_limit,
                args.grid,
                args.workers
            )

    elif args.command == 'export':
        # 导出iKalibr参数
        print("="*70)
        print("生成iKalibr内参文件")
        print("="*70)
        print(f"标定文件: {args.calib}")
        print(f"输出文件: {args.output}")
        print(f"焦距: {args.focal}")

        # 生成配置文件
        output_file = generate_ikalibr_intrinsics(args.calib, args.output, args.focal)

        print(f"\n✓ 生成成功: {output_file}")

        # 读取并显示配置
        with open(output_file, 'r') as f:
            config = yaml.safe_load(f)

        print(f"\n内参矩阵:")
        print(f"  fx = {config['intrinsic_parameters']['fx']:.2f}")
        print(f"  fy = {config['intrinsic_parameters']['fy']:.2f}")
        print(f"  cx = {config['intrinsic_parameters']['cx']:.2f}")
        print(f"  cy = {config['intrinsic_parameters']['cy']:.2f}")
        print(f"\n畸变系数: 全部为0（已去畸变）")

    else:
        parser.print_help()


if __name__ == "__main__":
    main()

