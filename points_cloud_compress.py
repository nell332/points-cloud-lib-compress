import os
import yaml
import logging
import numpy as np
import open3d as o3d

from pathlib import Path
from typing import Tuple, Dict, Any


class PointCloudProcessor:
    def __init__(self, config_path: str = "config.yaml"):
        """
        初始化点云处理器

        Args:
            config_path: 配置文件路径
        """
        self.config = self.load_config(config_path)
        self.setup_logging()

    def load_config(self, config_path: str) -> Dict[str, Any]:
        """加载配置文件"""
        try:
            with open(config_path, 'r', encoding='utf-8') as file:
                config = yaml.safe_load(file)
            logging.info(f"配置文件加载成功: {config_path}")
            return config
        except Exception as e:
            logging.error(f"配置文件加载失败: {e}")
            raise

    def setup_logging(self):
        """设置日志"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )

    def load_point_cloud(self) -> np.ndarray:
        """
        加载点云数据

        Returns:
            point_cloud: Nx3的numpy数组 [x, y, z]
        """
        file_path = self.config['point_cloud']['file_path']
        file_format = self.config['point_cloud']['file_format']

        if not Path(file_path).exists():
            raise FileNotFoundError(f"点云文件不存在: {file_path}")

        try:
            if file_format.lower() in ['ply', 'pcd']:
                # 使用Open3D加载PLY/PCD文件
                pcd = o3d.io.read_point_cloud(file_path)
                points = np.asarray(pcd.points)
            elif file_format.lower() in ['xyz', 'txt']:
                # 加载XYZ/TXT格式（每行x y z）
                points = np.loadtxt(file_path)
            else:
                raise ValueError(f"不支持的文件格式: {file_format}")

            logging.info(f"点云加载成功: {points.shape[0]} 个点")
            return points

        except Exception as e:
            logging.error(f"点云加载失败: {e}")
            raise

    def crop_point_cloud(self, points: np.ndarray) -> np.ndarray:
        """
        根据配置范围截取点云

        Args:
            points: 原始点云数据

        Returns:
            cropped_points: 截取后的点云
        """
        crop_config = self.config['point_cloud']['crop_range']
        x_min, x_max = crop_config['x']
        y_min, y_max = crop_config['y']
        z_min, z_max = crop_config['z']

        # 创建掩码筛选范围内的点
        mask = (
                (points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
                (points[:, 1] >= y_min) & (points[:, 1] <= y_max) &
                (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        )

        cropped_points = points[mask]

        logging.info(f"点云截取完成: {cropped_points.shape[0]} 个点 (原始: {points.shape[0]})")
        return cropped_points

    def points_to_grid(self, points: np.ndarray) -> np.ndarray:
        """
        将点云转换为二维网格数组

        Args:
            points: 点云数据 [x, y, z]

        Returns:
            grid: 二维数组，x,y为坐标，z为值
        """
        grid_config = self.config['point_cloud']['grid']
        resolution = grid_config['resolution']
        fill_value = grid_config['fill_value']
        method = grid_config['method']

        # 计算网格范围
        x_min, x_max = self.config['point_cloud']['crop_range']['x']
        y_min, y_max = self.config['point_cloud']['crop_range']['y']

        # 计算网格尺寸
        cols = int((x_max - x_min) / resolution) + 1
        rows = int((y_max - y_min) / resolution) + 1

        # 初始化网格
        grid = np.full((rows, cols), fill_value, dtype=np.float32)

        # 计算每个点在网格中的位置
        x_coords = ((points[:, 0] - x_min) / resolution).astype(int)
        y_coords = ((points[:, 1] - y_min) / resolution).astype(int)

        # 确保坐标在网格范围内
        valid_mask = (x_coords >= 0) & (x_coords < cols) & (y_coords >= 0) & (y_coords < rows)
        x_coords = x_coords[valid_mask]
        y_coords = y_coords[valid_mask]
        z_values = points[valid_mask, 2]

        if method == "max":
            # 取每个网格中的最大z值
            for x, y, z in zip(x_coords, y_coords, z_values):
                if grid[y, x] == fill_value or z > grid[y, x]:
                    grid[y, x] = z

        elif method == "min":
            # 取每个网格中的最小z值
            for x, y, z in zip(x_coords, y_coords, z_values):
                if grid[y, x] == fill_value or z < grid[y, x]:
                    grid[y, x] = z

        elif method == "mean":
            # 计算每个网格的平均z值
            sum_grid = np.zeros((rows, cols), dtype=np.float32)
            count_grid = np.zeros((rows, cols), dtype=np.int32)

            for x, y, z in zip(x_coords, y_coords, z_values):
                sum_grid[y, x] += z
                count_grid[y, x] += 1

            valid_cells = count_grid > 0
            grid[valid_cells] = sum_grid[valid_cells] / count_grid[valid_cells]

        elif method == "nearest":
            # 取每个网格中最近的点（基于索引）
            for x, y, z in zip(x_coords, y_coords, z_values):
                grid[y, x] = z

        else:
            raise ValueError(f"不支持的网格化方法: {method}")

        logging.info(f"网格化完成: {grid.shape}, 有效单元格: {np.sum(grid != fill_value)}")
        return grid

    def save_results(self, grid: np.ndarray):
        """保存结果"""
        output_config = self.config['point_cloud']['output']

        if output_config['save_npy']:
            output_path = output_config['output_path']
            np.save(output_path, grid)
            logging.info(f"网格数据保存为: {output_path}")

        if output_config['save_txt']:
            txt_path = Path(output_config['output_path']).with_suffix('.txt')
            np.savetxt(txt_path, grid, fmt='%.6f')
            logging.info(f"网格数据保存为: {txt_path}")

    def process(self) -> np.ndarray:
        """
        完整的处理流程

        Returns:
            grid: 最终的网格数据
        """
        logging.info("开始处理点云数据...")

        # 1. 加载点云
        points = self.load_point_cloud()

        # 2. 截取点云
        cropped_points = self.crop_point_cloud(points)

        # 3. 转换为网格
        grid = self.points_to_grid(cropped_points)

        # 4. 保存结果
        self.save_results(grid)

        logging.info("点云处理完成!")
        return grid


def main():
    """主函数"""
    try:
        processor = PointCloudProcessor("config.yaml")
        grid = processor.process()

        # 打印一些统计信息
        fill_value = processor.config['point_cloud']['grid']['fill_value']
        valid_data = grid[grid != fill_value]

        print(f"\n网格统计信息:")
        print(f"网格尺寸: {grid.shape}")
        print(f"有效数据点: {len(valid_data)}")
        print(f"Z值范围: [{np.min(valid_data):.3f}, {np.max(valid_data):.3f}]")
        print(f"Z值均值: {np.mean(valid_data):.3f}")

    except Exception as e:
        logging.error(f"处理失败: {e}")


if __name__ == "__main__":
    main()