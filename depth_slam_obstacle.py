"""
深度图 SLAM 避障系统
基于 Berxel 3D 相机的实时障碍物检测与路径规划

功能:
1. 实时获取深度图数据
2. 障碍物检测与分割
3. 可通行区域分析
4. 简单的路径规划建议
5. 3D点云可视化（可选）

作者: Zhuo
日期: 2025-10-15
"""

import cv2
import numpy as np
from typing import Tuple, List, Optional
import time


class DepthSLAMObstacleDetector:
    """基于深度图的 SLAM 避障检测器"""
    
    def __init__(self, 
                 depth_threshold_near: float = 0.3,  # 近距离阈值(米)
                 depth_threshold_far: float = 5.0,   # 远距离阈值(米)
                 obstacle_height_min: float = 0.1,   # 最小障碍物高度(米)
                 grid_resolution: float = 0.05):      # 网格分辨率(米)
        """
        初始化深度SLAM避障检测器
        
        Args:
            depth_threshold_near: 近距离阈值（小于此值的认为太近）
            depth_threshold_far: 远距离阈值（大于此值的忽略）
            obstacle_height_min: 最小障碍物高度
            grid_resolution: 占据栅格地图的分辨率
        """
        self.depth_threshold_near = depth_threshold_near
        self.depth_threshold_far = depth_threshold_far
        self.obstacle_height_min = obstacle_height_min
        self.grid_resolution = grid_resolution
        
        # 占据栅格地图 (Occupancy Grid)
        self.occupancy_map = None
        self.map_size = (200, 200)  # 10m x 10m @ 5cm分辨率
        self.robot_position = (self.map_size[0] // 2, self.map_size[1] // 2)
        
        # 统计信息
        self.frame_count = 0
        self.processing_times = []
        
    def process_depth_frame(self, depth_image: np.ndarray, 
                           color_image: Optional[np.ndarray] = None) -> Tuple[np.ndarray, dict]:
        """
        处理单帧深度图像
        
        Args:
            depth_image: 深度图 (单位: 米)
            color_image: 彩色图像 (可选)
            
        Returns:
            obstacle_mask: 障碍物掩码
            info: 检测信息字典
        """
        start_time = time.time()
        
        # 1. 深度图预处理
        depth_filtered = self._filter_depth(depth_image)
        
        # 2. 障碍物检测
        obstacle_mask = self._detect_obstacles(depth_filtered)
        
        # 3. 更新占据栅格地图
        self._update_occupancy_map(depth_filtered, obstacle_mask)
        
        # 4. 可通行区域分析
        navigable_zones = self._find_navigable_zones(obstacle_mask)
        
        # 5. 路径规划建议
        suggested_direction = self._suggest_direction(navigable_zones)
        
        # 统计信息
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        self.frame_count += 1
        
        info = {
            'frame_count': self.frame_count,
            'processing_time': processing_time,
            'avg_processing_time': np.mean(self.processing_times[-30:]),
            'obstacle_count': np.sum(obstacle_mask > 0),
            'navigable_zones': navigable_zones,
            'suggested_direction': suggested_direction,
            'min_depth': np.min(depth_filtered[depth_filtered > 0]) if np.any(depth_filtered > 0) else 0,
            'max_depth': np.max(depth_filtered[depth_filtered > 0]) if np.any(depth_filtered > 0) else 0,
        }
        
        return obstacle_mask, info
    
    def _filter_depth(self, depth: np.ndarray) -> np.ndarray:
        """
        深度图滤波与去噪
        
        Args:
            depth: 原始深度图
            
        Returns:
            filtered_depth: 滤波后的深度图
        """
        # 去除无效值
        depth_valid = depth.copy()
        depth_valid[depth_valid <= 0] = 0
        depth_valid[depth_valid > self.depth_threshold_far] = 0
        
        # 中值滤波去噪
        depth_filtered = cv2.medianBlur(depth_valid.astype(np.float32), 5)
        
        # 双边滤波保留边缘
        depth_filtered = cv2.bilateralFilter(
            depth_filtered.astype(np.float32), 9, 75, 75
        )
        
        return depth_filtered
    
    def _detect_obstacles(self, depth: np.ndarray) -> np.ndarray:
        """
        检测障碍物
        
        Args:
            depth: 滤波后的深度图
            
        Returns:
            obstacle_mask: 障碍物掩码 (0=可通行, 255=障碍物)
        """
        h, w = depth.shape
        obstacle_mask = np.zeros((h, w), dtype=np.uint8)
        
        # 1. 近距离障碍物（太近的物体）
        near_obstacles = (depth > 0) & (depth < self.depth_threshold_near)
        obstacle_mask[near_obstacles] = 255
        
        # 2. 使用深度梯度检测障碍物边缘
        # 计算深度梯度
        depth_grad_x = cv2.Sobel(depth, cv2.CV_32F, 1, 0, ksize=3)
        depth_grad_y = cv2.Sobel(depth, cv2.CV_32F, 0, 1, ksize=3)
        depth_grad = np.sqrt(depth_grad_x**2 + depth_grad_y**2)
        
        # 梯度大的区域可能是障碍物边缘
        edge_obstacles = depth_grad > 0.5  # 阈值可调
        obstacle_mask[edge_obstacles] = 255
        
        # 3. 形态学处理：膨胀以增加安全区域
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        obstacle_mask = cv2.dilate(obstacle_mask, kernel, iterations=2)
        
        return obstacle_mask
    
    def _update_occupancy_map(self, depth: np.ndarray, obstacle_mask: np.ndarray):
        """
        更新占据栅格地图
        
        Args:
            depth: 深度图
            obstacle_mask: 障碍物掩码
        """
        if self.occupancy_map is None:
            self.occupancy_map = np.zeros(self.map_size, dtype=np.float32)
        
        # TODO: 实现从深度图到占据栅格的投影
        # 这里需要相机内参才能准确投影
        pass
    
    def _find_navigable_zones(self, obstacle_mask: np.ndarray) -> List[dict]:
        """
        寻找可通行区域
        
        Args:
            obstacle_mask: 障碍物掩码
            
        Returns:
            navigable_zones: 可通行区域列表
        """
        # 反转掩码：可通行区域为白色
        navigable = cv2.bitwise_not(obstacle_mask)
        
        # 连通域分析
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            navigable, connectivity=8
        )
        
        zones = []
        for i in range(1, num_labels):  # 跳过背景
            area = stats[i, cv2.CC_STAT_AREA]
            if area > 1000:  # 最小面积阈值
                zone = {
                    'id': i,
                    'area': area,
                    'centroid': centroids[i],
                    'bbox': (stats[i, cv2.CC_STAT_LEFT],
                            stats[i, cv2.CC_STAT_TOP],
                            stats[i, cv2.CC_STAT_WIDTH],
                            stats[i, cv2.CC_STAT_HEIGHT])
                }
                zones.append(zone)
        
        # 按面积排序
        zones.sort(key=lambda x: x['area'], reverse=True)
        
        return zones
    
    def _suggest_direction(self, navigable_zones: List[dict]) -> str:
        """
        根据可通行区域建议移动方向
        
        Args:
            navigable_zones: 可通行区域列表
            
        Returns:
            direction: 建议方向 ('forward', 'left', 'right', 'stop')
        """
        if not navigable_zones:
            return 'stop'
        
        # 选择最大的可通行区域
        largest_zone = navigable_zones[0]
        centroid_x = largest_zone['centroid'][0]
        
        # 假设图像宽度
        image_width = 640  # TODO: 从实际图像获取
        center_x = image_width / 2
        
        # 根据质心位置决定方向
        if abs(centroid_x - center_x) < image_width * 0.2:
            return 'forward'
        elif centroid_x < center_x:
            return 'left'
        else:
            return 'right'
    
    def visualize(self, depth_image: np.ndarray, 
                  obstacle_mask: np.ndarray,
                  info: dict,
                  color_image: Optional[np.ndarray] = None) -> np.ndarray:
        """
        可视化检测结果
        
        Args:
            depth_image: 原始深度图
            obstacle_mask: 障碍物掩码
            info: 检测信息
            color_image: 彩色图像（可选）
            
        Returns:
            vis_image: 可视化图像
        """
        # 深度图伪彩色
        depth_normalized = np.zeros_like(depth_image)
        cv2.normalize(depth_image, depth_normalized, 0, 255, cv2.NORM_MINMAX)
        depth_colored = cv2.applyColorMap(
            depth_normalized.astype(np.uint8),
            cv2.COLORMAP_JET
        )
        
        # 叠加障碍物掩码
        obstacle_colored = cv2.cvtColor(obstacle_mask, cv2.COLOR_GRAY2BGR)
        obstacle_colored[:, :, 0] = 0  # 只保留红色通道
        
        vis_image = cv2.addWeighted(depth_colored, 0.7, obstacle_colored, 0.3, 0)
        
        # 绘制可通行区域
        for zone in info['navigable_zones'][:3]:  # 显示前3个最大区域
            cx, cy = zone['centroid']
            cv2.circle(vis_image, (int(cx), int(cy)), 10, (0, 255, 0), -1)
            cv2.putText(vis_image, f"Zone {zone['id']}", 
                       (int(cx) + 15, int(cy)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # 显示方向建议
        direction = info['suggested_direction']
        direction_color = (0, 255, 0) if direction == 'forward' else (0, 255, 255)
        cv2.putText(vis_image, f"Direction: {direction}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, direction_color, 2)
        
        # 显示统计信息
        cv2.putText(vis_image, f"FPS: {1000/info['processing_time']:.1f}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(vis_image, f"Min Depth: {info['min_depth']:.2f}m", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(vis_image, f"Obstacles: {info['obstacle_count']}", 
                   (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        return vis_image


def main():
    """
    主函数 - 测试深度SLAM避障系统
    """
    print("=" * 60)
    print("深度图 SLAM 避障系统")
    print("=" * 60)
    
    # 初始化检测器
    detector = DepthSLAMObstacleDetector(
        depth_threshold_near=0.5,
        depth_threshold_far=5.0,
        obstacle_height_min=0.1,
        grid_resolution=0.05
    )
    
    print("\n[INFO] 检测器初始化完成")
    print(f"  - 近距离阈值: {detector.depth_threshold_near}m")
    print(f"  - 远距离阈值: {detector.depth_threshold_far}m")
    print(f"  - 网格分辨率: {detector.grid_resolution}m")
    
    # TODO: 集成 Berxel 相机
    # from berxel_camera import BerxelCamera
    # camera = BerxelCamera()
    # camera.open()
    
    # 模拟深度图数据用于测试
    print("\n[WARNING] 当前使用模拟数据测试")
    print("[INFO] 按 'q' 退出")
    
    while True:
        # 生成模拟深度图（实际应从相机读取）
        depth_sim = np.random.rand(480, 640) * 3.0 + 0.5
        
        # 添加一些模拟障碍物
        depth_sim[200:280, 250:350] = 0.3  # 近距离障碍物
        depth_sim[100:150, 450:550] = 1.5  # 中距离障碍物
        
        # 处理深度帧
        obstacle_mask, info = detector.process_depth_frame(depth_sim)
        
        # 可视化
        vis_image = detector.visualize(depth_sim, obstacle_mask, info)
        
        # 显示结果
        cv2.imshow('Depth SLAM Obstacle Detection', vis_image)
        cv2.imshow('Obstacle Mask', obstacle_mask)
        
        # 按键处理
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
    
    cv2.destroyAllWindows()
    
    print("\n" + "=" * 60)
    print("统计信息:")
    print(f"  - 总帧数: {detector.frame_count}")
    print(f"  - 平均处理时间: {np.mean(detector.processing_times):.3f}s")
    print(f"  - 平均FPS: {1.0/np.mean(detector.processing_times):.1f}")
    print("=" * 60)


if __name__ == "__main__":
    main()
