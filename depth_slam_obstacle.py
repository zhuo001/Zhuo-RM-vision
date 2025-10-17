"""
深度SLAM障碍物检测模块
基于深度图的实时障碍物检测与导航区域分析

作者: Zhuo RM Team
日期: 2025-10-17
"""

import cv2
import numpy as np
from scipy import ndimage
import time


class DepthSLAMObstacleDetector:
    """
    深度SLAM障碍物检测器
    
    功能:
    - 实时深度图处理
    - 障碍物检测与分割
    - 可导航区域分析
    - 方向建议生成
    """
    
    def __init__(self, 
                 depth_threshold_near=0.5,
                 depth_threshold_far=5.0,
                 obstacle_height_min=0.1,
                 grid_resolution=0.05,
                 min_navigable_area=1000):
        """
        初始化SLAM检测器
        
        Args:
            depth_threshold_near: 近距离阈值(米)，小于此值视为障碍物
            depth_threshold_far: 远距离阈值(米)，大于此值忽略
            obstacle_height_min: 最小障碍物高度(米)
            grid_resolution: 占据栅格分辨率(米)
            min_navigable_area: 最小可导航区域面积(像素)
        """
        self.depth_threshold_near = depth_threshold_near
        self.depth_threshold_far = depth_threshold_far
        self.obstacle_height_min = obstacle_height_min
        self.grid_resolution = grid_resolution
        self.min_navigable_area = min_navigable_area
        
        # 统计信息
        self.frame_count = 0
        self.processing_times = []
        
        # 可视化颜色配置
        self.color_obstacle = (0, 0, 255)    # 红色：障碍物
        self.color_navigable = (0, 255, 0)   # 绿色：可导航
        self.color_unknown = (128, 128, 128) # 灰色：未知
        
    def process_depth_frame(self, depth_meters, color_frame=None):
        """
        处理单帧深度图
        
        Args:
            depth_meters: 深度图(米) shape=(H, W)
            color_frame: 可选的彩色图，用于可视化
            
        Returns:
            obstacle_mask: 障碍物掩码 (uint8)
            info: 决策信息字典
        """
        start_time = time.time()
        
        # 1. 预处理深度图
        depth_processed = self._preprocess_depth(depth_meters)
        
        # 2. 检测障碍物
        obstacle_mask = self._detect_obstacles(depth_processed)
        
        # 3. 分析可导航区域
        navigable_zones = self._analyze_navigable_zones(obstacle_mask)
        
        # 4. 生成导航建议
        direction = self._suggest_direction(navigable_zones, obstacle_mask)
        
        # 5. 计算统计信息
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        self.frame_count += 1
        
        # 组装返回信息
        info = {
            'suggested_direction': direction,
            'navigable_zones': navigable_zones,
            'obstacle_count': np.sum(obstacle_mask > 0),
            'min_depth': np.min(depth_processed[depth_processed > 0]) if np.any(depth_processed > 0) else 0,
            'processing_time': processing_time,
            'frame_count': self.frame_count
        }
        
        return obstacle_mask, info
    
    def _preprocess_depth(self, depth):
        """预处理深度图：去噪、填充"""
        # 复制避免修改原始数据
        depth_clean = depth.copy()
        
        # 过滤无效值
        depth_clean[depth_clean <= 0] = 0
        depth_clean[depth_clean > self.depth_threshold_far] = 0
        
        # 中值滤波去噪
        depth_clean = cv2.medianBlur(depth_clean.astype(np.float32), 5)
        
        return depth_clean
    
    def _detect_obstacles(self, depth):
        """检测障碍物区域"""
        h, w = depth.shape
        obstacle_mask = np.zeros((h, w), dtype=np.uint8)
        
        # 近距离障碍物检测
        near_obstacles = (depth > 0) & (depth < self.depth_threshold_near)
        obstacle_mask[near_obstacles] = 255
        
        # 形态学处理：闭运算连接邻近障碍物
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)
        
        # 去除小噪点
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
        
        return obstacle_mask
    
    def _analyze_navigable_zones(self, obstacle_mask):
        """分析可导航区域"""
        h, w = obstacle_mask.shape
        
        # 反转掩码：0=障碍物，255=可导航
        navigable_mask = cv2.bitwise_not(obstacle_mask)
        
        # 连通域分析
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            navigable_mask, connectivity=8
        )
        
        zones = []
        for i in range(1, num_labels):  # 跳过背景(label=0)
            area = stats[i, cv2.CC_STAT_AREA]
            
            # 过滤小区域
            if area < self.min_navigable_area:
                continue
            
            cx, cy = centroids[i]
            left = stats[i, cv2.CC_STAT_LEFT]
            top = stats[i, cv2.CC_STAT_TOP]
            width = stats[i, cv2.CC_STAT_WIDTH]
            height = stats[i, cv2.CC_STAT_HEIGHT]
            
            # 计算导航评分（考虑面积、位置、宽度）
            # 偏好前方、宽阔的区域
            position_score = 1.0 - abs(cx - w/2) / (w/2)  # 中心位置得分高
            width_score = min(width / w, 1.0)             # 宽度得分
            area_score = min(area / (w * h * 0.3), 1.0)  # 面积得分
            
            score = (position_score * 0.4 + width_score * 0.3 + area_score * 0.3)
            
            zones.append({
                'centroid': (cx, cy),
                'area': area,
                'bbox': (left, top, width, height),
                'score': score
            })
        
        # 按评分排序
        zones.sort(key=lambda x: x['score'], reverse=True)
        
        return zones
    
    def _suggest_direction(self, zones, obstacle_mask):
        """根据可导航区域建议移动方向"""
        h, w = obstacle_mask.shape
        
        if len(zones) == 0:
            return 'stop'  # 无可导航区域
        
        # 获取最佳可导航区域
        best_zone = zones[0]
        cx, cy = best_zone['centroid']
        
        # 检查前方是否有障碍物
        front_region = obstacle_mask[h//3:2*h//3, w//3:2*w//3]
        front_obstacle_ratio = np.sum(front_region > 0) / front_region.size
        
        # 决策逻辑
        if front_obstacle_ratio > 0.3:
            # 前方障碍物较多，根据最佳区域位置决定转向
            if cx < w * 0.4:
                return 'left'
            elif cx > w * 0.6:
                return 'right'
            else:
                return 'stop'
        else:
            # 前方相对畅通
            if cx < w * 0.35:
                return 'left'
            elif cx > w * 0.65:
                return 'right'
            else:
                return 'forward'
    
    def visualize(self, depth, obstacle_mask, info, color_frame=None):
        """
        可视化SLAM结果
        
        Args:
            depth: 原始深度图(米)
            obstacle_mask: 障碍物掩码
            info: 决策信息
            color_frame: 可选彩色图
            
        Returns:
            vis_image: 可视化图像
        """
        h, w = depth.shape
        
        # 创建可视化画布
        if color_frame is not None:
            vis = color_frame.copy()
        else:
            # 深度图伪彩色
            depth_norm = np.clip(depth / self.depth_threshold_far, 0, 1)
            depth_vis = cv2.applyColorMap(
                (depth_norm * 255).astype(np.uint8),
                cv2.COLORMAP_JET
            )
            vis = depth_vis
        
        # 叠加障碍物掩码
        obstacle_overlay = np.zeros_like(vis)
        obstacle_overlay[obstacle_mask > 0] = self.color_obstacle
        vis = cv2.addWeighted(vis, 0.7, obstacle_overlay, 0.3, 0)
        
        # 绘制可导航区域
        for i, zone in enumerate(info['navigable_zones'][:3]):  # 最多显示3个
            cx, cy = int(zone['centroid'][0]), int(zone['centroid'][1])
            left, top, width, height = zone['bbox']
            
            # 绘制边界框
            color = self.color_navigable
            cv2.rectangle(vis, (left, top), (left+width, top+height), color, 2)
            
            # 绘制中心点
            cv2.circle(vis, (cx, cy), 8, color, -1)
            
            # 显示评分
            score_text = f"{zone['score']:.2f}"
            cv2.putText(vis, score_text, (cx-20, cy-15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # 添加信息文字
        direction = info['suggested_direction']
        cv2.putText(vis, f"Direction: {direction.upper()}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.putText(vis, f"FPS: {1.0/info['processing_time']:.1f}", (10, 65),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(vis, f"Obstacles: {info['obstacle_count']}", (10, 95),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 绘制方向指示箭头
        self._draw_direction_arrow(vis, direction)
        
        return vis
    
    def _draw_direction_arrow(self, image, direction):
        """在图像上绘制方向指示箭头"""
        h, w = image.shape[:2]
        center = (w // 2, h - 80)
        arrow_length = 60
        
        color = (0, 255, 255)
        thickness = 3
        
        if direction == 'forward':
            end_point = (center[0], center[1] - arrow_length)
        elif direction == 'left':
            end_point = (center[0] - arrow_length, center[1])
        elif direction == 'right':
            end_point = (center[0] + arrow_length, center[1])
        else:  # stop
            # 绘制停止标志（圆圈+叉）
            cv2.circle(image, center, 30, (0, 0, 255), 3)
            cv2.line(image, (center[0]-20, center[1]-20), 
                    (center[0]+20, center[1]+20), (0, 0, 255), 3)
            cv2.line(image, (center[0]+20, center[1]-20), 
                    (center[0]-20, center[1]+20), (0, 0, 255), 3)
            return
        
        cv2.arrowedLine(image, center, end_point, color, thickness, tipLength=0.3)
    
    def get_statistics(self):
        """获取处理统计信息"""
        if len(self.processing_times) == 0:
            return {
                'total_frames': 0,
                'avg_processing_time': 0,
                'avg_fps': 0
            }
        
        return {
            'total_frames': self.frame_count,
            'avg_processing_time': np.mean(self.processing_times),
            'avg_fps': 1.0 / np.mean(self.processing_times)
        }


if __name__ == "__main__":
    print("深度SLAM障碍物检测模块")
    print("使用方法: 导入DepthSLAMObstacleDetector类")
    print("示例:")
    print("  slam = DepthSLAMObstacleDetector()")
    print("  obstacle_mask, info = slam.process_depth_frame(depth_meters)")
    print("  vis = slam.visualize(depth_meters, obstacle_mask, info)")
