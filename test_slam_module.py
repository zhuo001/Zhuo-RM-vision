#!/usr/bin/env python3
"""
SLAM模块测试脚本
测试depth_slam_obstacle.py模块的基本功能

作者: Zhuo RM Team  
日期: 2025-10-17
"""

import numpy as np
import cv2
import sys

try:
    from depth_slam_obstacle import DepthSLAMObstacleDetector
    print("✅ SLAM模块导入成功")
except ImportError as e:
    print(f"❌ 导入失败: {e}")
    sys.exit(1)


def test_slam_with_simulated_data():
    """使用模拟数据测试SLAM"""
    print("\n" + "="*60)
    print("🧪 SLAM模块功能测试（模拟数据）")
    print("="*60)
    
    # 初始化SLAM检测器
    slam = DepthSLAMObstacleDetector(
        depth_threshold_near=0.8,
        depth_threshold_far=5.0,
        min_navigable_area=500
    )
    print("\n✅ SLAM检测器初始化成功")
    
    # 生成模拟深度图
    print("\n生成模拟深度图...")
    depth = np.random.rand(400, 640) * 3.0 + 1.0  # 1-4米随机深度
    
    # 添加模拟障碍物
    depth[100:200, 200:300] = 0.5   # 近距离障碍物
    depth[50:150, 450:550] = 0.6    # 另一个障碍物
    depth[250:350, 100:200] = 0.7   # 第三个障碍物
    
    print(f"深度图形状: {depth.shape}")
    print(f"深度范围: {depth.min():.2f}m - {depth.max():.2f}m")
    
    # 处理深度帧
    print("\n处理深度帧...")
    obstacle_mask, info = slam.process_depth_frame(depth)
    
    print("\n📊 处理结果:")
    print(f"  - 建议方向: {info['suggested_direction']}")
    print(f"  - 障碍物数量: {info['obstacle_count']}")
    print(f"  - 可导航区域: {len(info['navigable_zones'])}")
    print(f"  - 最近障碍物: {info['min_depth']:.2f}m")
    print(f"  - 处理时间: {info['processing_time']*1000:.1f}ms")
    
    # 显示前3个可导航区域
    if len(info['navigable_zones']) > 0:
        print("\n🎯 可导航区域(前3个):")
        for i, zone in enumerate(info['navigable_zones'][:3]):
            print(f"  区域{i+1}:")
            print(f"    - 中心: ({zone['centroid'][0]:.0f}, {zone['centroid'][1]:.0f})")
            print(f"    - 面积: {zone['area']}")
            print(f"    - 评分: {zone['score']:.3f}")
    
    # 可视化
    print("\n生成可视化...")
    vis = slam.visualize(depth, obstacle_mask, info)
    
    # 显示图像
    print("\n显示结果（按任意键关闭）...")
    cv2.imshow('SLAM Test - Simulated Data', vis)
    cv2.imshow('Obstacle Mask', obstacle_mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # 获取统计信息
    stats = slam.get_statistics()
    print("\n📈 统计信息:")
    print(f"  - 总帧数: {stats['total_frames']}")
    print(f"  - 平均处理时间: {stats['avg_processing_time']*1000:.1f}ms")
    print(f"  - 平均FPS: {stats['avg_fps']:.1f}")
    
    print("\n" + "="*60)
    print("✅ 测试完成！")
    print("="*60)
    
    return True


def test_slam_stress():
    """压力测试：连续处理多帧"""
    print("\n" + "="*60)
    print("⚡ SLAM模块压力测试（100帧）")
    print("="*60)
    
    slam = DepthSLAMObstacleDetector()
    
    print("\n处理100帧模拟数据...")
    for i in range(100):
        depth = np.random.rand(400, 640) * 3.0 + 1.0
        obstacle_mask, info = slam.process_depth_frame(depth)
        
        if (i + 1) % 20 == 0:
            print(f"  已处理 {i+1}/100 帧 - "
                  f"方向: {info['suggested_direction']:8s} - "
                  f"FPS: {1.0/info['processing_time']:.1f}")
    
    stats = slam.get_statistics()
    print("\n📊 压力测试结果:")
    print(f"  - 总帧数: {stats['total_frames']}")
    print(f"  - 平均处理时间: {stats['avg_processing_time']*1000:.1f}ms")
    print(f"  - 平均FPS: {stats['avg_fps']:.1f}")
    print(f"  - 吞吐量: {stats['total_frames']:.0f} 帧")
    
    print("\n✅ 压力测试完成！")
    print("="*60)
    
    return True


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="SLAM模块测试")
    parser.add_argument('--mode', choices=['basic', 'stress', 'all'], 
                       default='basic', help='测试模式')
    
    args = parser.parse_args()
    
    try:
        if args.mode == 'basic' or args.mode == 'all':
            test_slam_with_simulated_data()
        
        if args.mode == 'stress' or args.mode == 'all':
            test_slam_stress()
        
        print("\n🎉 所有测试通过！")
        sys.exit(0)
        
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
