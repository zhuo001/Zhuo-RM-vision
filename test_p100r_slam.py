"""
Berxel P100R 相机测试 + SLAM 避障集成
测试相机连接、深度图获取，并集成到SLAM避障系统

作者: Zhuo
日期: 2025-10-15
"""

from __future__ import annotations

import logging
import sys
import traceback
from contextlib import contextmanager
from dataclasses import dataclass
from enum import Enum, auto
from typing import Generator, Optional, Tuple

import cv2
import numpy as np
from numpy.typing import NDArray

from berxel_camera import BerxelCamera
from depth_slam_obstacle import DepthSLAMObstacleDetector


# ============================================================================
# 配置常量
# ============================================================================

@dataclass(frozen=True)
class SLAMConfig:
    """SLAM 检测器配置参数"""
    depth_threshold_near: int = 500    # 近距离阈值 (mm)
    depth_threshold_far: int = 5000    # 远距离阈值 (mm)
    obstacle_height_min: int = 100     # 最小障碍物高度 (mm)
    grid_resolution: int = 50          # 网格分辨率 (mm)


@dataclass(frozen=True)
class DisplayConfig:
    """显示相关配置"""
    stats_log_interval: int = 10       # 统计信息打印间隔帧数
    preview_wait_ms: int = 30          # 预览模式等待时间 (ms)
    font_scale: float = 0.6            # 字体缩放比例
    font_thickness: int = 2            # 字体粗细
    camera_label: str = "Berxel P100R Camera"


class TestMode(Enum):
    """测试模式枚举"""
    PREVIEW = auto()      # 快速预览
    SLAM_FULL = auto()    # 完整SLAM测试
    CONNECTION = auto()   # 仅连接测试


# ============================================================================
# 日志配置
# ============================================================================

logging.basicConfig(
    level=logging.INFO,
    format='[%(levelname)s] %(message)s'
)
logger = logging.getLogger(__name__)


# ============================================================================
# 辅助函数
# ============================================================================

def print_separator(title: str = "", char: str = "=", width: int = 60) -> None:
    """打印分隔线"""
    if title:
        print(f"\n{char * width}")
        print(title)
        print(char * width)
    else:
        print(char * width)


def depth_to_colormap(depth_frame: NDArray[np.uint16]) -> NDArray[np.uint8]:
    """将深度图转换为伪彩色图像"""
    depth_normalized = np.zeros_like(depth_frame, dtype=np.float32)
    cv2.normalize(
        depth_frame.astype(np.float32), 
        depth_normalized, 
        0, 255, 
        cv2.NORM_MINMAX
    )
    return cv2.applyColorMap(
        depth_normalized.astype(np.uint8),
        cv2.COLORMAP_JET
    )


@contextmanager
def camera_session() -> Generator[Optional[BerxelCamera], None, None]:
    """相机会话上下文管理器"""
    camera = BerxelCamera()
    try:
        if camera.initialize():
            logger.info("相机初始化成功")
            yield camera
        else:
            logger.error("相机初始化失败")
            yield None
    finally:
        camera.release()
        logger.info("相机资源已释放")


# ============================================================================
# 相机测试模块
# ============================================================================

class CameraTester:
    """相机测试类"""
    
    def __init__(self) -> None:
        self._camera: Optional[BerxelCamera] = None
    
    def test_connection(self) -> Optional[BerxelCamera]:
        """测试相机连接并返回相机实例"""
        print_separator("测试 Berxel P100R 相机连接")
        
        camera = BerxelCamera()
        
        # 步骤 1: 初始化
        logger.info("步骤 1/3: 初始化相机...")
        if not camera.initialize():
            self._print_connection_error()
            return None
        logger.info("相机初始化成功！")
        
        # 步骤 2: 彩色图像
        logger.info("步骤 2/3: 测试彩色图像获取...")
        color_frame = camera.get_frame()
        if color_frame is None:
            logger.error("无法获取彩色图像")
            camera.release()
            return None
        logger.info(f"彩色图像尺寸: {color_frame.shape}")
        
        # 步骤 3: 深度图像
        logger.info("步骤 3/3: 测试深度图像获取...")
        depth_frame = camera.get_depth()
        if depth_frame is None:
            logger.error("无法获取深度图像")
            camera.release()
            return None
        
        self._log_depth_stats(depth_frame)
        
        print_separator()
        print("✅ 相机测试通过！")
        print_separator()
        
        self._camera = camera
        return camera
    
    @staticmethod
    def _print_connection_error() -> None:
        """打印连接错误帮助信息"""
        logger.error("相机初始化失败！")
        print("请检查:")
        print("  1. P100R是否正确连接到USB端口")
        print("  2. Berxel驱动是否已安装")
        print("  3. berxel_wrapper扩展是否已编译")
    
    @staticmethod
    def _log_depth_stats(depth_frame: NDArray) -> None:
        """记录深度统计信息"""
        valid_depths = depth_frame[depth_frame > 0]
        if len(valid_depths) > 0:
            logger.info(f"深度图像尺寸: {depth_frame.shape}")
            logger.info(f"深度范围: {np.min(valid_depths):.1f}mm - {np.max(depth_frame):.1f}mm")


# ============================================================================
# SLAM 避障系统
# ============================================================================

class SLAMRunner:
    """SLAM 避障运行器"""
    
    def __init__(
        self, 
        camera: BerxelCamera, 
        config: SLAMConfig = SLAMConfig(),
        display_config: DisplayConfig = DisplayConfig()
    ) -> None:
        self._camera = camera
        self._config = config
        self._display = display_config
        self._detector = self._create_detector()
        self._frame_count = 0
        self._screenshot_count = 0
    
    def _create_detector(self) -> DepthSLAMObstacleDetector:
        """创建 SLAM 检测器"""
        return DepthSLAMObstacleDetector(
            depth_threshold_near=self._config.depth_threshold_near,
            depth_threshold_far=self._config.depth_threshold_far,
            obstacle_height_min=self._config.obstacle_height_min,
            grid_resolution=self._config.grid_resolution
        )
    
    def run(self) -> None:
        """运行 SLAM 避障检测循环"""
        self._print_startup_info()
        
        try:
            self._main_loop()
        except KeyboardInterrupt:
            logger.info("用户中断 (Ctrl+C)")
        except Exception as e:
            logger.error(f"运行时错误: {e}")
            traceback.print_exc()
        finally:
            cv2.destroyAllWindows()
            self._print_statistics()
    
    def _print_startup_info(self) -> None:
        """打印启动信息"""
        print_separator("启动 SLAM 避障系统 (使用 P100R 相机)")
        
        logger.info("SLAM检测器初始化完成")
        print(f"  - 近距离阈值: {self._config.depth_threshold_near}mm")
        print(f"  - 远距离阈值: {self._config.depth_threshold_far}mm")
        print(f"  - 网格分辨率: {self._config.grid_resolution}mm")
        print("\n[INFO] 按 'q' 退出, 's' 截图保存")
        print_separator()
    
    def _main_loop(self) -> None:
        """主处理循环"""
        while True:
            # 获取相机数据
            color_frame, depth_frame = self._get_frames()
            if color_frame is None or depth_frame is None:
                logger.warning("无法获取相机数据，跳过此帧")
                continue
            
            # 处理帧
            vis_image, obstacle_mask, info = self._process_frame(
                color_frame, depth_frame
            )
            
            # 显示结果
            self._display_results(vis_image, color_frame, obstacle_mask)
            
            # 更新统计
            self._update_stats(info)
            
            # 处理用户输入
            if not self._handle_input(vis_image):
                break
    
    def _get_frames(self) -> Tuple[Optional[NDArray], Optional[NDArray]]:
        """获取相机帧"""
        return self._camera.get_frame(), self._camera.get_depth()
    
    def _process_frame(
        self, 
        color_frame: NDArray, 
        depth_frame: NDArray
    ) -> Tuple[NDArray, NDArray, dict]:
        """处理单帧数据"""
        # 深度单位转换: mm -> m
        depth_in_meters = depth_frame.astype(np.float32) / 1000.0
        
        # SLAM 处理
        obstacle_mask, info = self._detector.process_depth_frame(
            depth_in_meters, color_frame
        )
        
        # 可视化
        vis_image = self._detector.visualize(
            depth_in_meters, obstacle_mask, info, color_frame
        )
        
        # 添加相机标签
        self._add_camera_label(vis_image)
        
        return vis_image, obstacle_mask, info
    
    def _add_camera_label(self, image: NDArray) -> None:
        """在图像上添加相机标签"""
        cv2.putText(
            image,
            self._display.camera_label,
            (10, image.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            self._display.font_scale,
            (0, 255, 255),
            self._display.font_thickness
        )
    
    def _display_results(
        self, 
        vis_image: NDArray, 
        color_frame: NDArray, 
        obstacle_mask: NDArray
    ) -> None:
        """显示处理结果"""
        cv2.imshow('SLAM Obstacle Detection (P100R)', vis_image)
        cv2.imshow('Color Frame', color_frame)
        cv2.imshow('Obstacle Mask', obstacle_mask)
    
    def _update_stats(self, info: dict) -> None:
        """更新并打印统计信息"""
        self._frame_count += 1
        
        if self._frame_count % self._display.stats_log_interval == 0:
            fps = 1000 / info['processing_time'] if info['processing_time'] > 0 else 0
            print(
                f"[Frame {self._frame_count}] "
                f"FPS: {fps:.1f} | "
                f"Direction: {info['suggested_direction']} | "
                f"Depth: {info['min_depth']:.2f}m-{info['max_depth']:.2f}m | "
                f"Obstacles: {info['obstacle_count']}"
            )
    
    def _handle_input(self, vis_image: NDArray) -> bool:
        """处理用户输入，返回是否继续运行"""
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            logger.info("用户退出")
            return False
        
        if key == ord('s'):
            self._save_screenshot(vis_image)
        
        return True
    
    def _save_screenshot(self, image: NDArray) -> None:
        """保存截图"""
        self._screenshot_count += 1
        filename = f"slam_screenshot_{self._screenshot_count}.png"
        cv2.imwrite(filename, image)
        logger.info(f"截图已保存: {filename}")
    
    def _print_statistics(self) -> None:
        """打印运行统计信息"""
        print_separator("运行统计")
        print(f"  - 总帧数: {self._detector.frame_count}")
        
        if len(self._detector.processing_times) > 0:
            avg_time = np.mean(self._detector.processing_times)
            print(f"  - 平均处理时间: {avg_time:.3f}s")
            print(f"  - 平均FPS: {1.0 / avg_time:.1f}")
        
        print_separator()


# ============================================================================
# 预览模式
# ============================================================================

class CameraPreview:
    """相机预览类"""
    
    def __init__(self, config: DisplayConfig = DisplayConfig()) -> None:
        self._config = config
    
    def run(self) -> None:
        """运行预览模式"""
        print_separator("Berxel P100R 快速预览模式")
        
        with camera_session() as camera:
            if camera is None:
                return
            
            logger.info("显示彩色和深度图像")
            logger.info("按 'q' 退出")
            
            self._preview_loop(camera)
    
    def _preview_loop(self, camera: BerxelCamera) -> None:
        """预览主循环"""
        try:
            while True:
                color_frame = camera.get_frame()
                depth_frame = camera.get_depth()
                
                if color_frame is not None:
                    cv2.imshow('P100R - Color', color_frame)
                
                if depth_frame is not None:
                    depth_colored = depth_to_colormap(depth_frame)
                    cv2.imshow('P100R - Depth', depth_colored)
                
                if cv2.waitKey(self._config.preview_wait_ms) & 0xFF == ord('q'):
                    break
        finally:
            cv2.destroyAllWindows()


# ============================================================================
# 主程序
# ============================================================================

def get_user_choice() -> TestMode:
    """获取用户选择的测试模式"""
    print("\n请选择测试模式:")
    print("  1 - 快速预览相机画面")
    print("  2 - 完整SLAM避障测试")
    print("  3 - 仅测试相机连接")
    
    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        choice = input("\n请输入选项 (1/2/3) [默认: 2]: ").strip() or "2"
    
    mode_map = {
        "1": TestMode.PREVIEW,
        "2": TestMode.SLAM_FULL,
        "3": TestMode.CONNECTION,
    }
    
    return mode_map.get(choice, TestMode.SLAM_FULL)


def main() -> None:
    """主函数"""
    print_separator("Berxel P100R 相机测试与 SLAM 避障集成")
    
    mode = get_user_choice()
    
    if mode == TestMode.PREVIEW:
        CameraPreview().run()
    
    elif mode == TestMode.SLAM_FULL:
        tester = CameraTester()
        camera = tester.test_connection()
        if camera is not None:
            input("\n按 Enter 键开始 SLAM 避障测试...")
            SLAMRunner(camera).run()
            camera.release()
    
    elif mode == TestMode.CONNECTION:
        tester = CameraTester()
        camera = tester.test_connection()
        if camera is not None:
            camera.release()
    
    print("\n程序结束\n")


if __name__ == "__main__":
    main()
