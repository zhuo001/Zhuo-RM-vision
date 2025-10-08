import cv2
import numpy as np
import os
import sys
import time

# 添加 Berxel SDK 路径
berxel_lib_path = "/home/zhuo-skadi/Documents/ros2-robt/libs"
sys.path.append(berxel_lib_path)

try:
    import berxel
except ImportError as e:
    print(f"错误：无法导入 berxel SDK: {e}")
    print(f"请确认 SDK 路径是否正确: {berxel_lib_path}")
    sys.exit(1)

def test_berxel_camera():
    try:
        # 初始化设备
        print("正在初始化 Berxel 相机...")
        device = berxel.Device()
        device.initialize()
        print("相机初始化成功！")

        # 获取相机信息
        print("\n相机信息：")
        print(f"序列号: {device.get_serial_number()}")
        print(f"固件版本: {device.get_firmware_version()}")

        # 启动数据流
        print("\n启动数据流...")
        device.start_streams()
        print("数据流启动成功！")

        # 显示图像
        while True:
            # 获取彩色图像
            color_frame = device.get_color_frame()
            if color_frame is not None:
                color_image = np.array(color_frame.get_data())
                cv2.imshow('Berxel Color Stream', color_image)

            # 获取深度图像
            depth_frame = device.get_depth_frame()
            if depth_frame is not None:
                depth_image = np.array(depth_frame.get_data())
                # 归一化深度图像以便显示
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03),
                    cv2.COLORMAP_JET
                )
                cv2.imshow('Berxel Depth Stream', depth_colormap)

            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n停止预览...")
                break

    except Exception as e:
        print(f"\n错误：{e}")

    finally:
        # 清理资源
        print("清理资源...")
        if 'device' in locals():
            device.stop_streams()
            device.deinitialize()
        cv2.destroyAllWindows()
        print("测试完成！")

if __name__ == "__main__":
    print("=== Berxel 相机测试程序 ===")
    test_berxel_camera()