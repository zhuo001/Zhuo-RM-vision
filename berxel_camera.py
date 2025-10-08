import berxel_wrapper
import cv2
import numpy as np

class BerxelCamera:
    def __init__(self):
        self.initialized = False
        
    def initialize(self):
        """初始化相机"""
        try:
            berxel_wrapper.init_camera()
            self.initialized = True
            print("Berxel相机初始化成功")
            return True
        except Exception as e:
            print(f"相机初始化失败: {str(e)}")
            return False
            
    def get_frame(self):
        """获取一帧彩色图像"""
        if not self.initialized:
            return None
            
        try:
            frame = berxel_wrapper.get_frame()
            if frame is None:
                return None
            
            # 确保返回BGR格式以兼容OpenCV
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
        except Exception as e:
            print(f"获取彩色图像失败: {str(e)}")
            return None
            
    def get_depth(self):
        """获取一帧深度图像"""
        if not self.initialized:
            return None
            
        try:
            depth = berxel_wrapper.get_depth()
            if depth is None:
                return None
                
            # 返回深度图像（单位：毫米）
            return depth
            
        except Exception as e:
            print(f"获取深度图像失败: {str(e)}")
            return None
            
    def release(self):
        """释放相机资源"""
        if self.initialized:
            try:
                berxel_wrapper.release_camera()
            except Exception as e:
                print(f"释放相机资源失败: {str(e)}")
            finally:
                self.initialized = False
