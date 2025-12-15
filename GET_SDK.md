# 获取Berxel P100R SDK库文件

## 当前问题
- 仓库中缺少`libs/BerxelHawk.dll`和`BerxelHawk.lib`
- 无法编译`berxel_wrapper.cpp` Python扩展
- 目前只能通过UVC读取彩色图，无法获取深度图

## 解决步骤

### 1. 从官方获取SDK
访问 Berxel官网下载P100R完整SDK：
- Windows版本需要：`BerxelHawk.dll` + `BerxelHawk.lib`
- 将DLL和LIB文件复制到`Zhuo-RM-vision/libs/`目录

### 2. 或者从其他设备复制
如果你在另一台电脑上安装过SDK：
```bash
# 找到SDK安装目录（通常在C:\Program Files\Berxel或类似路径）
# 复制以下文件到仓库libs/目录：
BerxelHawk.dll
BerxelHawk.lib
```

### 3. 编译Python扩展
```powershell
cd Zhuo-RM-vision
python setup.py build_ext --inplace
```

### 4. 测试
```powershell
python test_p100r_slam.py
```

## 临时替代方案（无SDK）
如果暂时无法获取SDK，可以使用OpenCV UVC读取：
- P100R的深度图可能通过特殊格式传输（需要解码）
- 或者使用立体视觉算法从双目图像计算深度（如果P100R支持）

查看相机支持的所有格式：
```python
import cv2
cap = cv2.VideoCapture(0)
for i in range(20):
    print(f"Format {i}: {cap.get(i)}")
```
