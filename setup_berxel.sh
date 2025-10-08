#!/bin/bash

# 添加Berxel SDK库到LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/zhuo-skadi/Documents/berxel-sdk-master/libs:$LD_LIBRARY_PATH

# 添加Berxel SDK头文件路径到CPATH
export CPATH=/home/zhuo-skadi/Documents/berxel-sdk-master/Include:$CPATH

# 创建系统链接
echo "正在创建系统链接..."
sudo ln -sf /home/zhuo-skadi/Documents/berxel-sdk-master/libs/libBerxelHawk.so /usr/lib/
sudo ln -sf /home/zhuo-skadi/Documents/berxel-sdk-master/libs/libBerxelInterface.so /usr/lib/
sudo ln -sf /home/zhuo-skadi/Documents/berxel-sdk-master/libs/libBerxelCommonDriver.so /usr/lib/
sudo ln -sf /home/zhuo-skadi/Documents/berxel-sdk-master/libs/libBerxelLogDriver.so /usr/lib/
sudo ln -sf /home/zhuo-skadi/Documents/berxel-sdk-master/libs/libBerxelNetDriver.so /usr/lib/
sudo ln -sf /home/zhuo-skadi/Documents/berxel-sdk-master/libs/libBerxelUvcDriver.so /usr/lib/

# 设置头文件目录
sudo mkdir -p /usr/include/berxel
sudo cp -r /home/zhuo-skadi/Documents/berxel-sdk-master/Include/* /usr/include/berxel/

# 重新加载动态链接器配置
sudo ldconfig

echo "Berxel SDK环境设置完成"