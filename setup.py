from setuptools import setup, Extension
import numpy as np
import os
import sys

# 检测操作系统
is_windows = sys.platform.startswith('win')

# 获取当前目录
current_dir = os.path.dirname(os.path.abspath(__file__))

# 配置编译参数
if is_windows:
    include_dirs = [
        np.get_include(),
        os.path.join(current_dir, 'Include')
    ]
    library_dirs = [os.path.join(current_dir, 'libs')]
    libraries = ['BerxelHawk']
    extra_compile_args = []
else:
    include_dirs = [
        np.get_include(),
        os.path.join(current_dir, 'Include')
    ]
    library_dirs = [os.path.join(current_dir, 'libs')]
    libraries = ['BerxelHawk']
    extra_compile_args = ['-std=c++11']

berxel_module = Extension(
    'berxel_wrapper',
    sources=['berxel_wrapper.cpp'],
    include_dirs=include_dirs,
    library_dirs=library_dirs,
    libraries=libraries,
    extra_compile_args=extra_compile_args
)

setup(
    name="berxel_wrapper",
    version="1.0",
    ext_modules=[berxel_module]
)