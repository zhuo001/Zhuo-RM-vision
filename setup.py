from setuptools import setup, Extension
import numpy as np

berxel_module = Extension(
    'berxel_wrapper',
    sources=['berxel_wrapper.cpp'],
    include_dirs=[
        np.get_include(),
        '/home/zhuo-skadi/Documents/berxel-sdk-master/Include'
    ],
    library_dirs=['/home/zhuo-skadi/Documents/berxel-sdk-master/libs'],
    libraries=['BerxelHawk'],
    extra_compile_args=['-std=c++11']
)

setup(
    name="berxel_wrapper",
    version="1.0",
    ext_modules=[berxel_module]
)