#include <Python.h>
#include <numpy/arrayobject.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include "BerxelHawkContext.h"
#include "BerxelHawkDevice.h"
#include "BerxelHawkFrame.h"
#include "BerxelHawkDefines.h"
#include <iostream>

using namespace berxel;

static berxel::BerxelHawkContext* g_context = nullptr;
static berxel::BerxelHawkDevice* g_device = nullptr;

static PyObject* init_camera(PyObject* self, PyObject* args) {
    try {
        // 获取Berxel Context
        g_context = berxel::BerxelHawkContext::getBerxelContext();
        if (!g_context) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to get Berxel context");
            return nullptr;
        }
        
        // 查找设备
        berxel::BerxelHawkDeviceInfo* pDeviceInfo = nullptr;
        uint32_t deviceCount = 0;
        if (g_context->getDeviceList(&pDeviceInfo, &deviceCount) != 0 || deviceCount <= 0 || pDeviceInfo == nullptr) {
            PyErr_SetString(PyExc_RuntimeError, "No Berxel camera found");
            return nullptr;
        }
        
        // 打开第一个设备
        g_device = g_context->openDevice(pDeviceInfo[0]);
        if (!g_device) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to open camera");
            return nullptr;
        }
        
        // 同步系统时间
        g_device->setSystemClock();
        
        // 设置流模式
        g_device->setStreamFlagMode(berxel::BERXEL_HAWK_SINGULAR_STREAM_FLAG_MODE);
        
        // 设置彩色图像模式
        berxel::BerxelHawkStreamFrameMode colorMode;
        g_device->getCurrentFrameMode(berxel::BERXEL_HAWK_COLOR_STREAM, &colorMode);
        colorMode.resolutionX = 1920;
        colorMode.resolutionY = 1080;
        colorMode.framerate = 30;
        if (g_device->setFrameMode(berxel::BERXEL_HAWK_COLOR_STREAM, &colorMode) != 0) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to set color frame mode");
            return nullptr;
        }
        
        // 仅启动彩色图像流
        if (g_device->startStreams(berxel::BERXEL_HAWK_COLOR_STREAM) != 0) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to start streams");
            return nullptr;
        }
        
        Py_RETURN_NONE;
    }
    catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

static PyObject* get_frame(PyObject* self, PyObject* args) {
    try {
        if (!g_device) {
            PyErr_SetString(PyExc_RuntimeError, "Camera not initialized");
            return nullptr;
        }
        
        // 获取一帧图像
        berxel::BerxelHawkFrame* frame = nullptr;
        if (g_device->readColorFrame(frame, -1) != 0 || !frame) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to get frame");
            return nullptr;
        }
        
        // 获取图像数据
        npy_intp dims[3] = {(npy_intp)frame->getHeight(), (npy_intp)frame->getWidth(), 3};
        PyObject* array = PyArray_SimpleNewFromData(3, dims, NPY_UINT8, (void*)frame->getData());
        
        // 释放帧
        g_device->releaseFrame(frame);
        
        return array;
    }
    catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

static PyObject* release_camera(PyObject* self, PyObject* args) {
    try {
        if (g_device) {
            g_device->stopStreams(berxel::BERXEL_HAWK_COLOR_STREAM);
            g_context->closeDevice(g_device);
            g_device = nullptr;
        }
        
        if (g_context) {
            berxel::BerxelHawkContext::destroyBerxelContext(g_context);
            g_context = nullptr;
        }
        
        Py_RETURN_NONE;
    }
    catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

static PyObject* get_depth(PyObject* self, PyObject* args) {
    try {
        if (!g_device) {
            PyErr_SetString(PyExc_RuntimeError, "Camera not initialized");
            return nullptr;
        }
        
        // 获取一帧深度图像
        berxel::BerxelHawkFrame* frame = nullptr;
        if (g_device->readDepthFrame(frame, -1) != 0 || !frame) {
            PyErr_SetString(PyExc_RuntimeError, "Failed to get depth frame");
            return nullptr;
        }
        
        // 获取深度图像数据
        npy_intp dims[2] = {(npy_intp)frame->getHeight(), (npy_intp)frame->getWidth()};
        PyObject* array = PyArray_SimpleNewFromData(2, dims, NPY_UINT16, (void*)frame->getData());
        
        // 释放帧
        g_device->releaseFrame(frame);
        
        return array;
    }
    catch (const std::exception& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return nullptr;
    }
}

static PyMethodDef BerxelMethods[] = {
    {"init_camera", init_camera, METH_NOARGS, "Initialize Berxel camera"},
    {"get_frame", get_frame, METH_NOARGS, "Get a color frame from the camera"},
    {"get_depth", get_depth, METH_NOARGS, "Get a depth frame from the camera"},
    {"release_camera", release_camera, METH_NOARGS, "Release camera resources"},
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef berxel_module = {
    PyModuleDef_HEAD_INIT,
    "berxel_wrapper",
    NULL,
    -1,
    BerxelMethods
};

PyMODINIT_FUNC PyInit_berxel_wrapper(void) {
    import_array();
    return PyModule_Create(&berxel_module);
}