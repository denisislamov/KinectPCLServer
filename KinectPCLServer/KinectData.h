#pragma once

#include "stdafx.h"

template<class Interface>
inline void SafeRelease(Interface *& IRelease)
{
    if (IRelease != nullptr)
    {
        IRelease->Release();
        IRelease = nullptr;
    }
}

typedef struct KinectData
{
    static const int DEPTH_FRAME_WIDTH  = 512;
    static const int DEPTH_FRAME_HEIGHT = 424;
    static const int DEPTH_FRAME_SIZE = DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT;
    static const int BYTES_PER_PIXEL = 4;

    char * depthBuffer = new char[DEPTH_FRAME_SIZE];
    char * rgbMapDepthBuffer = new char[BYTES_PER_PIXEL * DEPTH_FRAME_SIZE];

    KinectData();
    ~KinectData();

} KinectData;

inline KinectData::KinectData()
{
    depthBuffer       = new char[DEPTH_FRAME_SIZE];
    rgbMapDepthBuffer = new char[BYTES_PER_PIXEL * DEPTH_FRAME_SIZE];
    
    std::cout << "Create Kinect Data" << std::endl;
}

inline KinectData::~KinectData()
{
    if (depthBuffer != nullptr)
    {
        delete[] depthBuffer;
        depthBuffer = nullptr;
    }

    if (rgbMapDepthBuffer != nullptr)
    {
        delete[] rgbMapDepthBuffer;
        rgbMapDepthBuffer = nullptr;
    }
}