// KinectPCLServer.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"

#include "KinectData.h"
#include "CustomCloudViewer.h"
#include "KinectUdpDataReceiver.h"
#include "Viewer.h"

KinectData KinectDataLocalHost;

int _tmain(int argc, _TCHAR* argv[])
{
    auto kinectUdpDataReceiver_8888 = new KinectUdpDataReceiver(8888);
    kinectUdpDataReceiver_8888->start();

    Viewer viewer;
    viewer.addKinectUdpDataReceiver(kinectUdpDataReceiver_8888);

    while (true)
    {
        if (GetKeyState(VK_ESCAPE) < 0)
        {
            break;
        }
    }

    kinectUdpDataReceiver_8888->stop();
    viewer.stop();

    return 0;
}
