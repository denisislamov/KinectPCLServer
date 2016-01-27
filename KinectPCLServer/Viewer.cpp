#include "stdafx.h"
#include "Viewer.h"

Viewer::Viewer()
{
    viewer = new  pcl::visualization::PCLVisualizer("3D Viewer");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
}

Viewer::~Viewer()
{
    stop();

    viewThread.join();
}

void Viewer::addKinectUdpDataReceiver(KinectUdpDataReceiver* value)
{
    boost::mutex::scoped_lock lock(mutex);
    kinectUdpDataReceiverList.push_back(value);
}

void Viewer::ViewThreadFunction()
{
    while (running)
    {
        viewer->spinOnce(30);
        for (auto kinectUdpDataReceiver : kinectUdpDataReceiverList)
        {
            ConstPtr constPtrCloud(kinectUdpDataReceiver->cloud);
            viewer->addPointCloud(constPtrCloud, "cloud", 0);
        }
    }
}

void Viewer::start()
{
    viewThread = boost::thread(&Viewer::ViewThreadFunction, this);
}

void Viewer::stop()
{
    boost::unique_lock<boost::mutex> lock(mutex);

    quit = true;
    running = false;

    lock.unlock();
}

bool Viewer::isRunning() const
{
    return running;
}