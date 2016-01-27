#pragma once
#include "KinectUdpDataReceiver.h"

class Viewer
{
    typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;
    typedef PointCloud::ConstPtr ConstPtr;

private:
    boost::thread viewThread;
    mutable boost::mutex mutex;

    void ViewThreadFunction();

    pcl::visualization::PCLVisualizer * viewer;
    std::vector<KinectUdpDataReceiver *> kinectUdpDataReceiverList;

    bool quit;
    bool running;

public:
    Viewer();
    virtual ~Viewer();

    void addKinectUdpDataReceiver(KinectUdpDataReceiver* value);
    virtual void start();
    virtual void stop();
    virtual bool isRunning() const;
};

