#pragma once

#include "stdafx.h"

struct KinectData;

class KinectUdpDataReceiver 
{
private:
    SOCKET socketVal;
    struct sockaddr_in serverSocketAdress;
    struct sockaddr_in otherSocketAdress;
    int socketLength;
    int receiverLength;

    WSADATA wsa;
    int port;

    char * socketBuffer;
    KinectData * kinectData;
    
    HRESULT result;
    IKinectSensor * sensor;
    ICoordinateMapper * mapper;

    static const int MAX_RECIEVE_INDEX = 24;
    bool receivedIndexArray[MAX_RECIEVE_INDEX];

    boost::thread udpThread;
    boost::thread convertKinectDataThread;
    mutable boost::mutex mutex;

    void UdpThreadFunction();
    void ConvertKinectDataThreadFunction();

    bool quit;
    bool running;
    bool calculateNewData;

public:
    static const int MAX_PACKET_SIZE = 54272;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

    KinectUdpDataReceiver(int portValue);
    virtual ~KinectUdpDataReceiver() throw ();;
    
    virtual void start();
    virtual void stop();
    virtual bool isRunning() const;
    
    virtual std::string getName() const;
    virtual float getFramesPerSecond() const;
    
    virtual KinectData* GetKinectData();

protected:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertKinectDataToPointXYZRGBA();
};

