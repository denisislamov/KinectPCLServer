#include "stdafx.h"
#include "KinectUdpDataReceiver.h"

KinectUdpDataReceiver::KinectUdpDataReceiver(int portValue = 8888) :
  result(S_OK),
  sensor(nullptr),
  mapper(nullptr),
  quit   (false),
  running(false),
  calculateNewData(false)
{
    socketLength       = sizeof(otherSocketAdress);
    socketBuffer       = new char[MAX_PACKET_SIZE + 2];
    kinectData         = new KinectData();
    
    port = portValue;
    memset(receivedIndexArray, false, MAX_RECIEVE_INDEX);

    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());

    cloud->width = static_cast<uint32_t>(KinectData::DEPTH_FRAME_WIDTH);
    cloud->height = static_cast<uint32_t>(KinectData::DEPTH_FRAME_HEIGHT);
    cloud->is_dense = false;

    cloud->points.resize(cloud->height * cloud->width);

    std::cout << "Initialising winsock" << std::endl;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        auto errorMsg = std::string("Exception : Failed. Error Code : ") + 
                        boost::lexical_cast<std::string>(WSAStartup);
        throw std::exception(errorMsg.c_str());
    }
    std::cout << "Initialised" << std::endl;

    if ((socketVal = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
    {
        auto errorMsg = std::string("Exception : Could not create socket : ") +
                        boost::lexical_cast<std::string>(WSAGetLastError());
        throw std::exception(errorMsg.c_str());
    }
    std::cout << "Socket created" << std::endl;

    serverSocketAdress.sin_family      = AF_INET;
    serverSocketAdress.sin_addr.s_addr = INADDR_ANY;
    serverSocketAdress.sin_port        = htons(portValue);

    result = GetDefaultKinectSensor(&sensor);
    if (FAILED(result))
    {
        throw std::exception("Exception : GetDefaultKinectSensor()");
    }
    std::cout << "Succeeded get default kinect sensor" << std::endl;

    result = sensor->Open();
    if (FAILED(result))
    {
        throw std::exception("Exception : IKinectSensor::Open()");
    }
    std::cout << "Succeeded open kinect" << std::endl;

    result = sensor->get_CoordinateMapper(&mapper);
    if (FAILED(result))
    {
        throw std::exception("Exception : IKinectSensor::get_CoordinateMapper()");
    }
    std::cout << "Succeeded get coordinate mapper" << std::endl;
}

KinectUdpDataReceiver::~KinectUdpDataReceiver() throw()
{
    stop();
    
    if (sensor)
    {
        sensor->Close();
    }
    SafeRelease(sensor);
    SafeRelease(mapper);

    udpThread.join();
    convertKinectDataThread.join();
}


void KinectUdpDataReceiver::start()
{
    if (bind(socketVal, reinterpret_cast<struct sockaddr *>(&serverSocketAdress), 
        sizeof(serverSocketAdress)) == SOCKET_ERROR)
    {
        auto errorMsg = std::string("Exception : Bind failed with error code : ") +
            boost::lexical_cast<std::string>(WSAGetLastError());
        throw std::exception(errorMsg.c_str());
    }
    std::cout << "Bind done" << std::endl;
    running = true;

    udpThread = boost::thread(&KinectUdpDataReceiver::UdpThreadFunction, this);
    convertKinectDataThread = boost::thread(&KinectUdpDataReceiver::ConvertKinectDataThreadFunction, this);
}

void KinectUdpDataReceiver::stop()
{
    boost::unique_lock<boost::mutex> lock(mutex);

    quit = true;
    running = false;

    lock.unlock();
}

bool KinectUdpDataReceiver::isRunning() const
{
    return running;
}

std::string KinectUdpDataReceiver::getName() const
{
    return std::string("KinectUdpDataReceiver");
}

float KinectUdpDataReceiver::getFramesPerSecond() const
{
    return 30.0f;
}

inline KinectData* KinectUdpDataReceiver::GetKinectData()
{
    return kinectData;
}

inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr KinectUdpDataReceiver::convertKinectDataToPointXYZRGBA()
{
    return {};
}

void KinectUdpDataReceiver::UdpThreadFunction()
{
    std::cout << "Start udp thread function" << std::endl;
    while (!quit)
    {
        if ((receiverLength = recvfrom(socketVal, reinterpret_cast<char *> (socketBuffer), MAX_PACKET_SIZE + 2, 0, reinterpret_cast<struct sockaddr *>(&otherSocketAdress), &socketLength)) == SOCKET_ERROR)
        {
            auto errorMsg = std::string("Exception : recvfrom() failed with error code : ") +
                            boost::lexical_cast<std::string>(WSAGetLastError());
            throw std::exception(errorMsg.c_str());
        }

        if (receiverLength == MAX_PACKET_SIZE + 2)
        {
            if (socketBuffer[0] == socketBuffer[1] - 10)
            {
                if (0 <= socketBuffer[0] && socketBuffer[0] < 8)
                {
                    receivedIndexArray[socketBuffer[0]] = true;
                    memcpy(kinectData->depthBuffer + socketBuffer[0] * MAX_PACKET_SIZE, socketBuffer + 2, MAX_PACKET_SIZE);
                }
                else if (8 < socketBuffer[0] && socketBuffer[0] < 24)
                {
                    receivedIndexArray[socketBuffer[0]] = true;
                    memcpy(kinectData->rgbMapDepthBuffer + (socketBuffer[0] - 8) * MAX_PACKET_SIZE, socketBuffer + 2, MAX_PACKET_SIZE);
                }
            }
        }

        boost::unique_lock<boost::mutex> lock(mutex);
        for (int i = 0; i < MAX_RECIEVE_INDEX; ++i)
        {
            if (receivedIndexArray[i])
            {
                calculateNewData = true;
            }
            else
            {
                calculateNewData = false;
                break;
            }
        }
        lock.unlock();
    }

    closesocket(socketVal);
    WSACleanup();
}

void KinectUdpDataReceiver::ConvertKinectDataThreadFunction()
{
    std::cout << "Start convert kinect data thread function" << std::endl;
    while (!quit)
    {
        if (calculateNewData)
        {
           /* pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());*/

            cloud->width = static_cast<uint32_t>(KinectData::DEPTH_FRAME_WIDTH);
            cloud->height = static_cast<uint32_t>(KinectData::DEPTH_FRAME_HEIGHT);
            cloud->is_dense = false;

            cloud->points.resize(cloud->height * cloud->width);

            pcl::PointXYZRGBA* pt = &cloud->points[0];

            for (int y = 0; y < KinectData::DEPTH_FRAME_HEIGHT; y++)
            {
                for (int x = 0; x < KinectData::DEPTH_FRAME_WIDTH; x++, pt++)
                {
                    pcl::PointXYZRGBA point;

                    DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
                    int index = y * KinectData::DEPTH_FRAME_WIDTH + x;
                    UINT16 depth = kinectData->depthBuffer[y * KinectData::DEPTH_FRAME_WIDTH + x];

                    CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
                    mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);

                    point.x = cameraSpacePoint.X;
                    point.y = cameraSpacePoint.Y;
                    point.z = cameraSpacePoint.Z;

                    point.r = kinectData->rgbMapDepthBuffer[index * 4];
                    point.g = kinectData->rgbMapDepthBuffer[index * 4 + 1];
                    point.b = kinectData->rgbMapDepthBuffer[index * 4 + 2];
                    point.a = kinectData->rgbMapDepthBuffer[index * 4 + 3];

                    *pt = point;
                }
            }

            boost::unique_lock<boost::mutex> lock(mutex);
            memset(receivedIndexArray, false, MAX_RECIEVE_INDEX);
            calculateNewData = false;
            lock.unlock();
        }
    }
}