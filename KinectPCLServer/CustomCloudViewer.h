#pragma once

#include "stdafx.h"

namespace pcl
{
    namespace visualization
    {
        class PCL_EXPORTS CustomCloudViewer : boost::noncopyable
        {
        public:
            typedef PointCloud<PointXYZRGBA> ColorACloud;
            typedef PointCloud<PointXYZRGB> ColorCloud;
            typedef PointCloud<PointXYZI> GrayCloud;
            typedef PointCloud<PointXYZ> MonochromeCloud;

            explicit CustomCloudViewer(const std::string& window_name);
            ~CustomCloudViewer();

            void showCloud(const ColorCloud::ConstPtr      &cloud, const std::string& cloudname = "cloud");
            void showCloud(const ColorACloud::ConstPtr     &cloud, const std::string& cloudname = "cloud");
            void showCloud(const GrayCloud::ConstPtr       &cloud, const std::string& cloudname = "cloud");
            void showCloud(const MonochromeCloud::ConstPtr &cloud, const std::string& cloudname = "cloud");

            bool wasStopped(int millis_to_wait = 1);

            typedef boost::function1<void, PCLVisualizer&> VizCallable;

            void runOnVisualizationThread(VizCallable x, const std::string& key = "callable");
            void runOnVisualizationThreadOnce(VizCallable x);
            void removeVisualizationCallable(const std::string& key = "callable");

            boost::signals2::connection registerKeyboardCallback(void(*callback) (const KeyboardEvent&, void*), void* cookie = nullptr)
            {
                return (registerKeyboardCallback(bind(callback, _1, cookie)));
            }

            template<typename T> boost::signals2::connection registerKeyboardCallback(void (T::*callback) (const KeyboardEvent&, void*), T& instance, void* cookie = nullptr)
            {
                return (registerKeyboardCallback(boost::bind(callback, boost::ref(instance), _1, cookie)));
            }

            boost::signals2::connection
                registerMouseCallback(void(*callback) (const pcl::visualization::MouseEvent&, void*), void* cookie = nullptr)
            {
                    return (registerMouseCallback(boost::bind(callback, _1, cookie)));
                }

            template<typename T> boost::signals2::connection registerMouseCallback(void (T::*callback) (const MouseEvent&, void*), T& instance,
                void* cookie = nullptr)
            {
                return (registerMouseCallback(boost::bind(callback, boost::ref(instance), _1, cookie)));
            }

            boost::signals2::connection registerPointPickingCallback(void(*callback) (const PointPickingEvent&, void*),
                void* cookie = nullptr)
            {
                return (registerPointPickingCallback(boost::bind(callback, _1, cookie)));
            }

            template<typename T> boost::signals2::connection registerPointPickingCallback(void (T::*callback) (const PointPickingEvent&, void*),
                T& instance,
                void* cookie = nullptr)
            {
                return (registerPointPickingCallback(boost::bind(callback, boost::ref(instance), _1, cookie)));
            }

        private:
            struct CustomCloudViewer_impl;
            std::auto_ptr<CustomCloudViewer_impl> impl_;

            boost::signals2::connection registerMouseCallback(boost::function<void(const MouseEvent&)>);
            boost::signals2::connection registerKeyboardCallback(boost::function<void(const KeyboardEvent&)>);
            boost::signals2::connection registerPointPickingCallback(boost::function<void(const PointPickingEvent&)>);
        };
    }
}
