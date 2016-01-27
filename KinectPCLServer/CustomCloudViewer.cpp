#include "stdafx.h"
#include "CustomCloudViewer.h"

namespace pcl
{
    struct cloud_show_base
    {
        virtual ~cloud_show_base() {}

        virtual void pop() = 0;
        virtual bool popped() const = 0;
        typedef boost::shared_ptr<cloud_show_base> Ptr;
    };

    template <typename CloudT>
    struct cloud_show : cloud_show_base
    {
        cloud_show(const std::string &cloud_name, typename CloudT::ConstPtr cloud,
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) :
        cloud_name(cloud_name), cloud(cloud), viewer(viewer), popped_(false)
        {}

        template <typename Handler> void
            pop(const Handler &handler)
        {
                auto psize = 1.0, opacity = 1.0, linesize = 1.0;
                viewer->getPointCloudRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, cloud_name);
                viewer->getPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, opacity, cloud_name);
                viewer->getPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, psize, cloud_name);
                if (!viewer->updatePointCloud(cloud, handler, cloud_name))
                {
                    viewer->addPointCloud(cloud, handler, cloud_name);
                    viewer->resetCameraViewpoint(cloud_name);
                }

                psize = 3.0f;

                // viewer->removePointCloud (cloud_name);
                viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH, linesize, cloud_name);
                viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_OPACITY, opacity, cloud_name);
                viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, psize, cloud_name);
                popped_ = true;
            }

        virtual void pop() override;

        virtual bool popped() const override
        {
            return popped_;
        }

        std::string cloud_name;
        typename CloudT::ConstPtr cloud;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
        bool popped_;
    };

    typedef PointCloud<PointXYZRGBA> cca;
    typedef PointCloud<PointXYZRGB>  cc;
    typedef PointCloud<PointXYZI>    gc;
    typedef PointCloud<PointXYZ>     mc;

    template <> void cloud_show<cca>::pop()
    {
        visualization::PointCloudColorHandlerRGBField<PointXYZRGBA> handler(cloud);
        pop(handler);
    }

    template <> void cloud_show<cc>::pop()
    {
        visualization::PointCloudColorHandlerRGBField<PointXYZRGB> handler(cloud);
        pop(handler);
    }

    template <> void cloud_show<gc>::pop()
    {
        visualization::PointCloudColorHandlerGenericField<PointXYZI> handler(cloud, "intensity");
        pop(handler);
    }

    template <> void cloud_show<mc>::pop()
    {
        visualization::PointCloudGeometryHandlerXYZ<PointXYZ> handler(cloud);
        pop(handler);
    }
}

struct pcl::visualization::CustomCloudViewer::CustomCloudViewer_impl
{
    explicit CustomCloudViewer_impl(const std::string& window_name) : window_name_(window_name), has_cloud_(false), quit_(false)
    {
        viewer_thread_ = boost::thread(boost::ref(*this));
        while (!viewer_)
        {
            boost::thread::yield();
        }
    }

    ~CustomCloudViewer_impl()
    {
    }

    template <typename T> void
        block_post_cloud(const typename T::ConstPtr &cloud, const std::string &name)
    {
            cloud_show_base::Ptr cs(new cloud_show<T>(name, cloud, viewer_));
            {
                boost::mutex::scoped_lock lock(mtx_);
                cloud_shows_.push_back(cs);
            }
            while (!cs->popped())
            {
                boost::thread::yield();
            }
        }

    template <typename T> void nonblock_post_cloud(const typename T::ConstPtr &cloud, const std::string &name)
    {
        cloud_show_base::Ptr cs(new cloud_show<T>(name, cloud, viewer_));
        {
            boost::mutex::scoped_lock lock(mtx_);

            cloud_shows_.push_back(cs);
        }
    }

    void operator() ()
    {
        using namespace pcl::visualization;

#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
        viewer_ = boost::shared_ptr<PCLVisualizer>(new PCLVisualizer(window_name_));
#else
        viewer_ = boost::make_shared<PCLVisualizer>(window_name_, true);
#endif
        viewer_->setBackgroundColor(0.1, 0.1, 0.1);
        viewer_->addCoordinateSystem(0.1, "global");

        while (!quit_)
        {
            {
                boost::mutex::scoped_lock lock(mtx_);
                while (!cloud_shows_.empty())
                {
                    cloud_shows_.back()->pop();
                    cloud_shows_.pop_back();
                }
            }

            {
            boost::mutex::scoped_lock lock(once_mtx);
            BOOST_FOREACH(CallableList::value_type& x, callables_once)
            {
                (x)(*viewer_);
            }
            callables_once.clear();
        }

            {
                boost::mutex::scoped_lock lock(c_mtx);
                BOOST_FOREACH(CallableMap::value_type& x, callables)
                {
                    (x.second)(*viewer_);
                }
            }

            if (viewer_->wasStopped())
            {
                quit_ = true;
            }
            else
            {
                boost::mutex::scoped_lock lock(spin_mtx_);
                viewer_->spinOnce(10);
            }
        }
        viewer_.reset();
    }

    void post(VizCallable x, const std::string &key)
    {
        boost::mutex::scoped_lock lock(c_mtx);
        callables[key] = x;
    }

    void post(VizCallable x)
    {
        boost::mutex::scoped_lock lock(once_mtx);
        callables_once.push_back(x);
    }

    void remove(const std::string &key)
    {
        boost::mutex::scoped_lock lock(c_mtx);
        if (callables.find(key) != callables.end())
        {
            callables.erase(key);
        }
    }

    std::string window_name_;
    boost::shared_ptr<PCLVisualizer> viewer_;
    boost::mutex mtx_, spin_mtx_, c_mtx, once_mtx;
    boost::thread viewer_thread_;
    bool has_cloud_;
    bool quit_;
    std::list<boost::shared_ptr<cloud_show_base> > cloud_shows_;
    typedef std::map<std::string, VizCallable> CallableMap;
    CallableMap callables;
    typedef std::list<VizCallable> CallableList;
    CallableList callables_once;
};

pcl::visualization::CustomCloudViewer::CustomCloudViewer(const std::string &window_name) :
impl_(new CustomCloudViewer_impl(window_name))
{}

pcl::visualization::CustomCloudViewer::~CustomCloudViewer()
{
    impl_->quit_ = true;
    impl_->viewer_thread_.join();
}

void pcl::visualization::CustomCloudViewer::showCloud(const ColorACloud::ConstPtr &cloud,
    const std::string &cloudname)
{
    if (!impl_->viewer_ || impl_->viewer_->wasStopped())
    {
        return;
    }
    impl_->block_post_cloud<ColorACloud>(cloud, cloudname);
}

void pcl::visualization::CustomCloudViewer::showCloud(const ColorCloud::ConstPtr &cloud,
    const std::string &cloudname)
{
    if (!impl_->viewer_ || impl_->viewer_->wasStopped())
    {
        return;
    }
    impl_->block_post_cloud<ColorCloud>(cloud, cloudname);
}

void pcl::visualization::CustomCloudViewer::showCloud(const GrayCloud::ConstPtr &cloud,
    const std::string &cloudname)
{
    if (!impl_->viewer_ || impl_->viewer_->wasStopped())
    {
        return;
    }
    impl_->block_post_cloud<GrayCloud>(cloud, cloudname);
}

void pcl::visualization::CustomCloudViewer::showCloud(const MonochromeCloud::ConstPtr &cloud,
    const std::string &cloudname)
{
    if (!impl_->viewer_ || impl_->viewer_->wasStopped())
    {
        return;
    }
    impl_->block_post_cloud<MonochromeCloud>(cloud, cloudname);
}

void pcl::visualization::CustomCloudViewer::runOnVisualizationThread(VizCallable x, const std::string &key)
{
    impl_->post(x, key);
}

void pcl::visualization::CustomCloudViewer::runOnVisualizationThreadOnce(VizCallable x)
{
    impl_->post(x);
}

void pcl::visualization::CustomCloudViewer::removeVisualizationCallable(const std::string &key)
{
    impl_->remove(key);
}

bool pcl::visualization::CustomCloudViewer::wasStopped(int)
{
    boost::thread::yield();
    return !impl_->viewer_;
}

boost::signals2::connection pcl::visualization::CustomCloudViewer::registerKeyboardCallback(boost::function<void(const KeyboardEvent&)> callback)
{
    return impl_->viewer_->registerKeyboardCallback(callback);
}

boost::signals2::connection pcl::visualization::CustomCloudViewer::registerMouseCallback(boost::function<void(const MouseEvent&)> callback)
{
    return impl_->viewer_->registerMouseCallback(callback);
}

boost::signals2::connection pcl::visualization::CustomCloudViewer::registerPointPickingCallback(boost::function<void(const PointPickingEvent&)> callback)
{
    return (impl_->viewer_->registerPointPickingCallback(callback));
}