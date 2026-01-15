#ifndef MCVIOFRONTEND_H
#define MCVIOFRONTEND_H

#include <mutex>
#include <queue>
#include <unordered_map>
#include <string>
#include <limits>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include "Multicam_VINS_frontend_data.h"
#include "feature_tracker/trackerbase.h"
#include "feature_tracker/image_processing.h"

#include "sensors.h"
#include "../utility/CameraPoseVisualization.h"


#define SINGLEDEPTH 0
#define USE_LIDAT_DEPTH 1
#define fuse_global_point 1
#define MERGELASER 0

using namespace std;
namespace MCVIO
{
    typedef std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::shared_ptr<MCVIO::SyncCameraProcessingResults>>>
        imuAndFrontEndMeasurement;

    class MCVIOfrontend
    {

    public:
        explicit MCVIOfrontend(string config_file);
        static constexpr int WINDOW_SIZE = 10;

    public:
        void setUpROS(ros::NodeHandle *pub_node, ros::NodeHandle *private_node);
        void addSensors(cv::FileStorage &fsSettings, ros::NodeHandle *private_node);
        void addMonocular(cv::FileNode &fsSettings, ros::NodeHandle *private_node);
        void processImage(const sensor_msgs::ImageConstPtr &color_msg);

        template <typename T>
        sensor_msgs::PointCloud2
        publishCloud(ros::Publisher *thisPub, T thisCloud, ros::Time thisStamp, std::string thisFrame);

    public:
        ros::NodeHandle *pub_node_;
        ros::NodeHandle *private_node_;

        ros::Publisher pub_img, pub_match;
        ros::Publisher pub_restart;

    public:
        // data interface
        std::mutex *datamuex_ = nullptr;
        std::mutex lidar_mutex, lidar_aligned_image_mutex;
        std::queue<MCVIO::FrontEndResult::Ptr> *fontend_output_queue = nullptr;

    public:
        // tracker
        std::vector<double> tracking_times;
        //
        vector<std::shared_ptr<TrackerBase>> trackerData;

        int pub_count = 1;

        bool first_image_flag = true;
        double first_image_time;
        double last_image_time = 0;
        bool init_pub = false;

        int SHOW_TRACK = 1;
        string config_file;
        
        vector<std::shared_ptr<MCVIOsensor>> sensors;  // SENSOR PTR

        //<frame_id, <index in sensors, index in tracker_data>>, tracker data index also indexes in backend for quick access for data
        unordered_map<string, int> sensors_tag;       // SENSORS TAG 
        unordered_map<string, int> tracker_tag;       // TRACKER TAG

        FrontEndResultsSynchronizer synchronizer;
    };
    inline void
    img_callback(const sensor_msgs::Image::ConstPtr color_msg, MCVIOfrontend *frontend)
    {
        frontend->processImage(color_msg);
    };
    inline MCVIOfrontend *MCVIOfrontend_;
} 
#endif 
