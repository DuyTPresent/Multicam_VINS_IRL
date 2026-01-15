#pragma once

#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

// CAMERA MODELS
#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "../../utility/tic_toc.h"
#include "../../utility/utility.h"
#include "../../utility/Twist.h"
#include "../sensors.h"
#include "trackerbase.h"

#include "ORBextractor/ORBextractor.h"


#define USE_ORB_SLAM2_DETECTOR 1
using namespace std;
using namespace camodocal;
using namespace Eigen;

namespace MCVIO
{
    struct CameraFrameStatus {
        int frame_id;
        double timestamp;
        int num_tracked_features;
        int num_total_features;
        int num_rejected_by_error;
        int num_rejected_by_descriptor;
        bool has_motion;
        double average_optical_flow_error;
        
        CameraFrameStatus() 
            : frame_id(-1), timestamp(0.0),
            num_tracked_features(0), num_total_features(0),
            num_rejected_by_error(0), num_rejected_by_descriptor(0),
            has_motion(false), average_optical_flow_error(0.0)
        {}
    };


    class ORBFeatureTracker : public TrackerBase
    {
    public:
        ORBFeatureTracker();

        void readImage(const cv::Mat &_img, double _cur_time); // ok

        void setMask(); // ok

        void quadtree_filter(int weight, int height); // ok

        bool updateID(unsigned int i);                          // trong ham front_end
        void readIntrinsicParameter(const string &calib_file);  // trong ham front_end
        void showUndistortion(const string &name);              // trong ham front_end
        void getPt(int idx, int &id, geometry_msgs::Point32 &p, cv::Point2f &p_uv, cv::Point2f &v); // trong ham front_end
        void getCurPt(int idx, cv::Point2f &cur_pt);            // trong ham front_end 

        void rejectRANSAC(); // ok

        void undistortedPoints(); // ok

        bool inBorder(const cv::Point2f &pt);        // ok
        void reduceVector_feature(vector<cv::KeyPoint> &v, vector<uchar> &status); //ok
        void reduceVector_Point2f(vector<cv::Point2f> &v, vector<uchar> status);
        void reduceVector_int(vector<int> &v, vector<uchar> status); //ok
        void reduce_Mat(cv::Mat &mat, vector<uchar> &status); //ok


        double getAdaptiveWeight(int feature_id, int track_cnt);
        double calculateFeatureQuality(int feature_id,int track_cnt,const CameraFrameStatus& status);
        void updateCameraStatus(int num_tracked, int num_total, int rejected_error, int rejected_desc, const std::vector<float>& optical_flow_errors);



#if USE_ORB_SLAM2_DETECTOR
        // Use ORB extractor provided in ORB_SLAM2
        std::shared_ptr<ORB_SLAM2::ORBextractor> ORBextractor_;
#else
        cv::Ptr<cv::ORB> ORBdetector;
        int nFeatures;
        float scaleFactor;
        int nlevels;
        int edgeThreshold;
        int firstLevel;
        int WTA_K;
        int scoreType;
        int patchSize;
        int fastThreshold;
#endif

    cv::Ptr<cv::DescriptorMatcher> matcher;             // Descriptor matcher

    int max_dist;
    cv::Mat prev_img, cur_img, forw_img;                          // Image // cur_img : image frame t-1, forw_img : frame t
    vector<cv::KeyPoint> prev_pts, cur_pts, forw_pts;             // Feature points
    cv::Mat cur_desc, forw_desc;                        // Descriptors for the feature points
    cv::Mat fisheye_mask;
    cv::Mat mask;

    vector<cv::KeyPoint> n_pts;                         // so luong feature points detected in the current image   
    cv::Mat n_desc;                                     // so luong descriptor cua feature points
    
    // UNDISTORTED POINTS
    vector<cv::Point2f> prev_un_pts, cur_un_pts;        // Undistorted points in the previous and current images
    vector<cv::Point2f> pts_velocity;                   // Velocity of the points between two frames
    map<int, cv::Point2f> cur_un_pts_map;               // Map of undistorted points in the current image
    map<int, cv::Point2f> prev_un_pts_map;              // Map of undistorted points in the previous image
    double cur_time;                                    // current time of the image (frame t)
    double prev_time;                                   // previous time of the image (frame t-1)

    // QUADTREE
    struct Node
    {
        pair<cv::Point2f, cv::Point2f> Size;
        vector<int> Point_Lists;
    };
    vector<int> List;
    Node node;
    vector<vector<int>> listid_List;
    vector<Node> node_List;

    // CAMERA
    std::shared_ptr<MCVIO::MCVIOcamera> cam;

    bool useOpticalFlow = true;

    int current_frame_id = 0;
    int camera_id = 0;
    // Camera status tracking
    CameraFrameStatus current_frame_status;
    CameraFrameStatus prev_frame_status;

    // Feature metadata
    std::map<int, double> feature_quality_scores;           // Quality score cho tung feature
    std::map<int, double> feature_detection_timestamps;     // Khi nao feature duoc detect
    std::map<int, int> feature_frame_ids;                   // Frame ID khi feature duoc detect
    std::map<int, int> feature_rejection_history;           // Lich su reject

    };
}