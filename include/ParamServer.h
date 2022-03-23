#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE 

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

enum class SensorType { VELODYNE, OUSTER, LIVOX };

class ParamServer
{
public:

    ros::NodeHandle nh;
    std::string robot_id;
    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    //Frames
    string lidarFrame;
    string baseFrame;
    string odomFrame;
    string mapFrame;
    // Save pcd
    bool savePCD;
    string savePCDDirectory;
    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downSampleRate;
    float lidarMinRange;
    float lidarMaxRange;
    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRpyWeight;
    vector<double> extRotVec;
    vector<double> extRpyVec;
    vector<double> extTransVec;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRpy;
    Eigen::Vector3d extTrans;
    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;
    // voxel filter params
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;
    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;
    // Surrounding map
    float surroundingKeyFrameAddingDistThreshold;
    float surroundingKeyFrameAddingAngleThreshold;
    float surroundingKeyFrameDensity;
    float surroundingKeyFrameSearchRadius;
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyFrameSize;
    float historyKeyFrameSearchRadius;
    float historyKeyFrameSearchTimeDiff;
    int   historyKeyFrameSearchNum;
    float historyKeyFrameFitnessScore;
    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("tc_lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("tc_lio_sam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("tc_lio_sam/odomTopic", odomTopic, "odometry/imu");

        nh.param<std::string>("tc_lio_sam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("tc_lio_sam/baseFrame", baseFrame, "base_link");
        nh.param<std::string>("tc_lio_sam/odomFrame", odomFrame, "odom");
        nh.param<std::string>("tc_lio_sam/mapFrame", mapFrame, "map");

        nh.param<bool>("tc_lio_sam/savePCD", savePCD, false);
        nh.param<std::string>("tc_lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        std::string sensorStr;
        nh.param<std::string>("tc_lio_sam/sensor", sensorStr, "");
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "livox")
        {
            sensor = SensorType::LIVOX;
        }
        else
        {
            ROS_ERROR_STREAM(
                "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox'): " << sensorStr);
            ros::shutdown();
        }

        nh.param<int>("tc_lio_sam/N_SCAN", N_SCAN, 16);
        nh.param<int>("tc_lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("tc_lio_sam/downSampleRate", downSampleRate, 1);
        nh.param<float>("tc_lio_sam/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("tc_lio_sam/lidarMaxRange", lidarMaxRange, 1000.0);

        nh.param<float>("tc_lio_sam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("tc_lio_sam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("tc_lio_sam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("tc_lio_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("tc_lio_sam/imuGravity", imuGravity, 9.80511);
        nh.param<float>("tc_lio_sam/imuRpyWeight", imuRpyWeight, 0.01);
        nh.param<vector<double>>("tc_lio_sam/extrinsicRot", extRotVec, vector<double>());
        nh.param<vector<double>>("tc_lio_sam/extrinsicRPY", extRpyVec, vector<double>());
        nh.param<vector<double>>("tc_lio_sam/extrinsicTrans", extTransVec, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotVec.data(), 3, 3);
        extRpy = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRpyVec.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransVec.data(), 3, 1);

        nh.param<float>("tc_lio_sam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("tc_lio_sam/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("tc_lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("tc_lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("tc_lio_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("tc_lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("tc_lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<int>("tc_lio_sam/numberOfCores", numberOfCores, 2);
        nh.param<double>("tc_lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("tc_lio_sam/surroundingKeyFrameAddingDistThreshold", surroundingKeyFrameAddingDistThreshold, 1.0);
        nh.param<float>("tc_lio_sam/surroundingKeyFrameAddingAngleThreshold", surroundingKeyFrameAddingAngleThreshold, 0.2);
        nh.param<float>("tc_lio_sam/surroundingKeyFrameDensity", surroundingKeyFrameDensity, 1.0);
        nh.param<float>("tc_lio_sam/surroundingKeyFrameSearchRadius", surroundingKeyFrameSearchRadius, 50.0);

        nh.param<bool>("tc_lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("tc_lio_sam/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("tc_lio_sam/surroundingKeyFrameSize", surroundingKeyFrameSize, 50);
        nh.param<float>("tc_lio_sam/historyKeyFrameSearchRadius", historyKeyFrameSearchRadius, 10.0);
        nh.param<float>("tc_lio_sam/historyKeyFrameSearchTimeDiff", historyKeyFrameSearchTimeDiff, 30.0);
        nh.param<int>("tc_lio_sam/historyKeyFrameSearchNum", historyKeyFrameSearchNum, 25);
        nh.param<float>("tc_lio_sam/historyKeyFrameFitnessScore", historyKeyFrameFitnessScore, 0.3);

        nh.param<float>("tc_lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("tc_lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("tc_lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imuIn)
    {
        sensor_msgs::Imu imu_out = imuIn;
        // rotate acceleration
        Eigen::Vector3d acc(imuIn.linear_acceleration.x, imuIn.linear_acceleration.y, imuIn.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imuIn.angular_velocity.x, imuIn.angular_velocity.y, imuIn.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        return imu_out;
    }
};

template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

template<typename T>
void imuAngularToRosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template<typename T>
void imuAccelToRosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

template<typename T>
void imuRpyToRosRpy(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif