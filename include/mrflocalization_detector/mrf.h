#ifndef __MRF_H__
#define __MRF_H__

#include <iostream>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <mrflocalization_detector/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

enum MeasurementClass
{
    ALIGNED = 0,
    MISALIGNED = 1,
    UNKNOWN = 2
};

class MRF
{
private:
    // tf names
    std::string mapTopicName;
    std::string mapFrameName;
    // map
    cv::Mat distMap_;
    double mapResolution_;
    Pose mapOrigin_;
    int mapWidth_, mapHeight_;
    bool gotMap_ = false;

    // scan
    bool gotScan_ = false;
    std::string scanTopicName;
    std::string scanFrameName;

    // ros subscribers and publishers
    ros::NodeHandle nh_;
    ros::Subscriber scanSub_;
    ros::Subscriber mapSub_;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    // callback function definitions
    void scanCB(const sensor_msgs::LaserScan::ConstPtr &msg);
    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    bool onMap(int u, int v);

    void xy2uv(double x, double y, int *u, int *v);

public:
    MRF() : nh_("~"), tfListener(tfBuffer),
            mapTopicName("map"),
            mapFrameName("map"),
            scanTopicName("scan"),
            scanFrameName("laser_front")
    {

        nh_.param("map_topic_name", mapTopicName, mapTopicName);
        nh_.param("map_frame_name", mapFrameName, mapFrameName);
        nh_.param("scan_topic_name", scanTopicName, scanTopicName);
        nh_.param("scan_frame_name", scanFrameName, scanFrameName);

        scanSub_ = nh_.subscribe("/scan_front", 1, &MRF::scanCB, this);
        mapSub_ = nh_.subscribe("/map", 1, &MRF::mapCB, this);

        ros::Rate loopRate(10);

        // check map
        int mapFailedCnt = 0;
        while (ros::ok())
        {
            ros::spinOnce();
            if (gotMap_)
                break;
            mapFailedCnt++;
            // 30 sec
            if (mapFailedCnt >= 300)
            {
                ROS_ERROR("Cannot get a map message."
                          " Did you publish the map?"
                          " Expected map topic name is %s\n",
                          mapTopicName.c_str());
                exit(1);
            }
            loopRate.sleep();
        }

        // check scan
        int scanFailedCnt = 0;
        while (ros::ok())
        {
            ros::spinOnce();
            if (gotScan_)
                break;
            scanFailedCnt++;
            if (scanFailedCnt >= 300)
            {
                ROS_ERROR("Cannot get a scan message."
                          " Did you publish the scan?"
                          " Expected scan topic name is %s\n",
                          scanTopicName.c_str());
                exit(1);
            }
            loopRate.sleep();
        }
    }
};

#endif