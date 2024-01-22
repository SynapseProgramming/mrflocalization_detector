#ifndef __MRF_H__
#define __MRF_H__

#include <iostream>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <mrflocalization_detector/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

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
    std::string mapName_ = "map";
    // map
    cv::Mat distMap_;
    double mapResolution_;
    Pose mapOrigin_;
    int mapWidth_, mapHeight_;
    bool gotMap_ = false;

    // scan
    bool gotScan_ = false;
    std::string scanName_ = "scan";

    // ros subscribers and publishers
    ros::NodeHandle nh_;
    ros::Subscriber scanSub_;
    ros::Subscriber mapSub_;

    // tf2_ros::Buffer tfBuffer;

    // callback function definitions
    void scanCB(const sensor_msgs::LaserScan::ConstPtr &msg);
    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg);

public:
    MRF() : nh_("~")
    {
        // TODO: topic names should be parameters
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
                          mapName_.c_str());
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
                          scanName_.c_str());
                exit(1);
            }
            loopRate.sleep();
        }
    }
};

#endif