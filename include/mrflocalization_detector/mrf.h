#ifndef __MRF_H__
#define __MRF_H__

#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>

enum MeasurementClass
{
    ALIGNED = 0,
    MISALIGNED = 1,
    UNKNOWN = 2
};

class MRF
{
private:
    // ros subscribers and publishers
    ros::NodeHandle nh_;
    ros::Subscriber scanSub_;

    // tf2_ros::Buffer tfBuffer;


    void scanCB(const sensor_msgs::LaserScan::ConstPtr &msg);

public:
    MRF() : nh_("~")
    {
        scanSub_ = nh_.subscribe("/scan_front", 1, &MRF::scanCB, this);
    }
};

#endif