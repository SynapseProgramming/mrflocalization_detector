#include <mrflocalization_detector/mrf.h>

void MRF::scanCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    std::cout << "scan message received!\n";
}

void MRF::mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    // perform distance transform to build the distance field
    mapWidth_ = msg->info.width;
    mapHeight_ = msg->info.height;
    mapResolution_ = msg->info.resolution;
    cv::Mat binMap(mapHeight_, mapWidth_, CV_8UC1);
    for (int v = 0; v < mapHeight_; v++)
    {
        for (int u = 0; u < mapWidth_; u++)
        {
            int node = v * mapWidth_ + u;
            int val = msg->data[node];
            if (val == 100)
                binMap.at<uchar>(v, u) = 0;
            else
                binMap.at<uchar>(v, u) = 1;
        }
    }
    cv::Mat distMap(mapHeight_, mapWidth_, CV_32FC1);
    cv::distanceTransform(binMap, distMap, cv::DIST_L2, 5);
    for (int v = 0; v < mapHeight_; v++)
    {
        for (int u = 0; u < mapWidth_; u++)
        {
            float d = distMap.at<float>(v, u) * (float)mapResolution_;
            distMap.at<float>(v, u) = d;
        }
    }
    distMap_ = distMap;
    tf2::Quaternion q(msg->info.origin.orientation.x,
                     msg->info.origin.orientation.y,
                     msg->info.origin.orientation.z,
                     msg->info.origin.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    mapOrigin_.setX(msg->info.origin.position.x);
    mapOrigin_.setY(msg->info.origin.position.y);
    mapOrigin_.setYaw(yaw);
    gotMap_ = true;
}