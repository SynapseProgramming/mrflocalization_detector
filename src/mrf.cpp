#include <mrflocalization_detector/mrf.h>

void MRF::scanCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    std::cout << "scan message received!\n";

    // plot point_out in rviz for testing
    visualization_msgs::Marker marker;
    marker.header.frame_id = mapFrameName;
    marker.header.stamp = ros::Time::now();
    marker.ns = "laser_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    std::vector<std::pair<double, double>> laserPoints;
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        double d = (double)i * msg->angle_increment + msg->angle_min;
        double xx = msg->ranges[i] * cos(d);
        double yy = msg->ranges[i] * sin(d);
        if (xx >= -100 && xx <= 100 && yy >= -100 && yy <= 100)
            laserPoints.push_back(std::make_pair(xx, yy));
    }
    // translate laser points to be wrt to map frame
    geometry_msgs::Point point;
    for (int i = 0; i < laserPoints.size(); i++)
    {
        auto it = laserPoints[i];
        geometry_msgs::PointStamped point_in;
        geometry_msgs::PointStamped point_out;
        point_in.header.frame_id = scanFrameName;
        point_in.point.x = it.first;
        point_in.point.y = it.second;
        try
        {
            point_out = tfBuffer.transform(point_in, mapFrameName);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        point = point_out.point;
        if (i % 10 == 0)
            marker.points.push_back(point);
    }

    markerPub_.publish(marker);

    gotScan_ = true;
}

void MRF::mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{

    if (gotMap_)
        return;
    // perform distance transform to build the distance field
    mapWidth_ = msg->info.width;
    mapHeight_ = msg->info.height;
    mapResolution_ = msg->info.resolution;
    std::cout << "map width: " << mapWidth_ << "\n";
    std::cout << "map height: " << mapHeight_ << "\n";
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
    std::cout << "origin x: " << mapOrigin_.getX() << "\n";
    std::cout << "origin y: " << mapOrigin_.getY() << "\n";
    std::cout << "origin yaw: " << mapOrigin_.getYaw() << "\n";
    gotMap_ = true;
}