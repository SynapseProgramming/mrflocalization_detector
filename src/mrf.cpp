#include <mrflocalization_detector/mrf.h>

void MRF::scanCB(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    std::cout << "scan message received!\n";

    std::vector<std::pair<double, double>> laserPoints;
    for (int i = 0; i < msg->ranges.size(); i++)
    {
        double d = (double)i * msg->angle_increment + msg->angle_min;
        double xx = msg->ranges[i] * cos(d);
        double yy = msg->ranges[i] * sin(d);
        if (xx >= -100 && xx <= 100 && yy >= -100 && yy <= 100)
        {
            // translate laser points to be wrt to map frame
            geometry_msgs::PointStamped point_in;
            geometry_msgs::PointStamped point_out;
            point_in.header.frame_id = scanFrameName;
            point_in.point.x = xx;
            point_in.point.y = yy;
            try
            {
                point_out = tfBuffer.transform(point_in, mapFrameName);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                return;
            }
            double mapX = point_out.point.x;
            double mapY = point_out.point.y;
            // image width and image height are pixel indices
            int imageWidth = 0;
            int imageHeight = 0;
            xy2uv(mapX, mapY, &imageWidth, &imageHeight);
            if (onMap(imageWidth, imageHeight))
            {
                std::cout << imageWidth << " " << imageHeight << " " << distMap_.at<float>(imageHeight, imageWidth) << "\n";
            }
        }
    }

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

bool MRF::onMap(int u, int v)
{
    if (0 <= u && u < mapWidth_ && 0 <= v && v < mapHeight_)
        return true;
    else
        return false;
}

void MRF::xy2uv(double x, double y, int *u, int *v)
{
    double dx = x - mapOrigin_.getX();
    double dy = y - mapOrigin_.getY();
    double yaw = -mapOrigin_.getYaw();
    double xx = dx * cos(yaw) - dy * sin(yaw);
    double yy = dx * sin(yaw) + dy * cos(yaw);
    *u = (int)(xx / mapResolution_);
    *v = (int)(yy / mapResolution_);
}