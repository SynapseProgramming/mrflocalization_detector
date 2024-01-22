#ifndef __POSE_H__
#define __POSE_H__

#include <cmath>
class Pose
{
private:
    double x_, y_, yaw_;

    void modifyYaw(void)
    {
        while (yaw_ < -M_PI)
            yaw_ += 2.0 * M_PI;
        while (yaw_ > M_PI)
            yaw_ -= 2.0 * M_PI;
    }

public:
    Pose() : x_(0.0), y_(0.0), yaw_(0.0){};

    Pose(double x, double y, double yaw) : x_(x), y_(y), yaw_(yaw){};

    ~Pose(){};

    inline void setX(double x) { x_ = x; }
    inline void setY(double y) { y_ = y; }
    inline void setYaw(double yaw) { yaw_ = yaw, modifyYaw(); }
    inline void setPose(double x, double y, double yaw) { x_ = x, y_ = y, yaw_ = yaw, modifyYaw(); }
    inline void setPose(Pose p) { x_ = p.x_, y_ = p.y_, yaw_ = p.yaw_, modifyYaw(); }

    inline double getX(void) { return x_; }
    inline double getY(void) { return y_; }
    inline double getYaw(void) { return yaw_; }
    inline Pose getPose(void) { return Pose(x_, y_, yaw_); }

}; // class Pose

#endif // __POSE_H__