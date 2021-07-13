#ifndef COMMON_H
#define COMMON_H
#include <cmath>
#include "mapInfo.h"

namespace route {
namespace common {

static double calPose2Pose(const Point &pose1, const Point &pose2)
    {
        double dist = sqrt((pose1.x - pose2.x) * (pose1.x - pose2.x) + (pose1.y - pose2.y) * (pose1.y - pose2.y));

        return dist;
    }

    static double calAtan2(const Point &pose1, const Point &pose2)
    {
        double angle;
        double delt_x = pose1.x - pose2.x;
        double delt_y = pose1.y - pose2.y;
        angle = atan2(delt_y, delt_x);

        return angle;
    }


    static double pi2pi(const double &oldAngle)
    {
        double angle = fmod(oldAngle, 2 * M_PI);
        if (oldAngle > M_PI)
        {
            angle = oldAngle - 2 * M_PI;
        }
        else if (oldAngle < -M_PI)
        {
            angle = oldAngle + 2 * M_PI;
        }
        else
        {
            angle = oldAngle;
        }

        return angle;
    }

    static double angleSumAndNormalize(double angle1, double angle2)
    {
        double sum = fmod((angle1 + angle2), 2 * M_PI);
        if (sum > M_PI)
        {
            sum -= 2 * M_PI;
        }
        else if (sum < -M_PI)
        {
            sum += 2 * M_PI;
        }

        return sum;
    }

    static double deg2rad(double x)
    {
        return x * M_PI / 180;
    }

    static double rad2deg(double x)
    {
        return x * 180 / M_PI;
    }

    static double mdeg2rad(double x)
    {
        return x * M_PI / 180 / 1000;
    }

    static double rad2mrad(double x)
    {
        return x * 180 * 1000 / M_PI;
    }
}
}


#endif // COMMON_H
