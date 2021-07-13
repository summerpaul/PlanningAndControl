#include "segment.h"
namespace route {
namespace segment {

Segment::Segment()
{

}
Segment::~Segment()
{

}
void Segment::setSegPara(const Lane& lane)
{
    m_lane = lane;

}
Point Segment::findTheoryPoint(const Point& currentPose, double length)
{
    int nearestPointIndex = 0;
    Point nearestPoint;
    for(; length > calPose2Pose(currentPose, m_discretePoints[nearestPointIndex]); nearestPointIndex +=1)
    {
        if(nearestPointIndex +1 >= int(m_discretePoints.size()))
        {
            break;
        }
        nearestPoint = m_discretePoints[nearestPointIndex];
    }
    return nearestPoint;

}
}
}
