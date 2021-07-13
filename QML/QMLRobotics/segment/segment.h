#ifndef SEGMENT_H
#define SEGMENT_H
#include <vector>
#include "common/mapInfo.h"
#include "common/common.h"
namespace route {
namespace segment {
using namespace common;
using namespace std;
class Segment
{
public:
    Segment();
    virtual ~Segment();
//    设置段的信息
    void setSegPara(const Lane& lane);
    double getStartAngle(){return m_lane.BP.yaw;}
    double getEndAngle(){return m_lane.FP.yaw;}
    double getSegSpeed(){return m_lane.info.speed;}
    int getMotionType(){return m_lane.info.motionType;}
    int getObstacleType(){return m_lane.info.obstacleType;}

//    找到最近点
    Point findTheoryPoint(const Point& currentPose, double length);
    vector<Point> getDiscreatePoints(){return m_discretePoints;}
    virtual void discretePoints(const double minInterval) = 0;


private:
    Lane m_lane;
    vector<Point> m_discretePoints;
};
}//segment
}//route


#endif // SEGMENT_H
