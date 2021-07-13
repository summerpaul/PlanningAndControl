#ifndef ROADMAPINFO_H
#define ROADMAPINFO_H
namespace route {
namespace roadmap {

//关键点,用于描述路段的起点与终点
struct WayPoint{
    int id;
    double x;
    double y;
    double yaw;
    int type;
    WayPoint():id(0),x(0),y(0),yaw(0),type(0){}
};
//离散点,用于描述离散后的具体点
struct Point{
    double x;
    double y;
    double yaw;
    double curvature;
};
//曲线类型
enum LaneType{
    LINE  = 0,
    POLY3 = 1,
    BEZIER= 2
};

//用于描述路段的形状信息
struct LaneShape{
    LaneType laneType;
    double p1;
    double p2;
    double p3;
    double p4;
    double length;
    LaneShape():laneType(LINE),p1(0),p2(0),p3(0),p4(0),length(0){}

};
//用于描述路段的其他信息
struct LaneInfo{
    double speed;
    int motionType;
    int obstacleType;
    LaneInfo():speed(0), motionType(1), obstacleType(0){}


};
//路段信息的整合
struct Lane{
    int id;
    int Bid;
    int Fid;
    LaneInfo info;
    LaneShape shape;
    Lane():id(0), Bid(0), Fid(0){}
};
}


}
#endif // ROADMAPINFO_H
