#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <mutex>
#include <vector>

using namespace std;

using u16 = unsigned int;
using u32 = unsigned long;

using si16 = int;
using si32 = long;

using f32 = float;
using f64 = double;

struct RobotCfg{
    static constexpr f32 rpmOffset{0.229};
    static constexpr f32 wheelCircumference_mm{207.345115137};
    static const u16 width_mm{159}; 
};

struct SystemCfg{
    static const u16 rate_hz{20};
};

struct LdsSensorCfg{
    static const u16 minDepth_mm{120};
    static const u16 maxDepth_mm{700};
    static const u16 sensorOffset_mm{20};
};

struct WheelVelocity{
    WheelVelocity(const si16 l,const si16 r):left_rpm(l),right_rpm(r){}
    si16 left_rpm{0};
    si16 right_rpm{0};
};

struct RobotPos{
    RobotPos(const f64& x,const f64& y,const f64& theta):
        x_mm(x),
        y_mm(y),
        theta_rad(theta){}
    f64 x_mm{0};
    f64 y_mm{0};
    f64 theta_rad{0};
};

struct AngConversions{
    static constexpr f32 degToRad{0.01745329251};
    static constexpr f32 radToDegree{57.2957795131};
};

struct Point2D{
    Point2D():x_mm(0),y_mm(0){}
    Point2D(f32 x,f32 y):x_mm(x),y_mm(y){}
    Point2D(const Point2D& p):x_mm(p.x_mm),y_mm(p.y_mm){}
    f32 x_mm{0};
    f32 y_mm{0};
};

struct Point2DPolar{
    Point2DPolar():r_mm(0),theta_rad(0){}
    Point2DPolar(f32 r,f32 theta):r_mm(r),theta_rad(theta){}
    Point2DPolar(const Point2DPolar& p):r_mm(p.r_mm),theta_rad(p.theta_rad){}
    f32 r_mm{0};
    f32 theta_rad{0};
};

struct Line2D{
    Line2D():p1(),p2(){}
    Line2D(const Point2D& point_1,const Point2D& point_2):p1(point_1),p2(point_2){}
    Point2D p1;
    Point2D p2;
    f32 angle;
};

struct Line2DPolar{
    Point2DPolar center;
    f32 start_ang_rad{0};
    f32 end_ang_rad{0};
};


struct LfxFeats{
    vector<Point2D> points;
    vector<Line2D> lines;
    vector<Point2D> corners;
    vector<Point2D> edges;
};

class Lock{
    mutex& m;
public:
    Lock(mutex& ref):m(ref){m.lock();}
    ~Lock(){m.unlock();}
};

#endif