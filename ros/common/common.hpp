#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

#include <mutex>

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
    static const u16 maxDepth_mm{3500};
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

class Lock{
    std::mutex& m;
public:
    Lock(std::mutex& ref):m(ref){m.lock();}
    ~Lock(){m.unlock();}
};

#endif