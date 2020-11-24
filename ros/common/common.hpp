#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

using u16 = unsigned int;
using u32 = unsigned long;

using si16 = int;
using si32 = long;

using f32 = float;
using f64 = double;

class RobotCfg{
public:
    static constexpr f32 rpmOffset{0.229};
    static constexpr f32 wheelCircumference_mm{207.345115137};
    static const u16 width_mm{159}; 
};

class SystemCfg{
public:
    static const u16 rate_hz{20};
};

class WheelVelocity{
public:
    WheelVelocity(const si16 l,const si16 r):left_rpm(l),right_rpm(r){}
    si16 left_rpm{0};
    si16 right_rpm{0};
};

class RobotPos{
public:
    f64 x_mm{0};
    f64 y_mm{0};
    f64 theta_rad{0};
};

class AngConversions{
    static constexpr f32 degToRad{0.01745329251};
    static constexpr f32 radToDegree{57.2957795131};
};

#endif