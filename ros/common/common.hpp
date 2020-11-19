#ifndef COMMON_TYPES_HPP
#define COMMON_TYPES_HPP

using u8  = unsigned short;
using u16 = unsigned int;
using u32 = unsigned long;

using si8  = short;
using si16 = int;
using si32 = long;

using f32 = float;
using f64 = double;

class RobotCfg{
public:
    static constexpr f32 rpmOffset{0.229};
    static const u8 wheelDiameter_mm{66};
};

class SystemCfg{
public:
    static const u8 rate_hz{20};
};

class WheelVelocity{
public:
    u8 left_rpm;
    u8 right_rpm;
};

#endif