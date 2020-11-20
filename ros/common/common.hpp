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
    static const u16 wheelDiameter_mm{66};
};

class SystemCfg{
public:
    static const u16 rate_hz{20};
};

class WheelVelocity{
public:
    si16 left_rpm;
    si16 right_rpm;
};

#endif