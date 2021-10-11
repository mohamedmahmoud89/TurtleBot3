#ifndef ODOM_FUNC_HPP
#define ODOM_FUNC_HPP

#include "common.hpp"
#include <cmath>

#define MIN_TO_SEC 60

inline f32 odomCalcDrivenDist(const si16 wheelVelocity){
    // num revolutions per cycles
    f32 num_revolutions = 
        (static_cast<f32>(RobotCfg::rpmOffset*wheelVelocity))/(MIN_TO_SEC*SystemCfg::rate_hz);
    
    // the driven distance
    f32 driven_dist = num_revolutions*RobotCfg::wheelCircumference_mm;

    return driven_dist;
}

inline void odomEstimatePos(RobotPos& robotPos,const WheelVelocity& wheelVel){
    if(wheelVel.left_rpm==wheelVel.right_rpm){
        f32 driven_dist=odomCalcDrivenDist(wheelVel.left_rpm);

        robotPos.x_mm+=(driven_dist*cos(robotPos.theta_rad));
        robotPos.y_mm+=(driven_dist*sin(robotPos.theta_rad));

        return;
    }
    
    f32 dd_r=odomCalcDrivenDist(wheelVel.right_rpm);
    f32 dd_l=odomCalcDrivenDist(wheelVel.left_rpm);
    
    if(abs(dd_r)>abs(dd_l)){
        f32 radius=static_cast<f32>(RobotCfg::width_mm)/2.0;
        f32 alpha_rad((dd_r-dd_l)/RobotCfg::width_mm);

        radius+= (dd_l/alpha_rad);

        f32 c_x(robotPos.x_mm-(radius*sin(robotPos.theta_rad)));
        f32 c_y(robotPos.y_mm+(radius*cos(robotPos.theta_rad)));
        robotPos.theta_rad+=alpha_rad;
        robotPos.theta_rad=fmod(robotPos.theta_rad+(2*M_PI),2*M_PI);
        robotPos.x_mm = c_x + radius*sin(robotPos.theta_rad);
        robotPos.y_mm = c_y - radius*cos(robotPos.theta_rad);

        return;
    }

    f32 radius=static_cast<f32>(RobotCfg::width_mm)/2.0;
    f32 alpha_rad((dd_l-dd_r)/RobotCfg::width_mm);

    radius+= (dd_r/alpha_rad);

    f32 c_x(robotPos.x_mm+(radius*sin(robotPos.theta_rad)));
    f32 c_y(robotPos.y_mm-(radius*cos(robotPos.theta_rad)));
    robotPos.theta_rad-=alpha_rad;
    robotPos.theta_rad=fmod(robotPos.theta_rad+(2*M_PI),2*M_PI);
    robotPos.x_mm = c_x - radius*sin(robotPos.theta_rad);
    robotPos.y_mm = c_y + radius*cos(robotPos.theta_rad);
}

#endif