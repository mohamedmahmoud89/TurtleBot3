#ifndef ODOM_FUNC_HPP
#define ODOM_FUNC_HPP

#include "funcBase.hpp"
#include "odomItf.hpp"

#define MIN_TO_SEC 60

class OdomFunc : public FuncBase{
    f32 calcDrivenDist(const si16 wheelVelocity){
        // num revolutions per cycles
        f32 num_revolutions = 
            (static_cast<f32>(RobotCfg::rpmOffset*wheelVelocity))/(MIN_TO_SEC*SystemCfg::rate_hz);
        
        // the driven distance
        f32 driven_dist = num_revolutions*RobotCfg::wheelCircumference_mm;

        return driven_dist;
    }
public:
    OdomFunc(const OdomInputData& in_ref,OdomOutputData& out_ref): FuncBase(in_ref,out_ref){}
    void exec() override{
        const OdomInputData& inData = dynamic_cast<const OdomInputData&>(this->inputs);
        OdomOutputData& outData = dynamic_cast<OdomOutputData&>(this->outputs);

        if(inData.wheelVel.left_rpm==inData.wheelVel.right_rpm){
            f32 driven_dist=calcDrivenDist(inData.wheelVel.left_rpm);

            outData.robotPos.x_mm+=(driven_dist*cos(outData.robotPos.theta_rad));
            outData.robotPos.y_mm+=(driven_dist*sin(outData.robotPos.theta_rad));

            return;
        }
        
        f32 dd_r=calcDrivenDist(inData.wheelVel.right_rpm);
        f32 dd_l=calcDrivenDist(inData.wheelVel.left_rpm);
        
        assert(dd_r!=dd_l);
        
        if(abs(dd_r)>abs(dd_l)){
            f32 radius=static_cast<f32>(RobotCfg::width_mm)/2.0;
            f32 alpha_rad((dd_r-dd_l)/RobotCfg::width_mm);

            radius+= (dd_l/alpha_rad);

            f32 c_x(outData.robotPos.x_mm-(radius*sin(outData.robotPos.theta_rad)));
            f32 c_y(outData.robotPos.y_mm+(radius*cos(outData.robotPos.theta_rad)));
            outData.robotPos.theta_rad+=alpha_rad;
            outData.robotPos.theta_rad=fmod(outData.robotPos.theta_rad+(2*M_PI),2*M_PI);
            outData.robotPos.x_mm = c_x + radius*sin(outData.robotPos.theta_rad);
            outData.robotPos.y_mm = c_y - radius*cos(outData.robotPos.theta_rad);

            return;
        }

        f32 radius=static_cast<f32>(RobotCfg::width_mm)/2.0;
        f32 alpha_rad((dd_l-dd_r)/RobotCfg::width_mm);

        radius+= (dd_r/alpha_rad);

        f32 c_x(outData.robotPos.x_mm+(radius*sin(outData.robotPos.theta_rad)));
        f32 c_y(outData.robotPos.y_mm-(radius*cos(outData.robotPos.theta_rad)));
        outData.robotPos.theta_rad-=alpha_rad;
        outData.robotPos.theta_rad=fmod(outData.robotPos.theta_rad+(2*M_PI),2*M_PI);
        outData.robotPos.x_mm = c_x - radius*sin(outData.robotPos.theta_rad);
        outData.robotPos.y_mm = c_y + radius*cos(outData.robotPos.theta_rad);
    }
};
#endif