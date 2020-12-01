#ifndef ODOM_NODE_HPP
#define ODOM_NODE_HPP

#include "nodeBase.hpp"
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Pose2D.h>

using namespace std;

#define MIN_TO_SEC 60 

class OdomNode : public RosNodeBase{
    void update() override{
        geometry_msgs::Pose2D msg;
        
        //has to be locked to avoid data contention
        {
            Lock lock(posLock);
            msg.x=robotPos.x_mm;
            msg.y=robotPos.y_mm;
            msg.theta=robotPos.theta_rad;
        }
        posDataPub.publish(msg);
        cout<<"pos = "<<msg.x<<" , "<<msg.y<<" , "<<msg.theta*AngConversions::radToDegree<<endl;
    }

    static f32 calcDrivenDist(const si16 wheelVelocity){
            // num revolutions per cycles
            f32 num_revolutions = 
                (static_cast<f32>(RobotCfg::rpmOffset*wheelVelocity))/(MIN_TO_SEC*SystemCfg::rate_hz);
            
            // the driven distance
            f32 driven_dist = num_revolutions*RobotCfg::wheelCircumference_mm;

            return driven_dist;
    }
    
    static void updatePos(const WheelVelocity& wheelVelocity){
        if(wheelVelocity.left_rpm==wheelVelocity.right_rpm){
            f32 driven_dist=calcDrivenDist(wheelVelocity.left_rpm);

            // has to be locked
            {
                Lock lock(posLock);
                robotPos.x_mm+=(driven_dist*cos(robotPos.theta_rad));
                robotPos.y_mm+=(driven_dist*sin(robotPos.theta_rad));
            }

            return;
        }
        
        f32 dd_r=calcDrivenDist(wheelVelocity.right_rpm);
        f32 dd_l=calcDrivenDist(wheelVelocity.left_rpm);
        
        assert(dd_r!=dd_l);
        
        if(abs(dd_r)>abs(dd_l)){
            f32 radius=static_cast<f32>(RobotCfg::width_mm)/2.0;
            f32 alpha_rad((dd_r-dd_l)/RobotCfg::width_mm);

            radius+= (dd_l/alpha_rad);
            
            {
                Lock lock(posLock);
                f32 c_x(robotPos.x_mm-(radius*sin(robotPos.theta_rad)));
                f32 c_y(robotPos.y_mm+(radius*cos(robotPos.theta_rad)));
                robotPos.theta_rad+=alpha_rad;
                robotPos.theta_rad=fmod(robotPos.theta_rad,2*M_PI);
                robotPos.x_mm = c_x + radius*sin(robotPos.theta_rad);
                robotPos.y_mm = c_y - radius*cos(robotPos.theta_rad);
            }
            return;
        }

        f32 radius=static_cast<f32>(RobotCfg::width_mm)/2.0;
        f32 alpha_rad((dd_l-dd_r)/RobotCfg::width_mm);

        radius+= (dd_r/alpha_rad);
        
        {
            Lock lock(posLock);
            f32 c_x(robotPos.x_mm+(radius*sin(robotPos.theta_rad)));
            f32 c_y(robotPos.y_mm-(radius*cos(robotPos.theta_rad)));
            robotPos.theta_rad-=alpha_rad;
            robotPos.theta_rad=fmod(robotPos.theta_rad,2*M_PI);
            robotPos.x_mm = c_x - radius*sin(robotPos.theta_rad);
            robotPos.y_mm = c_y + radius*cos(robotPos.theta_rad);
        }
    }  
    static void processData(const std_msgs::Int16MultiArray& msg){
        WheelVelocity vel(msg.data[0],msg.data[1]);
        
        updatePos(vel);
    }
public:
    OdomNode(): 
        RosNodeBase(),
        odomDataSub(n.subscribe("odom",100,processData)),
        posDataPub(n.advertise<geometry_msgs::Pose2D>("robotPos",1000)){}
private:
    Subscriber odomDataSub;
    Publisher posDataPub;
    static RobotPos robotPos;
    static mutex posLock;
};

RobotPos OdomNode::robotPos(0,0,M_PI_2);
mutex OdomNode::posLock;

#endif