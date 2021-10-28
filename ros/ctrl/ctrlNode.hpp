#ifndef CTRL_NODE_HPP
#define CTRL_NODE_HPP

#include "nodeBase.hpp"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <iostream>

using namespace std;

enum CtrlState{
    HMI_FOLLOW=0,
    PATH_FOLLOW
};

enum SegRelPosState{
    START=0,
    MID,
    END
};

struct CtrlNodeCfg{
    static constexpr f32 threshold_x_mm{10};
    static constexpr f32 threshold_y_mm{20};
    static constexpr f32 threshold_ang_deg{2};
};

class CtrlNode : public RosNodeBase{
    void update() override{
        vector<PathSeg> copy_path;
        RobotPos copy_pos;
        string copy_cmd;

        {
            Lock l(pathMux);
            copy_path=path;
        }

        {
            Lock l(posMux);
            copy_pos=globalPos;
        }

        {
            Lock l(cmdMux);
            copy_cmd=cmd;
        }

        string res_cmd = execStateMachine(copy_cmd,copy_path,copy_pos);
        
        publish(res_cmd);
    }

    bool isGoalReached(const RobotPos& goal,const RobotPos& pos){
        bool ret=false;
        if(
            fabs(pos.y_mm-goal.y_mm)<=CtrlNodeCfg::threshold_y_mm&&
            fmod(pos.theta_rad-goal.theta_rad+(2*M_PI),2*M_PI)<=2*CtrlNodeCfg::threshold_ang_deg*AngConversions::degToRad
        )
            ret=true;
        return ret;
    }

    string execStateMachine(const string command,const vector<PathSeg>& path,const RobotPos& pos){
        string ret;
        switch(state){
            case HMI_FOLLOW:
                if(command=="start"){
                    ret="stop";
                    state=PATH_FOLLOW;
                }
                else if(command=="abort"){
                    ret="stop";
                }
                else{
                    ret=command;
                }
                break;
            case PATH_FOLLOW:
                if(command=="abort"){
                    ret="stop";
                    state=HMI_FOLLOW;
                }
                else{
                    bool goal_reached=isGoalReached(path[path.size()-1].p2,pos);
                    
                    if(goal_reached){
                        ret="stop";
                        state=HMI_FOLLOW;
                    }
                    else{
                        ret=execPathCtrl(path,pos);
                    }
                }
                break;
            default:
                ret="stop";
                state=HMI_FOLLOW;
                break;
        }

        return ret;
    }

    string execPathCtrl(const vector<PathSeg>& path,const RobotPos& pos){
        string ret="stop";
        switch(segLoc){
            case START:
                if(isGoalReached(path[segNum].p1,pos)){
                    ret="stop";
                    segLoc=MID;
                }
                else{
                    f32 dist_pt_pt=sqrt(pow(path[segNum].p1.x_mm-pos.x_mm,2)+pow(path[segNum].p1.y_mm-pos.y_mm,2));
                    f32 theta=pos.theta_rad-path[segNum].p1.theta_rad;
                    theta=fmod(theta+(2*M_PI),2*M_PI);
                    f32 dist=dist_pt_pt*sin(theta);

                    // within angle and x thresholds --> stop
                    if(/*fabs(dist)<CtrlNodeCfg::threshold_x_mm&&*/theta<CtrlNodeCfg::threshold_ang_deg*AngConversions::degToRad){
                        ret="stop";
                        segLoc=MID;
                    }
                    // within x thresholds and looking left --> right
                    else if(/*fabs(dist)<CtrlNodeCfg::threshold_x_mm&&*/theta<M_PI){
                        ret="right_ctrl";
                    }
                    // within x thresholds and looking right --> left
                    else /*if(fabs(dist)<CtrlNodeCfg::threshold_x_mm)*/{
                        ret="left_ctrl";
                    }
                    // not within x thresholds and on the left side and perpendicular on the segment--> fwd
                    /*else if(dist<0&&fabs(theta+M_PI_2)<CtrlNodeCfg::threshold_ang_deg*AngConversions::degToRad){
                        ret="fwd";
                    }
                    // not within x thresholds and on the left side and not perp --> right
                    else if(dist<0){
                        ret="right";
                    }
                    // not within x thresholds and on the right side and perpendicular on the segment--> fwd
                    else if(fabs(theta-M_PI_2)<CtrlNodeCfg::threshold_ang_deg*AngConversions::degToRad){
                        ret="fwd";
                    }
                    // not within x thresholds and on the right side and not perp --> left
                    else{
                        ret="left";
                    }*/
                }
                break;
            case MID:
                {
                    // move till the pos projection on the seg is no longer on the seg
                    f32 dist_pt_pt=sqrt(pow(path[segNum].p2.x_mm-pos.x_mm,2)+pow(path[segNum].p2.y_mm-pos.y_mm,2));
                    f32 theta=atan2(path[segNum].p2.y_mm-pos.y_mm,path[segNum].p2.x_mm-pos.x_mm)-path[segNum].p2.theta_rad+M_PI_2;
                    f32 dist=dist_pt_pt*sin(theta);
                    f32 dist_norm=dist_pt_pt*cos(theta);
                    /*cout<<"p2= "<<path[segNum].p2.x_mm<<" , "<<path[segNum].p2.y_mm<<" , "<<path[segNum].p2.theta_rad*AngConversions::radToDegree<<endl;
                    cout<<"pos= "<<pos.x_mm<<" , "<<pos.y_mm<<" , "<<pos.theta_rad*AngConversions::radToDegree<<endl;
                    cout<<"dist_pt_pt= "<<dist_pt_pt<<endl;
                    cout<<"theta= "<<theta<<endl;
                    cout<<"dist= "<<dist<<endl;*/
                    if(dist<CtrlNodeCfg::threshold_y_mm){
                        ret="stop";
                        segLoc=END;
                    }
                    else{
                        theta=pos.theta_rad-path[segNum].p2.theta_rad;
                        theta=fmod(theta+(2*M_PI),2*M_PI);

                        // out of normal threshold and looking left --> right
                        if(dist_norm>CtrlNodeCfg::threshold_x_mm&&theta>CtrlNodeCfg::threshold_ang_deg&&theta<M_PI){
                            ret="right_ctrl";
                        }
                        // out of normal threshold and looking right --> left
                        else if(dist_norm>CtrlNodeCfg::threshold_x_mm&&theta>CtrlNodeCfg::threshold_ang_deg){
                            ret="left_ctrl";
                        }
                        else{
                            ret="fwd";
                        }
                    }
                }
                break;
            case END:
                {
                    f32 dist_pt_pt=sqrt(pow(path[segNum].p2.x_mm-pos.x_mm,2)+pow(path[segNum].p2.y_mm-pos.y_mm,2));
                    f32 theta=pos.theta_rad-path[segNum].p2.theta_rad;
                    theta=fmod(theta+(2*M_PI),2*M_PI);
                    f32 dist=dist_pt_pt*sin(theta);

                    // within relaxed angle thresholds --> stop
                    if(theta<=2*CtrlNodeCfg::threshold_ang_deg*AngConversions::degToRad){
                        ret="stop";
                        if(segNum<path.size()-1){
                            segLoc=START;
                            ++segNum;
                        }
                    }
                    // looking left --> right
                    else if(theta<M_PI){
                        ret="right_ctrl";
                    }
                    // looking right --> left
                    else{
                        ret="left_ctrl";
                    }   
                }
                break;
            default:
            break;
        };
        /*cout<<"segNum = "<<segNum<<endl;
        cout<<"segLoc = "<<segLoc<<endl;
        cout<<"seg cmd = "<<ret<<endl;*/

        return ret;
    }

    void publish(const string& command){
        std_msgs::String ctrlMsg;
        ctrlMsg.data=command;
        ctrlPub.publish(ctrlMsg);
    }

    static void storePos(const geometry_msgs::Pose2D& msg){
        Lock l(posMux);
        globalPos.x_mm=msg.x;
        globalPos.y_mm=msg.y;
        globalPos.theta_rad=msg.theta;
    }

    static void storePath(const geometry_msgs::PoseArray& msg){
        Lock l(pathMux);
        path.clear();
        path.reserve(msg.poses.size()/2);
        for(u16 idx=0;idx<msg.poses.size();idx+=2){
            PathSeg seg;
            auto pt1(msg.poses[idx]),pt2(msg.poses[idx+1]);
            seg.p1.x_mm=pt1.position.x;
            seg.p1.y_mm=pt1.position.y;
            seg.p1.theta_rad=pt1.orientation.z;
            seg.p2.x_mm=pt2.position.x;
            seg.p2.y_mm=pt2.position.y;
            seg.p2.theta_rad=pt2.orientation.z;
            path.push_back(seg);
        }
    }

    static void storeHmiCmd(const std_msgs::String& msg){
        Lock l(cmdMux);
        cmd=msg.data;
    }

    static void storeLocStatus(const std_msgs::Bool& msg){
        if(!msg.data){
            segNum=0;
            segLoc=START;
        }
    }
public:
    CtrlNode():
        RosNodeBase("ctrl"),
        state(HMI_FOLLOW),
        globalPosSub(n.subscribe("globalPos",100,storePos)),
        pathSub(n.subscribe("path",100,storePath)),
        ctrlCmdDataSub(n.subscribe("ctrlCmdCaptured",100,storeHmiCmd)),
        locStatSub(n.subscribe("locStatus",100,storeLocStatus)),
        ctrlPub(n.advertise<std_msgs::String>("ctrlCmd", 1000)){}
private:
    Subscriber globalPosSub;
    Subscriber pathSub;
    Subscriber ctrlCmdDataSub;
    Subscriber locStatSub;
    Publisher ctrlPub;
    CtrlState state;
    static SegRelPosState segLoc;
    static RobotPos globalPos;
    static mutex posMux;
    static string cmd;
    static mutex cmdMux;
    static vector<PathSeg> path;
    static mutex pathMux;
    static u16 segNum;
};

RobotPos CtrlNode::globalPos={0,0,M_PI_2};
mutex CtrlNode::posMux;
mutex CtrlNode::cmdMux;
string CtrlNode::cmd="";
vector<PathSeg> CtrlNode::path;
mutex CtrlNode::pathMux;
u16 CtrlNode::segNum=0;
SegRelPosState CtrlNode::segLoc=START;

#endif 