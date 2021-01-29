#ifndef LFX_NODE_HPP
#define LFX_NODE_HPP

#include "nodeBase.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>
#include <visualization_msgs/Marker.h>

using namespace std;

enum LfxFeatType{
    CORNER=0,
    EDGE
};

class LfxNode : public RosNodeBase{
    void update() override{
        RobotPos loc_pos(0,0,0);
        vector<u16> loc_scan;
        geometry_msgs::PoseArray msg;
        geometry_msgs::Pose feat;
        
        {
            Lock mux(posMux);
            loc_pos=pos;
        }
        {
            Lock mux(scanMux);
            loc_scan=scan;
        }

        extractFeats(loc_pos,loc_scan,msg.poses);

        //featPub.publish(msg);
    }

    void extractFeats(
            const RobotPos& pos,
            const vector<u16>& scan,
            vector<geometry_msgs::Pose>& feats){
        LfxFeats lfx_feats;
        vector<Point2DPolar> valid_scan;
        vector<Line2DPolar> line_params;

        // extract points
        for(u16 idx=0;idx<scan.size();idx++){
            if(scan[idx]>=LdsSensorCfg::minDepth_mm && scan[idx]<=LdsSensorCfg::maxDepth_mm){
                f32 range=scan[idx]-LdsSensorCfg::sensorOffset_mm;
                f32 ang=idx*AngConversions::degToRad;
                f32 x=pos.x_mm + (range*cos(ang));
                f32 y=pos.y_mm + (range*sin(ang));
                lfx_feats.points.push_back(Point2D(x,y));
                valid_scan.push_back(Point2DPolar(scan[idx],ang));
            }
        }

        if(valid_scan.empty())
            return;

        // extract lines
        segmentScan(valid_scan,0,valid_scan.size()-1,line_params);
        for(u16 i=0;i<line_params.size();++i){
            constructLineFeat(line_params[i],lfx_feats);
        }

        // extract corners/edges
        for(u16 i=0;i<lfx_feats.lines.size();++i){
            u16 j=(i+1)%lfx_feats.lines.size();
            auto l1=lfx_feats.lines[i];
            auto l2=lfx_feats.lines[j];
            // calc dist & delta angle between each 2 consecutive lines
            f32 dist=sqrt(pow(l1.p2.x_mm-l2.p1.x_mm,2)+pow(l1.p2.y_mm-l2.p1.y_mm,2));
            f32 ang=l2.angle-l1.angle;

            // normalize the ang
            ang=fmod(ang+(2*M_PI),2*M_PI);

            // check whether the 2 lines form corner/edge
            if(dist<100){
                if((ang>70*AngConversions::degToRad) && (ang<110*AngConversions::degToRad)){
                    lfx_feats.corners.push_back(l2.p1);
                }
                else if((ang>250*AngConversions::degToRad) && (ang<290*AngConversions::degToRad)){
                    lfx_feats.edges.push_back(l2.p1);
                }
            }
        }

        // display feats
        displayFeats(lfx_feats,pos);
    }

    inline void constructLineFeat(const Line2DPolar& line_param,LfxFeats& lfx_feats){
        f32 r=line_param.center.r_mm;
        f32 theta=line_param.center.theta_rad;
        Line2D l;
        f32 d_start=r*tan(theta-line_param.start_ang_rad);
        f32 d_end  =r*tan(line_param.end_ang_rad-theta);
        f32 ang_start= theta-M_PI_2;
        f32 ang_end= theta+M_PI_2;
        f32 center_x=pos.x_mm+r*cos(theta);
        f32 center_y=pos.y_mm+r*sin(theta);
        l.angle=theta+M_PI;
        l.angle=fmod(l.angle+(2*M_PI),2*M_PI);
        l.p1.x_mm=center_x+d_start*cos(ang_start);
        l.p1.y_mm=center_y+d_start*sin(ang_start);
        l.p2.x_mm=center_x+d_end*cos(ang_end);
        l.p2.y_mm=center_y+d_end*sin(ang_end);
        lfx_feats.lines.push_back(l);
    }

    void segmentScan(
            const vector<Point2DPolar>& scan_segment,
            const u16 start_idx,
            const u16 end_idx,
            vector<Line2DPolar>& line_params){
        u16 split_idx=0;
        f32 max_norm_dist=0;
        bool is_fitting_ok=false;
        Line2DPolar temp_line_param;

        if(end_idx-start_idx<2)
            return;

        fitLineLeastSquares(scan_segment,start_idx,end_idx,temp_line_param,max_norm_dist,split_idx); 

        if(max_norm_dist>normal_dist_threshold_mm){
            // split at slpit_idx and call segmentScan() twice 
            segmentScan(scan_segment,start_idx,split_idx,line_params);
            segmentScan(scan_segment,split_idx+1,end_idx,line_params);
            return;
        }

        line_params.push_back(temp_line_param);
    }

    void fitLineLeastSquares(
            const vector<Point2DPolar>& scan,
            const u16 start_idx,
            const u16 end_idx,
            Line2DPolar& line_params,
            f32& max_norm_dist,
            u16& max_norm_dist_idx){
        f64 first=0,second=0,third=0,fourth=0,sig=0;
        u16 M=end_idx-start_idx+1;
        
        if(M){
            for(u16 i=start_idx;i<=end_idx;++i){
                f64 p=pow(scan[i].r_mm,2);
                first+=(p*sin(2*scan[i].theta_rad));
                third+=(p*cos(2*scan[i].theta_rad));

                for(u16 j=start_idx;j<=end_idx;++j){
                    second+=(scan[i].r_mm*scan[j].r_mm*cos(scan[i].theta_rad)*sin(scan[j].theta_rad));
                    fourth+=(scan[i].r_mm*scan[j].r_mm*cos(scan[i].theta_rad+scan[j].theta_rad));
                }
            }

            line_params.center.theta_rad=0.5*atan2((second*2.0/M)-first,(fourth/M)-third);

            for(u16 i=start_idx;i<=end_idx;++i){
                sig+=scan[i].r_mm*cos(scan[i].theta_rad-line_params.center.theta_rad);
            }

            line_params.center.r_mm=sig/M;
            if(line_params.center.r_mm<0){
                line_params.center.theta_rad+=M_PI;
            }
            
            line_params.center.r_mm=fabs(line_params.center.r_mm)-LdsSensorCfg::sensorOffset_mm;
            line_params.center.theta_rad=fmod(line_params.center.theta_rad+(2*M_PI),2*M_PI);

            line_params.start_ang_rad=scan[start_idx].theta_rad;
            line_params.end_ang_rad=scan[end_idx].theta_rad;

            vector<f32> norm_dist(M,0);
            for(u16 i=start_idx;i<=end_idx;++i){
                norm_dist[i-start_idx]=scan[i].r_mm*cos(scan[i].theta_rad-line_params.center.theta_rad);
                norm_dist[i-start_idx]=fabs(fabs(norm_dist[i-start_idx])-line_params.center.r_mm);
            }

            for(u16 i=1;i<M-1;++i){
                if(norm_dist[i]>=norm_dist[i-1]&&norm_dist[i]>=norm_dist[i+1]&&norm_dist[i]>max_norm_dist){
                    max_norm_dist=norm_dist[i];
                    max_norm_dist_idx=i+start_idx;
                }
            }
        }
    }

    void displayFeats(const LfxFeats& feats,const RobotPos& pos){
        // points visualization
        visualization_msgs::Marker points;
        points.action = visualization_msgs::Marker::ADD;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.05;
        points.scale.y = 0.05;
        points.color.g = 1.0f;
        points.color.a = 1.0;
        points.pose.orientation.w = 1.0;
        points.header.frame_id = "/map";
        points.header.stamp = ros::Time::now();
        points.ns = "points";
        points.id = 0;
        
        for(u16 idx=0;idx<feats.points.size();++idx){
            geometry_msgs::Point p;
            p.x=feats.points[idx].x_mm/100;
            p.y=feats.points[idx].y_mm/100;
            p.z=0.2;
            points.points.push_back(p);
        }

        rvizPointsPub.publish(points);

        // lines visualization
        visualization_msgs::Marker lines;
        lines.action = visualization_msgs::Marker::ADD;
        lines.type = visualization_msgs::Marker::LINE_LIST;
        lines.scale.x = 0.07;
        lines.color.r = 1.0;
        lines.color.a = 1.0;
        lines.pose.orientation.w = 1.0;
        lines.header.frame_id = "/map";
        lines.header.stamp = ros::Time::now();
        lines.id = 1;
        
        for(u16 idx=0;idx<feats.lines.size();++idx){
            geometry_msgs::Point p;
            p.x=feats.lines[idx].p1.x_mm/100;
            p.y=feats.lines[idx].p1.y_mm/100;
            p.z=0.2;
            lines.points.push_back(p);
            p.x=feats.lines[idx].p2.x_mm/100;
            p.y=feats.lines[idx].p2.y_mm/100;
            lines.points.push_back(p);
        }

        rvizLinesPub.publish(lines);

        // corners visualization
        visualization_msgs::Marker corners;
        corners.action = visualization_msgs::Marker::ADD;
        corners.type = visualization_msgs::Marker::POINTS;
        corners.scale.x = 0.1;
        corners.scale.y = 0.1;
        corners.color.b = 1.0f;
        corners.color.a = 1.0;
        corners.pose.orientation.w = 1.0;
        corners.header.frame_id = "/map";
        corners.header.stamp = ros::Time::now();
        corners.ns = "corners";
        corners.id = 0;
        
        for(u16 idx=0;idx<feats.corners.size();++idx){
            geometry_msgs::Point p;
            p.x=feats.corners[idx].x_mm/100;
            p.y=feats.corners[idx].y_mm/100;
            p.z=0.2;
            corners.points.push_back(p);
        }

        rvizCornersPub.publish(corners);

        // edges visualization
        visualization_msgs::Marker edges;
        edges.action = visualization_msgs::Marker::ADD;
        edges.type = visualization_msgs::Marker::POINTS;
        edges.scale.x = 0.1;
        edges.scale.y = 0.1;
        edges.color.b = 1.0f;
        edges.color.a = 1.0;
        edges.color.r = 1.0f;
        edges.color.g = 1.0;
        edges.pose.orientation.w = 1.0;
        edges.header.frame_id = "/map";
        edges.header.stamp = ros::Time::now();
        edges.ns = "edges";
        edges.id = 0;
        
        for(u16 idx=0;idx<feats.edges.size();++idx){
            geometry_msgs::Point p;
            p.x=feats.edges[idx].x_mm/100;
            p.y=feats.edges[idx].y_mm/100;
            p.z=0.2;
            edges.points.push_back(p);
        }

        rvizEdgesPub.publish(edges);

        // pos visualization
        geometry_msgs::PoseStamped rviz_pos;
        rviz_pos.header.frame_id = "/map";
        rviz_pos.header.stamp = ros::Time::now();
        rviz_pos.pose.position.x = pos.x_mm/100;
        rviz_pos.pose.position.y = pos.y_mm/100;
        rviz_pos.pose.orientation.x=cos(pos.theta_rad/2);
        rviz_pos.pose.orientation.y=sin(pos.theta_rad/2);

        rvizPosPub.publish(rviz_pos);

        // lines vectors
        /*geometry_msgs::PoseArray pos_arr;
        pos_arr.header.frame_id = "/map";
        pos_arr.header.stamp = ros::Time::now();
        for(int i=0;i<feats.lines.size();++i){
            geometry_msgs::Pose pose;
            pose.position.x = feats.lines[i].p1.x_mm/100;
            pose.position.y = feats.lines[i].p1.y_mm/100;
            pose.orientation.x=cos(feats.lines[i].angle/2);
            pose.orientation.y=sin(feats.lines[i].angle/2);
            pos_arr.poses.push_back(pose);
        }
        
        rvizPosArrPub.publish(pos_arr);*/
    }

    static void storePos(const geometry_msgs::Pose2D& msg){
        Lock mux(posMux);
        pos.x_mm = msg.x;
        pos.y_mm = msg.y;
        pos.theta_rad =msg.theta;
    }

    static void storeScan(const sensor_msgs::LaserScan& msg){
        Lock mux(scanMux);
        for(u16 idx=0;idx<msg.ranges.size();idx++){
            scan[idx]=static_cast<u16>(msg.ranges[idx]);
        }
    }
public:
    LfxNode():
        RosNodeBase(),
        robotPosSub(n.subscribe("robotPos",100,storePos)),
        ldsScanSub(n.subscribe("ldsScan",100,storeScan)),
        rvizPointsPub(n.advertise<visualization_msgs::Marker>("rvizPoints", 1000)),
        rvizLinesPub(n.advertise<visualization_msgs::Marker>("rvizLines", 1000)),
        rvizCornersPub(n.advertise<visualization_msgs::Marker>("rvizCorners", 1000)),
        rvizEdgesPub(n.advertise<visualization_msgs::Marker>("rvizEdges", 1000)),
        rvizPosArrPub(n.advertise<geometry_msgs::PoseArray>("rvizLinesVectors", 1000)),
        rvizPosPub(n.advertise<geometry_msgs::PoseStamped>("rvizPos", 1000)){}
private:
    Subscriber robotPosSub;
    Subscriber ldsScanSub;
    Publisher featPub;
    Publisher rvizPointsPub;
    Publisher rvizLinesPub;
    Publisher rvizCornersPub;
    Publisher rvizEdgesPub;
    Publisher rvizPosPub;
    Publisher rvizPosArrPub;
    static RobotPos pos;
    static vector<u16> scan; 
    static mutex posMux;
    static mutex scanMux;
    static const u16 normal_dist_threshold_mm=25;
};

RobotPos LfxNode::pos(0,0,M_PI_2);
vector<u16> LfxNode::scan(360,0);
mutex LfxNode::posMux;
mutex LfxNode::scanMux;

#endif