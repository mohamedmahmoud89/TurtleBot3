#ifndef LFX_FUNC_HPP
#define LFX_FUNC_HPP

#include "funcBase.hpp"
#include "lfxItf.hpp"

class LfxFunc : public FuncBase{
    static const u16 normal_dist_threshold_mm=25;

    inline void constructLineFeat(const Line2DPolar& line_param,const RobotPos& pos,LfxFeats& lfx_feats){
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
public:
    LfxFunc(const LfxInputData& in_ref, LfxOutputData& out_ref):FuncBase(in_ref,out_ref){}
    void exec() override{
        const LfxInputData& inData = dynamic_cast<const LfxInputData&>(this->inputs);
        LfxOutputData& outData = dynamic_cast<LfxOutputData&>(this->outputs);

        vector<Point2DPolar> valid_scan;
        vector<Line2DPolar> line_params;
        const RobotPos& pos = inData.pos;
        const vector<u16>& scan = inData.scan;
        LfxFeats& lfx_feats = outData.feats;

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
            constructLineFeat(line_params[i],pos,lfx_feats);
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
    }
};

#endif