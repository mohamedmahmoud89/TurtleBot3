#include "pf.hpp"
#include "odom.hpp"
#include "feat.hpp"
#include <random>
#include <algorithm>

struct ParticleFilterCfg{
    static const u16 numParticles{500};
};

ParticleFilter::ParticleFilter(const ParticleFilterInitList inputs):
        ctrl_motion_std(inputs.ctrl_motion_std),
        ctrl_turn_std(inputs.ctrl_turn_std),
        meas_x_std(inputs.meas_x_std),
        meas_y_std(inputs.meas_y_std),
        particles_num(inputs.particles_num),
        world_x_boundary_mm(inputs.world_x_boundary_mm),
        world_y_boundary_mm(inputs.world_y_boundary_mm){
        // sample the particles
        particles.reserve(particles_num);
        for(u16 i=0;i<particles_num;++i){
            particles.push_back(RobotPos(rand() % world_x_boundary_mm,rand() % world_y_boundary_mm,(rand() % 360)*AngConversions::degToRad));
        }
    }

void ParticleFilter::predict(const WheelVelocity& wheelVelocity){
    pair<f32,f32> ctrl_std(computeSigmaCtrl(wheelVelocity));
    random_device dev;
    default_random_engine gen(dev());
    normal_distribution<f32> dist_l(
                wheelVelocity.left_rpm,ctrl_std.second);
    normal_distribution<f32> dist_r(
                wheelVelocity.right_rpm,ctrl_std.first);

    for(auto&i:particles){
        f32 l(dist_l(gen));
        f32 r(dist_r(gen));
        WheelVelocity sampled_vel(l,r);
        odomEstimatePos(i,sampled_vel);
    }
}

void ParticleFilter::update(
    vector<Point2D>& feat_corners,
    vector<Point2D>& feat_edges){
    auto weights(calcImpWeights(feat_corners,feat_edges));
    resample(weights);
}

RobotPos ParticleFilter::getPosMean(){
    // density estimation using mean particle
    f32 mean_x(0),mean_y(0),mean_cos(0),mean_sin(0);

    // particles
    for(auto& i:particles){
        mean_x+=i.x_mm;
        mean_y+=i.y_mm;
        mean_cos=cos(i.theta_rad);
        mean_sin=sin(i.theta_rad);
    }

    // mean
    auto sz(particles.size());
    mean_sin/=sz;
    mean_cos/=sz;
    return RobotPos(mean_x/sz,mean_y/sz,atan2(mean_sin,mean_cos));
}

f32 ParticleFilter::calcParticleWeight(const RobotPos& pos,vector<Point2D>&feats,const vector<Point2D>&landmarks,f32 weight){
    for(auto&feat:feats){
        // transform the features to world coords
        // based on the particle pos
        calcFeatGlobalPos(feat,pos);
    }
    unordered_map<u16,u16> associations=featAssociate(feats,landmarks);
    for(auto&assoc:associations){
        u16 landmars_idx=assoc.first;
        u16 feat_idx=assoc.second;

        // sample pdf using coords diff
        f32 delta_x(landmarks[landmars_idx].x_mm-feats[feat_idx].x_mm);
        f32 delta_y(landmarks[landmars_idx].y_mm-feats[feat_idx].y_mm);

        f32 nd1(normal_pdf(delta_x,0,meas_x_std));
        f32 nd2(normal_pdf(delta_y,0,meas_y_std));
        weight*=(nd1*nd2);
    }

    return weight;
}

vector<f32> ParticleFilter::calcImpWeights(
    vector<Point2D>& feat_corners,
    vector<Point2D>& feat_edges){
    vector<f32>ret;
    const f32 max_ref_dist(numeric_limits<f32>::max());
    for(auto& p:particles){
        f32 weight=1;
        weight=calcParticleWeight(p,feat_corners,Landmarks::corners,weight);
        weight=calcParticleWeight(p,feat_edges,Landmarks::edges,weight);
        ret.push_back(weight);
    }
    return ret;
}

pair<f32,f32> ParticleFilter::computeSigmaCtrl(const WheelVelocity& wheelVelocity){
    f32 r(odomCalcDrivenDist(wheelVelocity.right_rpm));
    f32 l(odomCalcDrivenDist(wheelVelocity.left_rpm));
    f32 motion_var_l(pow(ctrl_motion_std*l,2)+
                        pow(ctrl_turn_std*(l-r),2));
    f32 motion_var_r(pow(ctrl_motion_std*r,2)+
                        pow(ctrl_turn_std*(l-r),2));
    return make_pair<f32,f32>(
                    sqrt(motion_var_r),
                    sqrt(motion_var_l));
}

void ParticleFilter::resample(const vector<f32>& weights){
    vector<RobotPos>resampled;
    u16 N(particles.size());
    u16 idx(rand()%N);
    f32 beta(0);
    f32 mw(*max_element(weights.begin(),weights.end()));

    for(u16 i=0;i<N;++i){
        f32 random_num(static_cast<f32>(rand())/RAND_MAX);
        beta=(random_num*2*mw);
        while(beta>weights[idx]){
            beta-=weights[idx];
            idx=(idx+1)%N;
        }
        // sample new particles around the one of chosen index
        /*random_device rd;
        default_random_engine gen(rd());
        normal_distribution<f32> dist_x(particles[idx]->X(),20.0);
        normal_distribution<f32> dist_y(particles[idx]->Y(),20.0);
        normal_distribution<f32> dist_yaw(particles[idx]->Yaw(),((2.0 / 180.0) * M_PI));
        f32 x(dist_x(gen));
        f32 y(dist_y(gen));
        f32 yaw(dist_yaw(gen));*/
        resampled.push_back(particles[idx]);
    }
    particles.clear();
    particles=resampled;
}