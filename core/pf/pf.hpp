#ifndef PF_HPP
#define PF_HPP
#include "common.hpp"
#include <vector>
#include <cmath>

using namespace std;

struct ParticleFilterInitList{
        ParticleFilterInitList(f32 arg1,f32 arg2,f32 arg3,f32 arg4,u16 arg5,f32 arg6,f32 arg7):
                ctrl_motion_std(arg1),
                ctrl_turn_std(arg2),
                meas_x_std(arg3),
                meas_y_std(arg4),
                particles_num(arg5),
                world_x_boundary_mm(arg6),
                world_y_boundary_mm(arg7){}
        f32 ctrl_motion_std;
        f32 ctrl_turn_std;
        f32 meas_x_std;
        f32 meas_y_std;
        u16 particles_num;
        u16 world_x_boundary_mm;
        u16 world_y_boundary_mm;
};

inline f32 normal_pdf(f32 x, f32 m, f32 s)
{
    static const f32 inv_sqrt_2pi = 0.3989422804014327;
    f32 a = (x - m) / s;

    return inv_sqrt_2pi / s * std::exp(-0.5f * a * a);
}

class ParticleFilter{
public:
        ParticleFilter(
                const ParticleFilterInitList inputs);
        void predict(const WheelVelocity& wheelVelocity);
        void update(
                vector<Point2D>& feat_corners,
                vector<Point2D>& feat_edges);
        RobotPos getPosMean();
        const vector<RobotPos>& getParticles();
private:
        vector<f32> calcImpWeights(
                vector<Point2D>& feat_corners,
                vector<Point2D>& feat_edges);
        f32 calcParticleWeight(const RobotPos& pos,const vector<Point2D>&feats,const vector<Point2D>&landmarks,f32 weight);
        pair<f32,f32> computeSigmaCtrl(const WheelVelocity& wheelVelocity);
        void resample(const vector<f32>& weights);

        vector<RobotPos> particles;
        f32 ctrl_motion_std;
        f32 ctrl_turn_std;
        f32 meas_x_std;
        f32 meas_y_std;
        u16 particles_num;
        si16 resampling_debounce;
        u16 world_x_boundary_mm;
        u16 world_y_boundary_mm;
};

#endif
