#ifndef EKF_HPP
#define EKF_HPP

#include "common.hpp"
#include "Eigen/Dense"

using namespace Eigen;

class Ekf{
public:
    void predict();
    void update();
    Point2D mean;
    Matrix2f covariance;
};

#endif