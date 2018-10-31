#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // check the validity of the inputs
    if (estimations.empty()) return rmse;
    if (estimations.size() != ground_truth.size()) return rmse;

    //accumulate squared residuals
    for (int i = 0; i < estimations.size(); ++i) {
        VectorXd d = estimations[i] - ground_truth[i];
        rmse += d.cwiseProduct(d);
    }

    //calculate the mean
    rmse /= estimations.size();

    //calculate the squared root
    rmse = rmse.cwiseSqrt();

    //return the result
    return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    MatrixXd Hj(3, 4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float px2_py2 = px * px + py * py;
    float px2_py2_05 = sqrt(px2_py2);
    float px2_py2_1_5 = px2_py2 * px2_py2_05;

    //check division by zero
    assert(px2_py2 != 0.0);
    if (px2_py2 == 0.0) return Hj;

    //compute the Jacobian matrix
    Hj(0, 2) = 0;
    Hj(0, 3) = 0;
    Hj(1, 2) = 0;
    Hj(1, 3) = 0;

    Hj(0, 0) = px / px2_py2_05;
    Hj(0, 1) = py / px2_py2_05;
    Hj(2, 2) = Hj(0, 0);
    Hj(2, 3) = Hj(0, 1);

    Hj(1, 0) = -(py / px2_py2);
    Hj(1, 1) = px / px2_py2;

    Hj(2, 0) = py * (vx * py - vy * px) / px2_py2_1_5;
    Hj(2, 1) = px * (vy * px - vx * py) / px2_py2_1_5;

    return Hj;
}
