#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
    TODO:
      * Calculate the RMSE here.
    */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    // ... your code here
    //TODO

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
    /**
    TODO:
      * Calculate a Jacobian here.
    */
    /*
    MatrixXd Hj(3, 4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float px2_py2 = px * px + py * py;
    float px2_py2_05 = sqrt(px2_py2);

    //check division by zero
    assert(px2_py2 != 0.0);
    if (px2_py2 == 0.0) return Hj;

    //compute the Jacobian matrix
    Hj(0, 0) = px / px2_py2_05;
    Hj(0, 1) = py / px2_py2_05;
    Hj(2, 2) = Hj(0, 0);
    Hj(2, 3) = Hj(0, 1);

    Hj(1, 0) = -py / px2_py2;
    Hj(1, 1) = px / px2_py2;

    float t = (vx * py - vy * px) / pow(px2_py2, 1.5);
    Hj(2, 0) = py * t;
    Hj(2, 1) = px * t;

    return Hj;
    */

    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    //check division by zero
    if(fabs(c1) < 0.0001){
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    //compute the Jacobian matrix
    Hj << (px/c2), (py/c2), 0, 0,
            -(py/c1), (px/c1), 0, 0,
            py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    return Hj;
}
