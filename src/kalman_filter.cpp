#include <iostream>
#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &Hj_in, MatrixXd &R_laser_in,
                        MatrixXd &R_radar_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_laser_ = H_in;
    Hj_ = Hj_in;
    R_laser_ = R_laser_in;
    R_radar_ = R_radar_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd z_pred = H_laser_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_laser_.transpose();
    MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    // prediction
    VectorXd z_pred = VectorXd(3);

    z_pred(0) = sqrt(x_(0)*x_(0)+x_(1)*x_(1));
    z_pred(1) = atan2(x_(1), x_(0));

    if(z_pred(0) != 0.0){
        z_pred(2) = (x_(0)*x_(2)+x_(1)*x_(3))/z_pred(0);
    } else {
        z_pred(2) = 0;
    }

    VectorXd y = z - z_pred;
    // norm y
    while(y(1)<-M_PI){
        y(1)+=2*M_PI;
    }
    while(y(1)>M_PI){
        y(1)-=2*M_PI;
    }

    Hj_ = tools.CalculateJacobian(x_);

    MatrixXd Hjt = Hj_.transpose();
    MatrixXd S = Hj_ * P_ * Hjt + R_radar_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Hjt;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj_) * P_;
}
