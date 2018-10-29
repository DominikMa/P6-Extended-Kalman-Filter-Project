#include <iostream>
#include "kalman_filter.h"
#include <math.h>

#include <iostream>

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
    /**
    TODO:
      * predict the state
    */
    std::cout << "Predict:" << endl;
    x_ = F_ * x_;
    std::cout << "x = " << x_ << endl;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
    std::cout << "P = " << P_ << endl;
    std::cout << "Q = " << Q_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
    TODO:
      * update the state by using Kalman Filter equations
    */
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
    /**
    TODO:
      * update the state by using Extended Kalman Filter equations
    */
    std::cout << "x = " << x_ << endl;
    std::cout << "z = " << z << endl;

    //TODO Prediction
    VectorXd z_pred = VectorXd(3);

    z_pred(0) = sqrt(x_(0)*x_(0)+x_(1)*x_(1));

    //TODO norm
    z_pred(1) = atan2(x_(1), x_(0));
    while(z_pred(1)<-M_PI){
        z_pred(1)+=2*M_PI;
    }
    while(z_pred(1)>M_PI){
        z_pred(1)-=2*M_PI;
    }

    if(z_pred(0) != 0.0){
        z_pred(2) = (x_(0)*x_(2)+x_(1)*x_(3))/z_pred(0);
    } else {
        z_pred(2) = 0;
    }
    std::cout << "z_pred = " << z_pred << endl;


    Hj_ = tools.CalculateJacobian(x_);
    std::cout << "Hj_ = " << Hj_ << endl;
    VectorXd y = z - z_pred;
    while(y(1)<-M_PI){
        y(1)+=2*M_PI;
    }
    while(y(1)>M_PI){
        y(1)-=2*M_PI;
    }
    std::cout << "y = " << y << endl;

    MatrixXd Hjt = Hj_.transpose();
    std::cout << "Hjt = " << Hjt << endl;
    MatrixXd S = Hj_ * P_ * Hjt + R_radar_;
    std::cout << "P = " << P_ << endl;
    std::cout << "R_radar_ = " << R_radar_ << endl;
    std::cout << "S = " << S << endl;
    MatrixXd Si = S.inverse();
    std::cout << "Si = " << Si << endl;
    MatrixXd PHt = P_ * Hjt;
    std::cout << "PHt = " << PHt << endl;
    MatrixXd K = PHt * Si;
    std::cout << "K = " << K << endl;

    //new estimate

    x_ = x_ + (K * y);
    std::cout << "(K * y) = " << (K * y) << endl;
    std::cout << "new x = " << x_ << endl;
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj_) * P_;
}
