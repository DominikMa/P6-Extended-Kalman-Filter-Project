#include "FusionEKF.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);

    //measurement matrix
    Hj_ = MatrixXd(3, 4);
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    //state covariance matrix P
    P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    //the initial transition matrix F_
    F_ = MatrixXd(4, 4);
    F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Q_ = MatrixXd(4, 4);

    x_ = VectorXd(4);
    x_ << 1, 1, 1, 1;

    ekf_.Init(x_, P_, F_, H_laser_, Hj_, R_laser_, R_radar_, Q_);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        // first measurement
        cout << "EKF: " << endl;
        previous_timestamp_ = measurement_pack.timestamp_;
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
            cout << "x: " << measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]) << endl;
            cout << "y: " << measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]) << endl;
            ekf_.x_ << measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]),
                    measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]),
                    measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]),
                    measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            cout << "x: " << measurement_pack.raw_measurements_[0] << endl;
            cout << "y: " << measurement_pack.raw_measurements_[1] << endl;
            ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
        }

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;    //dt - expressed in seconds

    // skip second prediction is measurements arrive at nearly the same time
    if (dt > 0.01) {
        previous_timestamp_ = measurement_pack.timestamp_;

        //1. Modify the F matrix so that the time is integrated
        ekf_.F_(0, 2) = dt;
        ekf_.F_(1, 3) = dt;

        //2. Set the process covariance matrix Q
        //set the acceleration noise components
        float noise_ax = 9;
        float noise_ay = 9;
        ekf_.Q_ << pow(dt, 4) / 4 * noise_ax, 0, pow(dt, 3) / 2 * noise_ax, 0,
                0, pow(dt, 4) / 4 * noise_ay, 0, pow(dt, 3) / 2 * noise_ay,
                pow(dt, 3) / 2 * noise_ax, 0, pow(dt, 2) * noise_ax, 0,
                0, pow(dt, 3) / 2 * noise_ay, 0, pow(dt, 2) * noise_ay;

        ekf_.Predict();
    }

    /*****************************************************************************
     *  Update
     ****************************************************************************/
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        ekf_.Update(measurement_pack.raw_measurements_);
    }
}
