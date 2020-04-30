#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    ekf_.P_ = MatrixXd(4, 4);

    // measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    // measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    // Laser Measurement to State
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // Initialise p-matrix
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF()
{
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack)
{
    /**
     * Initialization
     */
    if (!is_initialized_)
    {
        // first measurement
        cout << "EKF: " << endl;

        // Initialise x
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            // Extract components from raw data
            double rho = measurement_pack.raw_measurements_[0];
            double phi = measurement_pack.raw_measurements_[1];
            double rho_dot = measurement_pack.raw_measurements_[2];

            // Convert to cartesian
            double x = rho * cos(phi);
            double y = rho * sin(phi);
            double vx = rho_dot * cos(phi);
            double vy = rho_dot * sin(phi);

            ekf_.x_ << x, y, vx, vy;

        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            float x = measurement_pack.raw_measurements_[0];
            float y = measurement_pack.raw_measurements_[1];

            ekf_.x_ << x, y, 1, 1;
        }

        // Set first timestamp
        previous_timestamp_ = measurement_pack.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /**
     * Prediction
     */

    double delta_t = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;

    // Reset timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // Initialise the dynamics matrix with a zero time delta to be updated after first iteration
    ekf_.F_=MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1;

    // Add time to state transition matrix
    ekf_.F_(0, 2) = delta_t;
    ekf_.F_(1, 3) = delta_t;

    // Measurement noise
    float noise_ax_ = 9.0;
    float noise_ay_ = 9.0;

    // Calculate our process noise covariance matrix
    double dt_2 = delta_t * delta_t;
    double dt_3 = dt_2 * delta_t;
    double dt_4 = dt_3 * delta_t;

    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4/4*noise_ax_, 0, dt_3/2*noise_ax_, 0,
               0, dt_4/4*noise_ay_, 0, dt_3/2*noise_ay_,
               dt_3/2*noise_ax_, 0, dt_2*noise_ax_, 0,
               0, dt_3/2*noise_ay_, 0,  dt_2*noise_ay_;

    ekf_.Predict();

    /**
     * Update
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        cout << "RADAR" << endl;
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;

        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else
    {
        cout << "LiDAR" << endl;
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;

        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "*************" << endl;
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "-------------" << endl;
    cout << "P_ = " << ekf_.P_ << endl;
    cout << "*************" << endl;
}
