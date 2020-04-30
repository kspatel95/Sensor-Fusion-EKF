#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::Init(VectorXd& x_in, MatrixXd& P_in, MatrixXd& F_in, MatrixXd& H_in, MatrixXd& R_in, MatrixXd& Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict()
{
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::CoreUpdate(MatrixXd y)
{
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd& z)
{
    VectorXd y = z - H_ * x_;

    CoreUpdate(y);
}

VectorXd KalmanFilter::RadarMeasurementToState()
{
    double rho = std::sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    double phi = std::atan2(x_(1), x_(0));
    double rho_dot;

    if (rho > 0.0001)
    {
        rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
    }
    else
    {
        std::cout << "WARN: Zero Division Error - RadarMeasurementToState() - rho > 0.0001 - Setting to Zero"
                  << std::endl;
        rho_dot = 0.0001;
    }

    VectorXd h_of_x(3);

    h_of_x << rho, phi, rho_dot;

    return h_of_x;
}

void KalmanFilter::UpdateEKF(const VectorXd& z)
{
    // Convert radar measurement to state vector.
    VectorXd hx = RadarMeasurementToState();

    VectorXd y = z - hx;

    // Make sure within the range of -pi/pi
    double pi = std::atan(1)*4;

    while(y(1) > pi){
        y(1) -= 2 * pi;
    }

    while(y(1) < -pi){
        y(1) += 2 * pi;
    }
    // Use same base actions as Update step
    CoreUpdate(y);
}
