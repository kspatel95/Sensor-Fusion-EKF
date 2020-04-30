#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools()
{
}

Tools::~Tools()
{
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd>& estimations, const vector<VectorXd>& ground_truth)
{
    // Initialise rmse vector
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // Validate non zero vector and equal length
    if (estimations.size() == 0)
    {
        std::cout << "ERROR: Estimation Vector Error for CalculateRMSE(), Possible "
                     "Empty/Zero Vector."
                  << std::endl;
        return rmse;
    }

    if (estimations.size() != ground_truth.size())
    {
        std::cout << "ERROR: Estimation Vector and Ground Truth Vector Size do Not "
                     "Match for CalculateRMSE()."
                  << std::endl;
        return rmse;
    }

    for (unsigned int i = 0; i < estimations.size(); i++)
    {
        VectorXd residual = estimations[i] - ground_truth[i];

        residual = residual.array() * residual.array();
        rmse += residual;
    }

    rmse = rmse / estimations.size();
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
    MatrixXd Hj(3, 4);

    float p_x = x_state(0);
    float p_y = x_state(1);
    float v_x = x_state(2);
    float v_y = x_state(3);

    float py2 = (p_x * p_x) + (p_y * p_y);
    float root_py2 = std::sqrt(py2);
    float pow_py2 = (py2 * root_py2);

    // Validate for zero division
    if (std::fabs(pow_py2) < 0.0001)
    {
        std::cout << "ERROR: Zero Division Error for CalculateJacobian()." << std::endl;
        return Hj;
    }

    Hj << (p_x / root_py2), (p_y / root_py2), 0, 0,
          -(p_y / py2), (p_x / py2), 0, 0,
          p_y * (v_x * p_y - v_y * p_x) / pow_py2, p_x * (v_y * p_x - v_x * p_y) / pow_py2, p_x / root_py2, p_y / root_py2;

    return Hj;
}
