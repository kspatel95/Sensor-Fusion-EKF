#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools
{
  public:
    /**
     * @brief Constructor.
     */
    Tools();

    /**
     * @brief Destructor.
     */
    virtual ~Tools();

    /**
     * @brief A helper method to calculate RMSE.
     */
    Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd>& estimations,
                                  const std::vector<Eigen::VectorXd>& ground_truth);

    /**
     * @brief A helper method to calculate Jacobians.
     */
    Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
};

#endif  // TOOLS_H_
