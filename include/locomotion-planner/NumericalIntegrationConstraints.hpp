#ifndef NUMERICAL_INTEGRATION_CONSTRAINTS_H
#define NUMERICAL_INTEGRATION_CONSTRAINTS_H

#include <Eigen/Dense>

class NumericalIntegrationConstraints
{

public:

    /** @brief Constructor function */
    NumericalIntegrationConstraints();

    /** @brief Destructor function */
    virtual ~NumericalIntegrationConstraints();

    virtual void firstOrderMethod(const double & integration_step,
											const Eigen::Vector3d & x_old,
											const Eigen::Vector3d & x_dot,
											const Eigen::Vector3d & x_new,
											Eigen::Vector3d & contraint_violation);

};

#endif