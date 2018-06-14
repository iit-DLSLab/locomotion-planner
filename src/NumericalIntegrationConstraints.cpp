#include<locomotion-planner/NumericalIntegrationConstraints.hpp>

NumericalIntegrationConstraints::NumericalIntegrationConstraints() {
  
}


NumericalIntegrationConstraints::~NumericalIntegrationConstraints() {

}

void NumericalIntegrationConstraints::firstOrderMethod(const double & integration_step,
											const Eigen::Vector3d & x_old,
											const Eigen::Vector3d & x_dot,
											const Eigen::Vector3d & x_new,
											Eigen::Vector3d & contraint_violation){

	for(int j=0; j<3; j++){
		contraint_violation(j) = x_new(j) - (x_old(j) + integration_step*x_dot(j));
	}

}



