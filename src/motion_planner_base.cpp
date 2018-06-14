#include "locomotion-planner/motion_planner_base.hpp"

#include <cassert>

/* Constructor. */
MotionPlannerBase::MotionPlannerBase()
{}

MotionPlannerBase::~MotionPlannerBase()
{}


void MotionPlannerBase::getTrajectory(const unsigned int & start_index,
                                              Eigen::VectorXd& com_traj_x,  
                                              Eigen::VectorXd& com_traj_y, 
                                              Eigen::VectorXd& com_traj_z){
com_traj_x.resize(com_traj_x_.rows());
com_traj_x.resize(com_traj_y_.rows());
com_traj_x.resize(com_traj_z_.rows());
com_traj_x.setZero();
com_traj_y.setZero();
com_traj_z.setZero();

for(int k=0; k<knots_num_; k++){

   // CoM trajectory
   com_traj_x(k) = solution_(states_per_knot_ * k + 0 + start_index);
   com_traj_y(k) = solution_(states_per_knot_ * k + 1 + start_index);
   com_traj_z(k) = solution_(states_per_knot_ * k + 2 + start_index);

 }

}


void MotionPlannerBase::setCoMTrajectory(const unsigned int & start_index){
  std::cout<<"set com traj"<<std::endl;
  std::cout<<knots_num_<<std::endl;
  std::cout<<states_per_knot_<<std::endl;
  com_traj_x_.resize(knots_num_);
  com_traj_y_.resize(knots_num_);
  com_traj_z_.resize(knots_num_);
  com_traj_x_.setZero();
  com_traj_y_.setZero();
  com_traj_z_.setZero();
  for(int k=0; k<knots_num_; k++){
    // CoM trajectory
    com_traj_x_(k) = solution_(states_per_knot_ * k + 0 + start_index);
    com_traj_y_(k) = solution_(states_per_knot_ * k + 1 + start_index);
    com_traj_z_(k) = solution_(states_per_knot_ * k + 2 + start_index);
  }

}

void MotionPlannerBase::getFootTrajectory(Eigen::VectorXd& foot_traj_x,  
                                              Eigen::VectorXd& foot_traj_y, 
                                              Eigen::VectorXd& foot_traj_z,
                                              unsigned int & foot_id){

 for(int k=0; k<knots_num_; k++){

   // Foot trajectory
   foot_traj_x(k) = solution_(states_per_knot_ * k + 6 + foot_id*3);
   foot_traj_y(k) = solution_(states_per_knot_ * k + 7 + foot_id*3);
   foot_traj_z(k) = solution_(states_per_knot_ * k + 8 + foot_id*3);

 }

}

void MotionPlannerBase::setSolution(const Eigen::VectorXd & sol){
  solution_ = sol;  
}

void MotionPlannerBase::setFootTrajectory(unsigned int & foot_id){
  std::cout<<"set Foot traj"<<std::endl;

  for(int k=0; k<knots_num_; k++){
    // CoM trajectory
    foot_traj_x(k) = solution_(states_per_knot_ * k + 6 + foot_id*3);
    foot_traj_y(k) = solution_(states_per_knot_ * k + 7 + foot_id*3);
    foot_traj_z(k) = solution_(states_per_knot_ * k + 8 + foot_id*3);
  }
  std::cout<<"done!"<<std::endl;

}

void MotionPlannerBase::setOptimizationPoints(const unsigned int & optimization_points){

  knots_num_ = optimization_points;

}

unsigned int MotionPlannerBase::getOptimizationPoints(){

 return knots_num_; 

}

void MotionPlannerBase::setTotalDuration(const double & duration_tot){

  duration_tot_ = duration_tot;

}

double & MotionPlannerBase::getTotalDuration(){

 return duration_tot_; 

}

void MotionPlannerBase::setInitialCoMPosition(const Eigen::Vector3d& initial_com_pos){
  
  initial_com_pos_ = initial_com_pos;

}

void MotionPlannerBase::setInitialCoMVelocity(const Eigen::Vector3d& initial_com_vel){
  
  initial_com_vel_ = initial_com_vel;

}

Eigen::Vector3d& MotionPlannerBase::getInitialCoMPosition(){
  
  return initial_com_pos_;

}

Eigen::Vector3d& MotionPlannerBase::getInitialCoMVelocity(){
  
  return initial_com_vel_;

}

void MotionPlannerBase::setInitialCoMOrientation(const Eigen::Vector3d& initial_com_orient){
  
  initial_com_orient_ = initial_com_orient;

}

void MotionPlannerBase::setInitialCoMAngVelocity(const Eigen::Vector3d& initial_com_ang_vel){
  
  initial_com_ang_vel_ = initial_com_ang_vel;

}

Eigen::Vector3d& MotionPlannerBase::getInitialCoMOrientation(){
  
  return initial_com_orient_;

}

Eigen::Vector3d& MotionPlannerBase::getInitialCoMAngVelocity(){
  
  return initial_com_ang_vel_;

}

void MotionPlannerBase::setTargetCoMPosition(const Eigen::Vector3d& target_com_pos){
  
  target_com_pos_ = target_com_pos; 

}

Eigen::Vector3d& MotionPlannerBase::getTargetCoMPosition(){
  
  return target_com_pos_;

}

double MotionPlannerBase::getQuadraticCost(const Eigen::Vector3d & final_state, 
                                                    const Eigen::Vector3d & target,
                                                    Eigen::Vector3d & gradient_f){

  gradient_f(0) = 2.0*(final_state(0)-target(0));
  gradient_f(1) = 2.0*(final_state(1)-target(1));
  gradient_f(2) = 2.0*(final_state(2)-target(2));

  return pow((final_state(0)-target(0)),2.0) + 
        pow((final_state(1)-target(1)),2.0) + 
        pow((final_state(2)-target(2)),2.0);
}

Eigen::Vector3d MotionPlannerBase::getAverageCoMLinSpeed(){
  
  Eigen::Vector3d desired_com_lin_vel;

  if(knots_num_!=0){
    desired_com_lin_vel(0) = target_com_pos_(0)/((double)knots_num_);
    desired_com_lin_vel(1) = target_com_pos_(1)/((double)knots_num_);
    desired_com_lin_vel(2) = target_com_pos_(2)/((double)knots_num_);
    std::cout<<"average desired speed is:"<<desired_com_lin_vel.transpose()<<std::endl;
  }else{
    std::cout<<"WARNING! the knots_num has not been set yet!"<<std::endl;
  }
  return desired_com_lin_vel;
}

void MotionPlannerBase::addCoMPosConstraint(){
  states_per_knot_+= 6;
  constraints_per_knot_ += 3;
  nnz_jac_g_per_knot_ += 9;
  CoM_pos_variable_index_ = first_available_index_;
  first_available_index_ += 6*knots_num_;
}

void MotionPlannerBase::addCoMOrientConstraint(){
  //enable_com_orientation = true;
  states_per_knot_+= 6;
  constraints_per_knot_ += 3;
  nnz_jac_g_per_knot_ += 9;
  CoM_orient_variable_index_ = first_available_index_;
  first_available_index_ += 6*knots_num_;
}

void MotionPlannerBase::getCoMConstraintValues(){

}

unsigned int & MotionPlannerBase::getStatesNumber(){

  return states_per_knot_;
}

unsigned int & MotionPlannerBase::getConstraintsNumber(unsigned int & nn_jac_g){

  nn_jac_g = nnz_jac_g_per_knot_;

  return constraints_per_knot_;
}