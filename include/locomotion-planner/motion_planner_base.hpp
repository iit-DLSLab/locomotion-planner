// Authors:  Romeo Orsolino
// Contacts: romeo.orsolino@iit.it
// Date:     01/05/2018

#ifndef __MOTION_PLANNER_BASE_HPP__
#define __MOTION_PLANNER_BASE_HPP__

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include <locomotion-planner/NumericalIntegrationConstraints.hpp>


class MotionPlannerBase
{
public:
  /** default constructor */
  MotionPlannerBase();

  /** default destructor */
  virtual ~MotionPlannerBase();

  /** @name Solution Methods */
  //@{
  /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
 //virtual void finalize_solution(SolverReturn status,
 //                               Index n, const Number* x, const Number* z_L, const Number* z_U,
 //                               Index m, const Number* g, const Number* lambda,
 //                               Number obj_value,
 //       const IpoptData* ip_data,
 //       IpoptCalculatedQuantities* ip_cq);
 ////@}

 //Eigen::VectorXd& get_solution();

  //void getTrajectory(const unsigned int & start_index, 
  //                      Eigen::VectorXd& com_traj_x, 
  //                      Eigen::VectorXd& com_traj_y, 
  //                      Eigen::VectorXd& com_traj_z);

   void getTrajectory(const unsigned int & start_index, 
                        Eigen::VectorXd& com_traj_x, 
                        Eigen::VectorXd& com_traj_y, 
                        Eigen::VectorXd& com_traj_z);

  void getFootTrajectory(Eigen::VectorXd& foot_traj_x,  
                                              Eigen::VectorXd& foot_traj_y, 
                                              Eigen::VectorXd& foot_traj_z,
                                              unsigned int & foot_id);

  void setSolution(const Eigen::VectorXd & sol);

  void setCoMTrajectory(const unsigned int & start_index);

  void setFootTrajectory(unsigned int & foot_id);

  void setOptimizationPoints(const unsigned int & optimization_points);

  unsigned int getOptimizationPoints();

  void setTotalDuration(const double & duration_tot);

  double & getTotalDuration();

  void setInitialCoMPosition(const Eigen::Vector3d& initial_com_pos);

  void setInitialCoMVelocity(const Eigen::Vector3d& initial_com_vel);

  Eigen::Vector3d& getInitialCoMPosition();

  Eigen::Vector3d& getInitialCoMVelocity();

  void setInitialCoMOrientation(const Eigen::Vector3d& initial_com_orient);

  void setInitialCoMAngVelocity(const Eigen::Vector3d& initial_com_ang_vel);

  Eigen::Vector3d& getInitialCoMOrientation();

  Eigen::Vector3d& getInitialCoMAngVelocity();

  void setTargetCoMPosition(const Eigen::Vector3d& target_com_pos);

  Eigen::Vector3d& getTargetCoMPosition();

  double getQuadraticCost(const Eigen::Vector3d& final_state, 
                                  const Eigen::Vector3d& target, 
                                  Eigen::Vector3d & gradient_f);

  Eigen::Vector3d getAverageCoMLinSpeed();

  void addCoMPosConstraint();

  void addCoMOrientConstraint();

  void setFeetNumber(unsigned int & feet_number_);

  void getCoMConstraintValues();

  unsigned int & getConstraintsNumber(unsigned int & nn_jac_g);

  unsigned int & getStatesNumber();

  void setTimeSteps(const double & duration_tot_);

  Eigen::VectorXd & getTimeSteps();

  //double total_cycle_duration = 0.0;
  Eigen::VectorXd solution_;
  Eigen::VectorXd initial_CoM_pos, initial_CoM_orient;
  Eigen::VectorXd initial_feet_pos, initial_grfs;
  Eigen::VectorXd foot_traj_x, foot_traj_y, foot_traj_z;
  Eigen::VectorXd force_traj_x, force_traj_y, force_traj_z;
  Eigen::VectorXd com_traj_x, com_traj_y, com_traj_z;

private:

  Eigen::Vector3d target_com_pos_, desired_com_lin_vel_;
  double duration_tot_ = 0.0;
  Eigen::Vector3d initial_com_pos_, initial_com_vel_, initial_com_orient_, initial_com_ang_vel_;
  Eigen::VectorXd com_traj_x_, com_traj_y_, com_traj_z_;

  unsigned int states_per_knot_ = 0;
  unsigned int knots_num_ = 0;
  unsigned int constraints_per_knot_ = 0;
  unsigned int nnz_jac_g_per_knot_ = 0;
  unsigned int total_states_num_ = 0;
  unsigned int total_constraints_num_ = 0;
  unsigned int first_available_index_ = 0;
  unsigned int CoM_pos_variable_index_;
  unsigned int CoM_orient_variable_index_;
  unsigned int feet_number_ = 0;

};


#endif