// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: MyNLP.cpp 2005 2011-06-06 12:55:16Z stefan $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#include "locomotion-planner/locomotion_planner.hpp"

#include <cassert>

using namespace Ipopt;

/* Constructor. */
MyNLP::MyNLP()
{}

MyNLP::~MyNLP()
{}

bool MyNLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                         Index& nnz_h_lag, IndexStyleEnum& index_style)
{
  total_duration = getTotalDuration();//1.0; // 1 second
  // The decision variables are: com position and orientation, feet positions, contact forces and 
  // phase durations. We consider two phases of each foot (swing and stance) per cycle. 
  // Every leg's cycle has a stance and swing phase. We divide the stance phase in three subphases. Each cycle
  // has got, therefore, 4 integration steps
  // We assume here to optimize over 2 cycles, we therefore have 5 phases for each foot (20 phase duration variables)
  // The trajectory of the feet during swing is divided in 3 subintervals. In a similar way, also the 
  // contact force trajectory is divided in 3 subintervals during the stance phase. 
  // If we consider 3 stance phases and 2 swing phases this means that we will have a total of 9 optimization knots.
  // we then have: n = 3+3+12+12+20 = 50
  states_per_knot = getStatesNumber();
  knots_num = getOptimizationPoints(); // initial knot + 9 optimization knots
  desired_com_lin_vel = getAverageCoMLinSpeed();

  total_states_num = states_per_knot*knots_num; // 50*9 = 450
  n = total_states_num;

  // The constraints are the fact that each trajectory has to belong to a polynomial curve, therefore we have 
  // one constraint per trajectory for a total of 10 trajectories (4 feet pos, 4 forces, com pos and com orient)
  constraints_per_knot = getConstraintsNumber(nnz_jac_g_per_knot);
  total_constraints_num =  constraints_per_knot*(knots_num-1); // 0*6 = 
  m = total_constraints_num;

  // 2 nonzeros in the jacobian (one for x1, and one for x2),
  //nnz_jac_g_per_knot = 9; //total_states_num*total_constraints_num;
  nnz_jac_g = nnz_jac_g_per_knot*(knots_num-1);

  // and 2 nonzeros in the hessian of the lagrangian
  // (one in the hessian of the objective for x2,
  //  and one in the hessian of the constraints for x1)
  nnz_h_lag = 0;

  // We use the standard fortran index style for row/col entries
  index_style = C_STYLE;//FORTRAN_STYLE;

  return true;
}

bool MyNLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
                            Index m, Number* g_l, Number* g_u)
{
  // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
  // If desired, we could assert to make sure they are what we think they are.
  assert(n == total_states_num);
  assert(m == total_constraints_num);


    //set values of the initial way
  Eigen::Vector3d initial_com_pos, initial_com_vel;
  initial_com_pos.setZero();
  initial_com_vel.setZero();
  initial_com_pos = getInitialCoMPosition();
  initial_com_vel = getInitialCoMVelocity();
  x_l[CoM_pos_index + 0] = initial_com_pos(0); x_u[CoM_pos_index + 0] = initial_com_pos(0);
  x_l[CoM_pos_index + 1] = initial_com_pos(1); x_u[CoM_pos_index + 1] = initial_com_pos(1);
  x_l[CoM_pos_index + 2] = initial_com_pos(2); x_u[CoM_pos_index + 2] = initial_com_pos(2);     
  x_l[CoM_pos_index + 3] = initial_com_vel(0); x_u[CoM_pos_index + 3] = initial_com_vel(0);
  x_l[CoM_pos_index + 4] = initial_com_vel(1); x_u[CoM_pos_index + 4] = initial_com_vel(1);
  x_l[5] = initial_com_vel(2); x_u[5] = initial_com_vel(2); 


  for(int k = 1; k<knots_num; k++){
    //CoM Position in the world frame:  

    for(int j = CoM_pos_index; j<states_per_knot; j++){ 
      x_l[k * states_per_knot + j] = -100.0; x_u[k * states_per_knot + j] = +100.0;      
    }   
  }

  for(int k = CoM_pos_index; k<knots_num-1; k++){
    g_l[k * constraints_per_knot + 0] = 0.0; g_u[k * constraints_per_knot + 0] = 0.0;
    g_l[k * constraints_per_knot + 1] = 0.0; g_u[k * constraints_per_knot + 1] = 0.0;
    g_l[k * constraints_per_knot + 2] = 0.0; g_u[k * constraints_per_knot + 2] = 0.0;
  }

// set up CoM orientation
  if(enable_com_orientation){
    Eigen::Vector3d initial_com_orient, initial_com_ang_vel;
    initial_com_orient.setZero();
    initial_com_vel.setZero();
    initial_com_orient = getInitialCoMOrientation();
    initial_com_vel = getInitialCoMAngVelocity();
    CoM_orient_index = knots_num*6;
    x_l[CoM_orient_index + 0] = initial_com_orient(0);  x_u[CoM_orient_index + 0] = initial_com_orient(0);
    x_l[CoM_orient_index + 1] = initial_com_orient(1);  x_u[CoM_orient_index + 1] = initial_com_orient(1);
    x_l[CoM_orient_index + 2] = initial_com_orient(2);  x_u[CoM_orient_index + 2] = initial_com_orient(2);     
    x_l[CoM_orient_index + 3] = initial_com_ang_vel(0); x_u[CoM_orient_index + 3] = initial_com_ang_vel(0);
    x_l[CoM_orient_index + 4] = initial_com_ang_vel(1); x_u[CoM_orient_index + 4] = initial_com_ang_vel(1);
    x_l[CoM_orient_index + 5] = initial_com_ang_vel(2); x_u[CoM_orient_index + 5] = initial_com_ang_vel(2); 
  
    for(int k = 1; k<knots_num; k++){
      //CoM Position in the world frame:  
      for(int j = CoM_orient_index; j<states_per_knot; j++){ 
        x_l[k * states_per_knot + j] = -100.0; x_u[k * states_per_knot + j] = +100.0;      
      }  
  
    }
  
    for(int k = CoM_orient_index; k<knots_num-1; k++){
      g_l[k * constraints_per_knot + 0] = 0.0; g_u[k * constraints_per_knot + 0] = 0.0;
      g_l[k * constraints_per_knot + 1] = 0.0; g_u[k * constraints_per_knot + 1] = 0.0;
      g_l[k * constraints_per_knot + 2] = 0.0; g_u[k * constraints_per_knot + 2] = 0.0;
    }
  }

  return true;
}

//bool MyNLP::get_scaling_parameters(Number& obj_scaling,
//									bool& use_x_scaling, Index n,
//									Number* x_scaling,
//									bool& use_g_scaling, Index m,
//									Number* g_scaling)
//{
//	obj_scaling = 1.0;
//	}

bool MyNLP::get_starting_point(Index n, bool init_x, Number* x,
                               bool init_z, Number* z_L, Number* z_U,
                               Index m, bool init_lambda,
                               Number* lambda)
{
  // Here, we assume we only have starting values for x, if you code
  // your own NLP, you can provide starting values for the others if
  // you wish.
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);

  for(int k = 0; k<knots_num; k++){

    for(int j = 0; j<states_per_knot; j++){
      x[k * states_per_knot + j] = 0.0;      
    }

  }
  return true;
}

bool MyNLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
  // return the value of the objective function
  const Eigen::Map<const Eigen::VectorXd> decision_vars(x, n);

  //obj_value = (-1.0 + x[4]);
  //obj_value = 0.0;
  target_com_pos = getTargetCoMPosition();
  //std::cout<<target_com_pos<<std::endl;
  Eigen::Vector3d final_com_state, com_lin_vel, grad_f;
  final_com_state(0) = decision_vars((knots_num-1)*6+0);
  final_com_state(1) = decision_vars((knots_num-1)*6+1);
  final_com_state(2) = decision_vars((knots_num-1)*6+2);
  double cost_temp = 0.0;
  for(int k = 0; k<knots_num; k++){
    com_lin_vel(0) = decision_vars(k * states_per_knot + 3);
    com_lin_vel(1) = decision_vars(k * states_per_knot + 4);
    com_lin_vel(2) = decision_vars(k * states_per_knot + 5);
    cost_temp += getQuadraticCost(com_lin_vel, desired_com_lin_vel, grad_f);
  }
  obj_value = cost_temp + getQuadraticCost(final_com_state, target_com_pos, grad_f);
  //std::cout<<obj_value<<std::endl;

  return true;
}

bool MyNLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  // return the gradient of the objective function grad_{x} f(x)
	assert(n == total_states_num);
  const Eigen::Map<const Eigen::VectorXd> decision_vars(x, n);
  Eigen::Vector3d final_com_state;
  final_com_state(0) = decision_vars(n-6);
  final_com_state(1) = decision_vars(n-5);
  final_com_state(2) = decision_vars(n-4);
  target_com_pos = getTargetCoMPosition();
  Eigen::Vector3d com_cost_gradient;
  getQuadraticCost(final_com_state, target_com_pos, com_cost_gradient);

  for(int k = 0; k<knots_num; k++){
    Eigen::Vector3d com_lin_vel, com_lin_vel_cost_gradient;
    com_lin_vel(0) = decision_vars(k * states_per_knot + 3);
    com_lin_vel(1) = decision_vars(k * states_per_knot + 4);
    com_lin_vel(2) = decision_vars(k * states_per_knot + 5);
    getQuadraticCost(com_lin_vel, desired_com_lin_vel, com_lin_vel_cost_gradient);
    //std::cout<<com_lin_vel_cost_gradient<<std::endl;
    if(k == knots_num-1){
        grad_f[k * states_per_knot + 0] = com_cost_gradient(0);
        grad_f[k * states_per_knot + 1] = com_cost_gradient(1);
        grad_f[k * states_per_knot + 2] = com_cost_gradient(2);
        grad_f[k * states_per_knot + 3] = com_lin_vel_cost_gradient(0);
        grad_f[k * states_per_knot + 4] = com_lin_vel_cost_gradient(1);
        grad_f[k * states_per_knot + 5] = com_lin_vel_cost_gradient(2);
     }else{
        grad_f[k * states_per_knot + 0] = 0.0;
        grad_f[k * states_per_knot + 1] = 0.0;
        grad_f[k * states_per_knot + 2] = 0.0;
        grad_f[k * states_per_knot + 3] = com_lin_vel_cost_gradient(0);
        grad_f[k * states_per_knot + 4] = com_lin_vel_cost_gradient(1);
        grad_f[k * states_per_knot + 5] = com_lin_vel_cost_gradient(2);
    }
    //std::cout<<grad_f[k * states_per_knot + 0]<<" "<<grad_f[k * states_per_knot + 1]<<" "<<grad_f[k * states_per_knot + 2]<<" "
    //<<grad_f[k * states_per_knot + 3]<<" "<<grad_f[k * states_per_knot + 4]<<" "<<grad_f[k * states_per_knot + 5]<<" ";
}

  return true;
}

bool MyNLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
  // return the value of the constraints: g(x)
const Eigen::Map<const Eigen::VectorXd> decision_vars(x, n);
const Eigen::Map<const Eigen::VectorXd> constraints(g, m);

Eigen::Vector3d next_com_pos, com_pos, com_lin_vel, constr_violation;
Eigen::VectorXd delta_t(knots_num); 
delta_t.setZero();

for(int k = 0; k<knots_num-1; k++){
  double kd = (double)k;
  double incr = getTotalDuration() / (double)knots_num;
  delta_t(k) = incr;
}

//unsigned int ind = 0;
//std::cout<<"i am here"<<std::endl;
//addTrajectoryConstraint(ind , decision_vars, knots_num, delta_t, com_constr_violations);
//std::cout<<com_constr_violations<<std::endl;
//for(int k = 0; k<knots_num-1; k++){
//  for(int j = 0; j<constraints_per_knot; j++){
//    g[k * constraints_per_knot + j] = com_constr_violations(j,k);
//    std::cout<<g[k * constraints_per_knot + j]<<std::endl;
//  }
//}

for(int k = 0; k<knots_num-1; k++){
  com_pos(0) = decision_vars(k * states_per_knot + 0);
  com_pos(1) = decision_vars(k * states_per_knot + 1);
  com_pos(2) = decision_vars(k * states_per_knot + 2);
  com_lin_vel(0) = decision_vars(k * states_per_knot + 3);
  com_lin_vel(1) = decision_vars(k * states_per_knot + 4);
  com_lin_vel(2) = decision_vars(k * states_per_knot + 5);
  next_com_pos(0) = decision_vars((k+1) * states_per_knot + 0);
  next_com_pos(1) = decision_vars((k+1) * states_per_knot + 1);
  next_com_pos(2) = decision_vars((k+1) * states_per_knot + 2);
  integration_constr.firstOrderMethod(delta_t(k), com_pos, com_lin_vel, next_com_pos, constr_violation);
  g[k * constraints_per_knot + 0] = constr_violation(0);
  g[k * constraints_per_knot + 1] = constr_violation(1);
  g[k * constraints_per_knot + 2] = constr_violation(2);
}
  //std::cout<<"finish"<<std::endl;
  return true;
}

bool MyNLP::eval_jac_g(Index n, const Number* x, bool new_x,
                       Index m, Index nele_jac, Index* iRow, Index *jCol,
                       Number* values)
{

Eigen::VectorXd delta_t(knots_num); 
delta_t.setZero();
for(int k = 0; k < knots_num-1; k++){
  double kd = (double)k;
  double incr = getTotalDuration() / (double)knots_num;
  delta_t(k) = kd * incr + 1.0;
}
//std::cout<<delta_t.transpose()<<std::endl;

  for(int k = 0; k<knots_num-1; k++){
    if (values == NULL) {
      // return the structure of the Jacobian
  
      // this particular Jacobian is dense
      iRow[k*nnz_jac_g_per_knot + 0] = 0 + k * 3; jCol[k*nnz_jac_g_per_knot + 0] = 0 + k * 6;
      iRow[k*nnz_jac_g_per_knot + 1] = 0 + k * 3; jCol[k*nnz_jac_g_per_knot + 1] = 3 + k * 6;
      iRow[k*nnz_jac_g_per_knot + 2] = 0 + k * 3; jCol[k*nnz_jac_g_per_knot + 2] = 6 + k * 6;
      
      iRow[k*nnz_jac_g_per_knot + 3] = 1 + k * 3; jCol[k*nnz_jac_g_per_knot + 3] = 1 + k * 6;
      iRow[k*nnz_jac_g_per_knot + 4] = 1 + k * 3; jCol[k*nnz_jac_g_per_knot + 4] = 4 + k * 6;
      iRow[k*nnz_jac_g_per_knot + 5] = 1 + k * 3; jCol[k*nnz_jac_g_per_knot + 5] = 7 + k * 6;
  
      iRow[k*nnz_jac_g_per_knot + 6] = 2 + k * 3; jCol[k*nnz_jac_g_per_knot + 6] = 2 + k * 6;
      iRow[k*nnz_jac_g_per_knot + 7] = 2 + k * 3; jCol[k*nnz_jac_g_per_knot + 7] = 5 + k * 6;
      iRow[k*nnz_jac_g_per_knot + 8] = 2 + k * 3; jCol[k*nnz_jac_g_per_knot + 8] = 8 + k * 6;
    }
    else {
      // return the values of the Jacobian of the constraints
      
      values[k*nnz_jac_g_per_knot + 0] = -1.0;
      values[k*nnz_jac_g_per_knot + 1] = -delta_t(k);
      values[k*nnz_jac_g_per_knot + 2] = 1.0;
  
      values[k*nnz_jac_g_per_knot + 3] = -1.0;
      values[k*nnz_jac_g_per_knot + 4] = -delta_t(k);
      values[k*nnz_jac_g_per_knot + 5] = 1.0;
  
      values[k*nnz_jac_g_per_knot + 6] = -1.0;
      values[k*nnz_jac_g_per_knot + 7] = -delta_t(k);
      values[k*nnz_jac_g_per_knot + 8] = 1.0;
    }
  }

	  return true;
}

bool MyNLP::eval_h(Index n, const Number* x, bool new_x,
                   Number obj_factor, Index m, const Number* lambda,
                   bool new_lambda, Index nele_hess, Index* iRow,
                   Index* jCol, Number* values)
{


  return false;
}

void MyNLP::finalize_solution(SolverReturn status,
                              Index n, const Number* x, const Number* z_L, const Number* z_U,
                              Index m, const Number* g, const Number* lambda,
                              Number obj_value,
			      const IpoptData* ip_data,
			      IpoptCalculatedQuantities* ip_cq)
{

  printf("\n\nObjective value\n");
  printf("f(x*) = %e\n", obj_value);

    // Eigen interfacing to raw buffers
  const Eigen::Map<const Eigen::VectorXd> solution(x, n);

  // Evaluating the solution
  setSolution(solution);
  
  unsigned int com_pos_index = 0;
  setCoMTrajectory(com_pos_index);
}

Eigen::VectorXd& MyNLP::get_solution()
{
  return solution_;
}

void MyNLP::addTrajectoryConstraint(const unsigned int & initial_index,  
                                    const Eigen::VectorXd & decision_variables, 
                                    const unsigned int & knots_number,
                                    const Eigen::VectorXd& increments, 
                                    Eigen::MatrixXd & constr_violations){

  Eigen::Vector3d next_com_pos, com_pos, com_lin_vel, constr_viol;
  constr_violations.resize(3,knots_number);
  constr_violations.setZero();
  
  
  for(int k = 0; k<knots_number-1; k++){
    com_pos(0) = decision_variables(k * 6 + 0);
    com_pos(1) = decision_variables(k * 6 + 1);
    com_pos(2) = decision_variables(k * 6 + 2);
    com_lin_vel(0) = decision_variables(k * 6 + 3);
    com_lin_vel(1) = decision_variables(k * 6 + 4);
    com_lin_vel(2) = decision_variables(k * 6 + 5);
    next_com_pos(0) = decision_variables((k+1) * 6 + 0);
    next_com_pos(1) = decision_variables((k+1) * 6 + 1);
    next_com_pos(2) = decision_variables((k+1) * 6 + 2);
    integration_constr.firstOrderMethod(increments(k), com_pos, com_lin_vel, next_com_pos, constr_viol);
    constr_violations(0,k * 6 + 0) = constr_viol(0);
    constr_violations(1,k * 6 + 1) = constr_viol(1);
    constr_violations(2,k * 6 + 2) = constr_viol(2);
  }
}
