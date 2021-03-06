// Copyright (C) 2005, 2007 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id$
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-09

#ifndef __MyNLP_HPP__
#define __MyNLP_HPP__

#include "IpTNLP.hpp"
#include "locomotion-planner/motion_planner_base.hpp"
#include <math.h>
#include <Eigen/Dense>
#include <iostream>

using namespace Ipopt;

/** C++ Example NLP for interfacing a problem with IPOPT.
 *  MyNLP implements a C++ example of problem 71 of the
 *  Hock-Schittkowski test suite. This example is designed to go
 *  along with the tutorial document and show how to interface
 *  with IPOPT through the TNLP interface. 
 *
 * Problem hs071 looks like this
 *
 *     min   x1*x4*(x1 + x2 + x3)  +  x3
 *     s.t.  x1*x2*x3*x4                   >=  25
 *           x1**2 + x2**2 + x3**2 + x4**2  =  40
 *           1 <=  x1,x2,x3,x4  <= 5
 *
 *     Starting point:
 *        x = (1, 5, 5, 1)
 *
 *     Optimal solution:
 *        x = (1.00000000, 4.74299963, 3.82114998, 1.37940829)
 *
 *
 */
class MyNLP : public TNLP, public MotionPlannerBase
{
public:
  /** default constructor */
  MyNLP();

  /** default destructor */
  virtual ~MyNLP();

  typedef Eigen::Vector3d Point;

  typedef  std::vector<Point> Trajectory3d;
  /**@name Overloaded from TNLP */
  //@{
  /** Method to return some info about the nlp */
  virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                            Index& nnz_h_lag, IndexStyleEnum& index_style);

  /** Method to return the bounds for my problem */
  virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                               Index m, Number* g_l, Number* g_u);

  /** Method to return the starting point for the algorithm */
  virtual bool get_starting_point(Index n, bool init_x, Number* x,
                                  bool init_z, Number* z_L, Number* z_U,
                                  Index m, bool init_lambda,
                                  Number* lambda);

  /** Method to return the objective value */
  virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value);

  /** Method to return the gradient of the objective */
  virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f);

  /** Method to return the constraint residuals */
  virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g);

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
  virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
                          Index m, Index nele_jac, Index* iRow, Index *jCol,
                          Number* values);

  /** Method to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
  virtual bool eval_h(Index n, const Number* x, bool new_x,
                      Number obj_factor, Index m, const Number* lambda,
                      bool new_lambda, Index nele_hess, Index* iRow,
                      Index* jCol, Number* values);

  //@}

  /** @name Solution Methods */
  //@{
  /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
  virtual void finalize_solution(SolverReturn status,
                                 Index n, const Number* x, const Number* z_L, const Number* z_U,
                                 Index m, const Number* g, const Number* lambda,
                                 Number obj_value,
         const IpoptData* ip_data,
         IpoptCalculatedQuantities* ip_cq);
  //@}

  Eigen::VectorXd& get_solution();

 virtual void addTrajectoryConstraint(const unsigned int & initial_index,  
                                    const Eigen::VectorXd & decision_variables, 
                                    const unsigned int & knots_number,
                                    const Eigen::VectorXd& increments, 
                                    Eigen::MatrixXd & constr_violations);

  //void getCoMTrajectory(Eigen::VectorXd& com_traj_x, Eigen::VectorXd& com_traj_y, Eigen::VectorXd& com_traj_z);
//
  //void getFootTrajectory(Eigen::VectorXd& foot_traj_x,  
  //                                            Eigen::VectorXd& foot_traj_y, 
  //                                            Eigen::VectorXd& foot_traj_z,
  //                                            unsigned int & foot_id);

private:
  /**@name Methods to block default compiler methods.
   * The compiler automatically generates the following three methods.
   *  Since the default compiler implementation is generally not what
   *  you want (for all but the most simple classes), we usually 
   *  put the declarations of these methods in the private section
   *  and never implement them. This prevents the compiler from
   *  implementing an incorrect "default" behavior without us
   *  knowing. (See Scott Meyers book, "Effective C++")
   *  
   */
  //@{
  //  MyNLP();
  MyNLP(const MyNLP&);

  MyNLP& operator=(const MyNLP&);
  //@}
  NumericalIntegrationConstraints integration_constr;
  //void setCoMTrajectory();
//
  //void setFootTrajectory(unsigned int & foot_id);

  //double b_foot[2] = {5.0,-2.0};
  //double f_camera[2] = {2.0*sqrt(2),0.0};
  double total_duration; 
  Eigen::Vector3d target_com_pos;
  Eigen::MatrixXd com_constr_violations;
  Eigen::VectorXd desired_com_lin_vel;

  unsigned int CoM_pos_index = 0;
  unsigned int CoM_orient_index = 0;
  unsigned int states_per_knot = 0;
  unsigned int knots_num = 0;
  unsigned int constraints_per_knot = 0;
  unsigned int nnz_jac_g_per_knot = 0;
  unsigned int optimization_points = 0;
  unsigned int total_states_num = 0;
  unsigned int total_constraints_num = 0;
  unsigned int first_available_index = 0;
  unsigned int CoM_variable_index;
  unsigned int feet_number = 0;

  bool enable_com_orientation = false;


};


#endif