// Copyright (C) 2004, 2006 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id: MyNLP.cpp 2005 2011-06-06 12:55:16Z stefan $
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2004-11-05

#include "motion_planner/motion_planner.hpp"

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
  total_cycle_duration = 1.0; // 1 second

  // The decision variables are: com position and orientation, feet positions, contact forces and 
  // phase durations. We consider two phases of each foot (swing and stance) per cycle. 
  // Every leg's cycle has a stance and swing phase. We divide the stance phase in three subphases. Each cycle
  // has got, therefore, 4 integration steps
  // We assume here to optimize over 2 cycles, we therefore have 5 phases for each foot (20 phase duration variables)
  // The trajectory of the feet during swing is divided in 3 subintervals. In a similar way, also the 
  // contact force trajectory is divided in 3 subintervals during the stance phase. 
  // If we consider 3 stance phases and 2 swing phases this means that we will have a total of 9 optimization knots.
  // we then have: n = 3+3+12+12+20 = 50
  states_per_knot = 50;
  knots_num = 9; // initial knot + 9 optimization knots
  com_traj_x.resize(knots_num);
  com_traj_y.resize(knots_num);
  com_traj_z.resize(knots_num);

  n = states_per_knot*knots_num; // 50*9 = 450

  // The constraints are the fact that each trajectory has to belong to a polynomial curve, therefore we have 
  // one constraint per trajectory for a total of 10 trajectories (4 feet pos, 4 forces, com pos and com orient)
  constraints_per_knot = 12;
  m = constraints_per_knot*knots_num; // 12*9 = 108

  // 2 nonzeros in the jacobian (one for x1, and one for x2),
  nnz_jac_g_per_knot = 24;
  nnz_jac_g = 0; //nnz_jac_g_per_knot*knots_num;

  // and 2 nonzeros in the hessian of the lagrangian
  // (one in the hessian of the objective for x2,
  //  and one in the hessian of the constraints for x1)
  nnz_h_lag = 0;

  // We use the standard fortran index style for row/col entries
  index_style = FORTRAN_STYLE;

  return true;
}

bool MyNLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
                            Index m, Number* g_l, Number* g_u)
{
  // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
  // If desired, we could assert to make sure they are what we think they are.
  assert(n == 450);
  assert(m == 108);

  for(int k = 0; k<knots_num; k++){
    // CoM orientation in the world frame:
    x_l[k * states_per_knot + 0] = -10000.0; x_u[k * states_per_knot + 0] = +10000.0;      
    x_l[k * states_per_knot + 1] = -10000.0; x_u[k * states_per_knot + 1] = +10000.0;    
    x_l[k * states_per_knot + 2] = -10000.0; x_u[k * states_per_knot + 2] = +10000.0;    
    // CoM Position in the world frame:    
    x_l[k * states_per_knot + 3] = -10000.0; x_u[k * states_per_knot + 3] = +10000.0;   
    x_l[k * states_per_knot + 4] = -10000.0; x_u[k * states_per_knot + 4] = +10000.0;   
    x_l[k * states_per_knot + 5] = -10000.0; x_u[k * states_per_knot + 5] = +10000.0;   
    // feet positions in the world frame:
      
    //LF:   
    x_l[k * states_per_knot + 6] = -10000.0;  x_u[k * states_per_knot + 6] = +1000.0; // x 
    x_l[k * states_per_knot + 7] = -10000.0;  x_u[k * states_per_knot + 7] = +1000.0; // y 
    x_l[k * states_per_knot + 8] = -10000.0;  x_u[k * states_per_knot + 8] = +1000.0; // z 
    //RF:     
    x_l[k * states_per_knot + 9] = -10000.0;  x_u[k * states_per_knot + 9] =  +1000.0; // x  
    x_l[k * states_per_knot + 10] = -10000.0; x_u[k * states_per_knot + 10] = +1000.0; // y  
    x_l[k * states_per_knot + 11] = -10000.0; x_u[k * states_per_knot + 11] = +1000.0; // z  
    //LH:    
    x_l[k * states_per_knot + 12] = -10000.0; x_u[k * states_per_knot + 12] = +1000.0; // x  
    x_l[k * states_per_knot + 13] = -10000.0; x_u[k * states_per_knot + 13] = +1000.0; // y  
    x_l[k * states_per_knot + 14] = -10000.0; x_u[k * states_per_knot + 14] = +1000.0; // z  
    //RH:    
    x_l[k * states_per_knot + 15] = -10000.0; x_u[k * states_per_knot + 15] = +1000.0; // x  
    x_l[k * states_per_knot + 16] = -10000.0; x_u[k * states_per_knot + 16] = +1000.0; // y  
    x_l[k * states_per_knot + 17] = -10000.0; x_u[k * states_per_knot + 17] = +1000.0; // z  
      
      
    // Contact Forces in the world frame:
      
    //LF:   
    x_l[k * states_per_knot + 18] = 0.0; x_u[k * states_per_knot + 18] = +1000.0; // x 
    x_l[k * states_per_knot + 19] = 0.0; x_u[k * states_per_knot + 19] = +1000.0; // y 
    x_l[k * states_per_knot + 20] = 0.0; x_u[k * states_per_knot + 20] = +1000.0; // z 
    //RF:   
    x_l[k * states_per_knot + 21] = 0.0; x_u[k * states_per_knot + 21] = +1000.0;  // x 
    x_l[k * states_per_knot + 22] = 0.0; x_u[k * states_per_knot + 22] = +1000.0;  // y 
    x_l[k * states_per_knot + 23] = 0.0; x_u[k * states_per_knot + 23] = +1000.0;  // z 
    //LH:     
    x_l[k * states_per_knot + 24] = 0.0; x_u[k * states_per_knot + 24] = +1000.0;  // x 
    x_l[k * states_per_knot + 25] = 0.0; x_u[k * states_per_knot + 25] = +1000.0;  // y 
    x_l[k * states_per_knot + 26] = 0.0; x_u[k * states_per_knot + 26] = +1000.0;  // z 
    //RH:     
    x_l[k * states_per_knot + 27] = 0.0; x_u[k * states_per_knot + 27] = +1000.0;  // x 
    x_l[k * states_per_knot + 28] = 0.0; x_u[k * states_per_knot + 28] = +1000.0;  // y 
    x_l[k * states_per_knot + 29] = 0.0; x_u[k * states_per_knot + 29] = +1000.0;  // z 
  
      // Phase duration of each foot:
     // LF
    x_l[k * states_per_knot + 30] = 0.0; x_u[k * states_per_knot + 30] = total_cycle_duration; // stance 1
    x_l[k * states_per_knot + 31] = 0.0; x_u[k * states_per_knot + 31] = total_cycle_duration; // swing 1
    x_l[k * states_per_knot + 32] = 0.0; x_u[k * states_per_knot + 32] = total_cycle_duration; // stance 2 
    x_l[k * states_per_knot + 33] = 0.0; x_u[k * states_per_knot + 33] = total_cycle_duration; // swing 2 
    x_l[k * states_per_knot + 34] = 0.0; x_u[k * states_per_knot + 34] = total_cycle_duration; // stance 3 
        // RF    
    x_l[k * states_per_knot + 35] = 0.0; x_u[k * states_per_knot + 35] = total_cycle_duration; // stance 1
    x_l[k * states_per_knot + 36] = 0.0; x_u[k * states_per_knot + 36] = total_cycle_duration; // swing 1
    x_l[k * states_per_knot + 37] = 0.0; x_u[k * states_per_knot + 37] = total_cycle_duration; // stance 2 
    x_l[k * states_per_knot + 38] = 0.0; x_u[k * states_per_knot + 38] = total_cycle_duration; // swing 2 
    x_l[k * states_per_knot + 39] = 0.0; x_u[k * states_per_knot + 39] = total_cycle_duration; // stance 3 
        // LH    
    x_l[k * states_per_knot + 40] = 0.0; x_u[k * states_per_knot + 40] = total_cycle_duration; // stance 1
    x_l[k * states_per_knot + 41] = 0.0; x_u[k * states_per_knot + 41] = total_cycle_duration; // swing 1
    x_l[k * states_per_knot + 42] = 0.0; x_u[k * states_per_knot + 42] = total_cycle_duration; // stance 2 
    x_l[k * states_per_knot + 43] = 0.0; x_u[k * states_per_knot + 43] = total_cycle_duration; // swing 2 
    x_l[k * states_per_knot + 44] = 0.0; x_u[k * states_per_knot + 44] = total_cycle_duration; // stance 3 
         // RH     
    x_l[k * states_per_knot + 45] = 0.0; x_u[k * states_per_knot + 45] = total_cycle_duration; // stance 1
    x_l[k * states_per_knot + 46] = 0.0; x_u[k * states_per_knot + 46] = total_cycle_duration; // swing 1
    x_l[k * states_per_knot + 47] = 0.0; x_u[k * states_per_knot + 47] = total_cycle_duration; // stance 2 
    x_l[k * states_per_knot + 48] = 0.0; x_u[k * states_per_knot + 48] = total_cycle_duration; // swing 2 
    x_l[k * states_per_knot + 49] = 0.0; x_u[k * states_per_knot + 49] = total_cycle_duration; // stance 3 
  }

  for(int k = 0; k<knots_num; k++){
    // constrain the kinematic reachability of the LF foot wrt the CoM to be in a box of predefined size
    g_l[k * constraints_per_knot + 0] = 0.0;   g_u[k * constraints_per_knot + 0] = 0.3;
    g_l[k * constraints_per_knot + 1] = 0.1;   g_u[k * constraints_per_knot + 1] = 0.3;
    g_l[k * constraints_per_knot + 2] = -0.65; g_u[k * constraints_per_knot + 2] = -0.65;
    // constrain the kinematic reachability of the RF foot wrt the CoM to be in a box of predefined size
    g_l[k * constraints_per_knot + 3] = -0.3;  g_u[k * constraints_per_knot + 3] = 0.0;
    g_l[k * constraints_per_knot + 4] = -0.3;  g_u[k * constraints_per_knot + 4] = -0.1;
    g_l[k * constraints_per_knot + 5] = -0.65; g_u[k * constraints_per_knot + 5] = -0.45;
    // constrain the kinematic reachability of the LH foot wrt the CoM to be in a box of predefined size
    g_l[k * constraints_per_knot + 6] = -0.3;   g_u[k * constraints_per_knot + 6] = 0.0;
    g_l[k * constraints_per_knot + 7] = 0.1;    g_u[k * constraints_per_knot + 7] = 0.3;
    g_l[k * constraints_per_knot + 8]  = -0.65; g_u[k * constraints_per_knot + 8]  = -0.45;
    // constrain the kinematic reachability of the RH foot wrt the CoM to be in a box of predefined size
    g_l[k * constraints_per_knot + 9]  = -0.3;  g_u[k * constraints_per_knot + 9]  = 0.0;
    g_l[k * constraints_per_knot + 10] = -0.3;  g_u[k * constraints_per_knot + 10] = -0.1;
    g_l[k * constraints_per_knot + 11] = -0.65; g_u[k * constraints_per_knot + 11] = -0.45;
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
    // CoM orientation in the world frame:
    x[k * states_per_knot + 0] = 0.0;      
    x[k * states_per_knot + 1] = 0.0;    
    x[k * states_per_knot + 2] = 0.0;    
    // CoM Position in the world frame:    
    x[k * states_per_knot + 3] = 0.0;   
    x[k * states_per_knot + 4] = 0.0;   
    x[k * states_per_knot + 5] = 0.0;   
    // feet positions in the world frame:
      
    //LF:   
    x[k * states_per_knot + 6] = 0.2;  // x 
    x[k * states_per_knot + 7] = 0.2;  // y 
    x[k * states_per_knot + 8] = -0.55;  // z 
    //RF:     
    x[k * states_per_knot + 9] = 0.2;   // x  
    x[k * states_per_knot + 10] =-0.2;   // y  
    x[k * states_per_knot + 11] =-0.55; // z  
    //LH:    
    x[k * states_per_knot + 12] = -0.2;   // x  
    x[k * states_per_knot + 13] = 0.2;   // y  
    x[k * states_per_knot + 14] = -0.55; // z  
    //RH:    
    x[k * states_per_knot + 15] = -0.2;   // x  
    x[k * states_per_knot + 16] = -0.2;   // y  
    x[k * states_per_knot + 17] = -0.55; // z  
      
      
    // Contact Forces in the world frame:
      
    //LF:   
    x[k * states_per_knot + 18] = 0.0;    
    x[k * states_per_knot + 19] = 0.0;    
    x[k * states_per_knot + 20] = 0.0;    
    //RF:   
    x[k * states_per_knot + 21] = 0.0;    
    x[k * states_per_knot + 22] = 0.0;    
    x[k * states_per_knot + 23] = 0.0;    
    //LH:     
    x[k * states_per_knot + 24] = 0.0;    
    x[k * states_per_knot + 25] = 0.0;    
    x[k * states_per_knot + 26] = 0.0;    
    //RH:     
    x[k * states_per_knot + 27] = 0.0;    
    x[k * states_per_knot + 28] = 0.0;    
    x[k * states_per_knot + 29] = 0.0;    
    // Phase duration of each foot:
     // LF
    x[k * states_per_knot + 30] = 0.0; // stance 1
    x[k * states_per_knot + 31] = 0.0; // swing 1
    x[k * states_per_knot + 32] = 0.0; // stance 2 
    x[k * states_per_knot + 33] = 0.0; // swing 2 
    x[k * states_per_knot + 34] = 0.0; // stance 3 
         // RF
    x[k * states_per_knot + 35] = 0.0; // stance 1
    x[k * states_per_knot + 36] = 0.0; // swing 1
    x[k * states_per_knot + 37] = 0.0; // stance 2 
    x[k * states_per_knot + 38] = 0.0; // swing 2 
    x[k * states_per_knot + 39] = 0.0; // stance 3 
         // LH
    x[k * states_per_knot + 40] = 0.0; // stance 1
    x[k * states_per_knot + 41] = 0.0; // swing 1
    x[k * states_per_knot + 42] = 0.0; // stance 2 
    x[k * states_per_knot + 43] = 0.0; // swing 2 
    x[k * states_per_knot + 44] = 0.0; // stance 3 
         // RH
    x[k * states_per_knot + 45] = 0.0; // stance 1
    x[k * states_per_knot + 46] = 0.0; // swing 1
    x[k * states_per_knot + 47] = 0.0; // stance 2 
    x[k * states_per_knot + 48] = 0.0; // swing 2 
    x[k * states_per_knot + 49] = 0.0; // stance 3 
  }
  return true;
}

bool MyNLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
  // return the value of the objective function

  //obj_value = (-1.0 + x[4]);
  obj_value = 0.0;
  return true;
}

bool MyNLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
  // return the gradient of the objective function grad_{x} f(x)
	assert(n == 450);
  // grad_{x1} f(x): x1 is not in the objective
  //grad_f[0] = 0.0;
  //grad_f[1] = 0.0;
  //grad_f[2] = 0.0;
  //grad_f[3] = 0.0;
  //grad_f[4] = 1.0;

  // grad_{x2} f(x):
//  Number x2 = x[1];
//  grad_f[1] = -2.0*(x2 - 2.0);

  return true;
}

bool MyNLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
  // return the value of the constraints: g(x)

for(int k = 0; k<knots_num; k++){
  // constrain the kinematic reachability of the LF foot wrt the CoM to be in a box of predefined size
  g[k * constraints_per_knot + 0] = x[k * states_per_knot + 6] - x[k * states_per_knot + 3]; // foot_LF_x - com_x
  g[k * constraints_per_knot + 1] = x[k * states_per_knot + 7] - x[k * states_per_knot + 4]; // foot_LF_y - com_y
  g[k * constraints_per_knot + 2] = x[k * states_per_knot + 8] - x[k * states_per_knot + 5]; // foot_LF_z - com_z
  // constrain the kinematic reachability of the RF foot wrt the CoM to be in a box of predefined size
  g[k * constraints_per_knot + 3] = x[k * states_per_knot + 8] -  x[k * states_per_knot + 3];   // foot_RF_x - com_x
  g[k * constraints_per_knot + 4] = x[k * states_per_knot + 9] -  x[k * states_per_knot + 4];   // foot_RF_y - com_y
  g[k * constraints_per_knot + 5] = x[k * states_per_knot + 10] - x[k * states_per_knot + 5];  // foot_RF_z - com_z
  // constrain the kinematic reachability of the LH foot wrt the CoM to be in a box of predefined size
  g[k * constraints_per_knot + 6] = x[k * states_per_knot + 11] - x[k * states_per_knot + 3]; // foot_LH_x - com_x
  g[k * constraints_per_knot + 7] = x[k * states_per_knot + 12] - x[k * states_per_knot + 4]; // foot_LH_y - com_y
  g[k * constraints_per_knot + 8] = x[k * states_per_knot + 13] - x[k * states_per_knot + 5]; // foot_LH_z - com_z
  // constrain the kinematic reachability of the RH foot wrt the CoM to be in a box of predefined size
  g[k * constraints_per_knot + 9] =  x[k * states_per_knot + 14] - x[k * states_per_knot + 3]; // foot_RH_x - com_x
  g[k * constraints_per_knot + 10] = x[k * states_per_knot + 15] - x[k * states_per_knot + 4]; // foot_RH_y - com_y
  g[k * constraints_per_knot + 11] = x[k * states_per_knot + 16] - x[k * states_per_knot + 5]; // foot_RH_z - com_z
}

//  g[1] = P0[1]*x[0] + P1[1]*x[1] + P2[1]*x[2] + P3[1]*x[3] + P4[1]*x[4];
//  g[2] = x[0] + x[1] + x[2] + x[3] + x[4] - 1.0;

  return true;
}

bool MyNLP::eval_jac_g(Index n, const Number* x, bool new_x,
                       Index m, Index nele_jac, Index* iRow, Index *jCol,
                       Number* values)
{

// the jacobian of the gradient is a 108 x 450 matrix (for knots_num = 9, m = 12*9 and for 50 variables) but it's very sparse
// We can divide it in knots_num sub-blocks, where each sub-block has the 

    std::cout<<values<<std::endl;
	  if (values == NULL) {
	    // return the structure of the Jacobian
      //for(int k = 0; k<knots_num; k++){
      //  std::cout<<"define the indices of the non-zero elements of the constraints Jacobian"<<std::endl;
	    //  // elements referring to the LF leg
      //  std::cout<<"LF"<<std::endl;
	    //  iRow[k * nnz_jac_g_per_knot + 0] = 0; jCol[k * nnz_jac_g_per_knot + 0] = 3;
	    //  iRow[k * nnz_jac_g_per_knot + 1] = 0; jCol[k * nnz_jac_g_per_knot + 1] = 6;
	    //  iRow[k * nnz_jac_g_per_knot + 2] = 1; jCol[k * nnz_jac_g_per_knot + 2] = 4;
	    //  iRow[k * nnz_jac_g_per_knot + 3] = 1; jCol[k * nnz_jac_g_per_knot + 3] = 7;
	    //  iRow[k * nnz_jac_g_per_knot + 4] = 2; jCol[k * nnz_jac_g_per_knot + 4] = 5;
	    //  iRow[k * nnz_jac_g_per_knot + 5] = 2; jCol[k * nnz_jac_g_per_knot + 5] = 8;
      //  // elements referring to the RF leg
	    //  std::cout<<"RF"<<std::endl;
      //  iRow[k * nnz_jac_g_per_knot + 6] = 3;   jCol[k * nnz_jac_g_per_knot + 6] = 3;
	    //  iRow[k * nnz_jac_g_per_knot + 7] = 3;   jCol[k * nnz_jac_g_per_knot + 7] = 9;
	    //  iRow[k * nnz_jac_g_per_knot + 8] = 4;   jCol[k * nnz_jac_g_per_knot + 8] = 4;
	    //  iRow[k * nnz_jac_g_per_knot + 9] = 4;   jCol[k * nnz_jac_g_per_knot + 9] = 10;
	    //  iRow[k * nnz_jac_g_per_knot + 10] = 5;  jCol[k * nnz_jac_g_per_knot + 10] = 5;
	    //  iRow[k * nnz_jac_g_per_knot + 11] = 5;  jCol[k * nnz_jac_g_per_knot + 11] = 11;
      //  // elements referring to the LH leg
	    //  std::cout<<"LH"<<std::endl;
      //  iRow[k * nnz_jac_g_per_knot + 12] = 6; jCol[k * nnz_jac_g_per_knot + 12] = 3;
	    //  iRow[k * nnz_jac_g_per_knot + 13] = 6; jCol[k * nnz_jac_g_per_knot + 13] = 12;
	    //  iRow[k * nnz_jac_g_per_knot + 14] = 7; jCol[k * nnz_jac_g_per_knot + 14] = 4;
      //  iRow[k * nnz_jac_g_per_knot + 15] = 7; jCol[k * nnz_jac_g_per_knot + 15] = 13;
      //  iRow[k * nnz_jac_g_per_knot + 16] = 8; jCol[k * nnz_jac_g_per_knot + 16] = 5;
      //  iRow[k * nnz_jac_g_per_knot + 17] = 8; jCol[k * nnz_jac_g_per_knot + 17] = 14;
      //  // elements referring to the RH leg
      //  std::cout<<"RH"<<std::endl;
      //  iRow[k * nnz_jac_g_per_knot + 18] = 9;  jCol[k * nnz_jac_g_per_knot + 18] = 3;
      //  iRow[k * nnz_jac_g_per_knot + 19] = 9;  jCol[k * nnz_jac_g_per_knot + 19] = 15;
      //  iRow[k * nnz_jac_g_per_knot + 20] = 10; jCol[k * nnz_jac_g_per_knot + 20] = 4;
      //  iRow[k * nnz_jac_g_per_knot + 21] = 10; jCol[k * nnz_jac_g_per_knot + 21] = 16;
      //  iRow[k * nnz_jac_g_per_knot + 22] = 11; jCol[k * nnz_jac_g_per_knot + 22] = 5;
      //  iRow[k * nnz_jac_g_per_knot + 23] = 11; jCol[k * nnz_jac_g_per_knot + 23] = 17;
      //  std::cout<<"End loop"<<std::endl;
      //}
      std::cout<<"End if"<<std::endl;
	  } else {
	    // return the values of the Jacobian of the constraints
      std::cout<<"debug"<<std::endl;
      //for(int k = 0; k<knots_num; k++){
      //  std::cout<<"define the values of the non-zero elements of the constraints Jacobian"<<std::endl;
      //  // elements referring to the LF leg
	    //  values[k * nnz_jac_g_per_knot + 0] = -1.0; // 0,0
	    //  values[k * nnz_jac_g_per_knot + 1] = 1.0; // 0,1
	    //  values[k * nnz_jac_g_per_knot + 2] = -1.0; // 0,2
	    //  values[k * nnz_jac_g_per_knot + 3] = 1.0; // 0,3
	    //  values[k * nnz_jac_g_per_knot + 4] = -1.0; // 0,4
      //  values[k * nnz_jac_g_per_knot + 5] = 1.0; // 1,0
      //  //    elements referring to the RF leg
	    //  values[k * nnz_jac_g_per_knot + 6] =  -1.0; // 1,0
	    //  values[k * nnz_jac_g_per_knot + 7] =  1.0;  // 1,1
	    //  values[k * nnz_jac_g_per_knot + 8] =  -1.0; // 1,2
	    //  values[k * nnz_jac_g_per_knot + 9] =  1.0;  // 1,3
	    //  values[k * nnz_jac_g_per_knot + 10]=  -1.0; // 1,4
      //  values[k * nnz_jac_g_per_knot + 11]=  1.0;  // 1,0
      //  //     elements referring to the LH leg
	    //  values[k * nnz_jac_g_per_knot + 12] =  -1.0; // 2,0
	    //  values[k * nnz_jac_g_per_knot + 13] =  1.0;  // 2,1
	    //  values[k * nnz_jac_g_per_knot + 14] =  -1.0; // 2,2
	    //  values[k * nnz_jac_g_per_knot + 15] =  1.0;  // 2,3
      //  values[k * nnz_jac_g_per_knot + 16] =  -1.0; // 1,0
      //  values[k * nnz_jac_g_per_knot + 17] =  1.0;  // 2,4
      //  //      elements referring to the RH leg
      //  values[k * nnz_jac_g_per_knot + 18] =  -1.0; // 1,0
      //  values[k * nnz_jac_g_per_knot + 19] =  1.0;  // 2,0
      //  values[k * nnz_jac_g_per_knot + 20] =  -1.0; // 2,1
      //  values[k * nnz_jac_g_per_knot + 21] =  1.0;  // 2,2
      //  values[k * nnz_jac_g_per_knot + 22] =  -1.0; // 2,3
      //  values[k * nnz_jac_g_per_knot + 23] =  1.0;  // 2,4
      //}
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
  // here is where we would store the solution to variables, or write to a file, etc
  // so we could use the solution.

  // For this example, we write the solution to the console
  //printf("\n\nSolution of the primal variables, x\n");
  //for (Index i=0; i<n; i++) {
  //  printf("x[%d] = %e\n", i, x[i]);
  //}

  //printf("\n\nSolution of the bound multipliers, z_L and z_U\n");
  //for (Index i=0; i<n; i++) {
  //  printf("z_L[%d] = %e\n", i, z_L[i]);
  //}
  //for (Index i=0; i<n; i++) {
  //  printf("z_U[%d] = %e\n", i, z_U[i]);
  //}

  printf("\n\nObjective value\n");
  printf("f(x*) = %e\n", obj_value);

    // Eigen interfacing to raw buffers
  const Eigen::Map<const Eigen::VectorXd> solution(x, n);

  // Evaluating the solution
  solution_ = solution;

  setCoMTrajectory();
}

Eigen::VectorXd& MyNLP::get_solution()
{
  return solution_;
}

void MyNLP::getCoMTrajectory(Eigen::VectorXd& com_traj_x,  
                                              Eigen::VectorXd& com_traj_y, 
                                              Eigen::VectorXd& com_traj_z){

 for(int k=0; k<knots_num; k++){

   // CoM trajectory
   com_traj_x(k) = solution_(states_per_knot * k + 3);
   com_traj_y(k) = solution_(states_per_knot * k + 4);
   com_traj_z(k) = solution_(states_per_knot * k + 5);

 }

}


void MyNLP::setCoMTrajectory(){
  std::cout<<"set com traj"<<std::endl;
  std::cout<<knots_num<<std::endl;
  std::cout<<states_per_knot<<std::endl;

  for(int k=0; k<knots_num; k++){
    // CoM trajectory
    com_traj_x(k) = solution_(states_per_knot * k + 3);
    com_traj_y(k) = solution_(states_per_knot * k + 4);
    com_traj_z(k) = solution_(states_per_knot * k + 5);
  }
  std::cout<<"done!"<<std::endl;

}

void MyNLP::getFootTrajectory(Eigen::VectorXd& foot_traj_x,  
                                              Eigen::VectorXd& foot_traj_y, 
                                              Eigen::VectorXd& foot_traj_z,
                                              unsigned int & foot_id){

 for(int k=0; k<knots_num; k++){

   // Foot trajectory
   foot_traj_x(k) = solution_(states_per_knot * k + 6 + foot_id*3);
   foot_traj_y(k) = solution_(states_per_knot * k + 7 + foot_id*3);
   foot_traj_z(k) = solution_(states_per_knot * k + 8 + foot_id*3);

 }

}


void MyNLP::setFootTrajectory(unsigned int & foot_id){
  std::cout<<"set Foot traj"<<std::endl;

  for(int k=0; k<knots_num; k++){
    // CoM trajectory
    foot_traj_x(k) = solution_(states_per_knot * k + 6 + foot_id*3);
    foot_traj_y(k) = solution_(states_per_knot * k + 7 + foot_id*3);
    foot_traj_z(k) = solution_(states_per_knot * k + 8 + foot_id*3);
  }
  std::cout<<"done!"<<std::endl;

}
