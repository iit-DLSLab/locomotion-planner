// Copyright (C) 2005, 2009 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id$
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-10

#include "IpIpoptApplication.hpp"
#include "locomotion-planner/locomotion_planner.hpp"
#include <locomotion-viewer/RvizMarkersPub.h>
#include <locomotion-viewer/NormalDistribution.h>


using namespace Ipopt;

int main(int argc, char** argv)
{

  // For visualizing things in rviz
  ros::init(argc, argv, "triangles");
  ros::NodeHandle nh_triangles;
  ros::Rate r(1);
    
  RvizPolygonsToolsPtr visual_poly_;
  ros::NodeHandle nh;
  visual_poly_.reset(new RvizPolygonsTools("base_frame","/rviz_visual_markers", nh));
  ApplicationReturnStatus status;

  NormalDistribution nd;
  Eigen::Vector3d noise1, noise2, des_vel, com_target, com_current, com_vel_current;
  Eigen::Vector3d com_current_orient, com_current_ang_vel;
  com_target = Eigen::Vector3d(10.0,2.0,1.0);
  com_current.setZero();
  com_vel_current.setZero();
  com_current_orient.setZero();
  com_current_ang_vel.setZero();
  double time_horizon = 5.0;

  des_vel = com_target/time_horizon;
  com_vel_current = des_vel;
  double replanning_freq = 1.0;
  double replanning_dt = 1.0/replanning_freq;

  while (ros::ok())
  {
  com_current += des_vel*replanning_dt;
  com_target += des_vel*replanning_dt;
  // Create a new instance of your nlp
  //  (use a SmartPtr, not raw)
  
  SmartPtr<MyNLP> mynlp  = new MyNLP();

  // Create a new instance of IpoptApplication
  //  (use a SmartPtr, not raw)
  // We are using the factory, since this allows us to compile this
  // example with an Ipopt Windows DLL
  SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
  app->RethrowNonIpoptException(true);

  // Change some options
  // Note: The following choices are only examples, they might not be
  //       suitable for your optimization problem.
  app->Options()->SetNumericValue("tol", 1e-6);
  app->Options()->SetStringValue("mu_strategy", "adaptive");
  app->Options()->SetStringValue("output_file", "ipopt.out");

  // The following overwrites the default name (ipopt.opt) of the
  // options file
  // app->Options()->SetStringValue("option_file_name", "hs071.opt");
  noise1 = nd.generate_1D_points(3, 0.0, 0.5);
  noise2 = nd.generate_1D_points(3, 0.0, 0.05);
  unsigned int knots = 10;
  mynlp->setOptimizationPoints(knots);
  mynlp->setTotalDuration(time_horizon);
  mynlp->addCoMPosConstraint();
  mynlp->setInitialCoMPosition(com_current+noise1);
  mynlp->setInitialCoMVelocity(com_vel_current+noise2);
  
  //mynlp->addCoMOrientConstraint();
  mynlp->setInitialCoMOrientation(com_current_orient);
  mynlp->setInitialCoMAngVelocity(com_current_ang_vel);

  mynlp->setTargetCoMPosition(com_target);
  // Initialize the IpoptApplication and process the options
//  ApplicationReturnStatus status;
  status = app->Initialize();
  if (status != Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    return (int) status;
  }

  // Ask Ipopt to solve the problem
  status = app->OptimizeTNLP(mynlp);
  
  Eigen::VectorXd sol = mynlp->get_solution();
  std::cout<<sol.transpose()<<std::endl;
  unsigned int states_per_knot = 50;
  Eigen::VectorXd com_orient_x(knots), com_orient_y(knots), com_orient_z(knots);

  Eigen::VectorXd com_traj_x(knots), com_traj_y(knots), com_traj_z(knots);
  Eigen::VectorXd com_vel_x(knots), com_vel_y(knots), com_vel_z(knots);


  Eigen::VectorXd foot_LF_x(knots), foot_LF_y(knots), foot_LF_z(knots);
  Eigen::VectorXd foot_RF_x(knots), foot_RF_y(knots), foot_RF_z(knots);
  Eigen::VectorXd foot_LH_x(knots), foot_LH_y(knots), foot_LH_z(knots);
  Eigen::VectorXd foot_RH_x(knots), foot_RH_y(knots), foot_RH_z(knots);

  Eigen::VectorXd force_LF_x(knots), force_LF_y(knots), force_LF_z(knots);
  Eigen::VectorXd force_RF_x(knots), force_RF_y(knots), force_RF_z(knots);
  Eigen::VectorXd force_LH_x(knots), force_LH_y(knots), force_LH_z(knots);
  Eigen::VectorXd force_RH_x(knots), force_RH_y(knots), force_RH_z(knots);

  unsigned int com_pos_index = 0;
  unsigned int com_vel_index = 3;
  mynlp->getTrajectory(com_pos_index, com_traj_x, com_traj_y, com_traj_z);
  mynlp->getTrajectory(com_vel_index, com_vel_x, com_vel_y, com_vel_z);

  unsigned int LF = 0;
  unsigned int RF = 1;
  unsigned int LH = 2;
  unsigned int RH = 3;

  //mynlp->getFootTrajectory(foot_LF_x, foot_LF_y, foot_LF_z, LF);//LF
  //mynlp->getFootTrajectory(foot_RF_x, foot_RF_y, foot_RF_z, RF);//RF
  //mynlp->getFootTrajectory(foot_LH_x, foot_LH_y, foot_LH_z, LH);//LH
  //mynlp->getFootTrajectory(foot_RH_x, foot_RH_y, foot_RH_z, RH);//RH

  std::cout<<std::endl;
  std::cout<<"Solution:"<<std::endl;
  //std::cout<<"CoM orient traj x: "<<com_orient_x.transpose()<<std::endl;
  //std::cout<<"CoM orient traj y: "<<com_orient_y.transpose()<<std::endl;
  //std::cout<<"CoM orient traj z: "<<com_orient_z.transpose()<<std::endl;
  //std::cout<<std::endl;
  std::cout<<"CoM pos traj x: "<<com_traj_x.transpose()<<std::endl;
  std::cout<<"CoM pos traj y: "<<com_traj_y.transpose()<<std::endl;
  std::cout<<"CoM pos traj z: "<<com_traj_z.transpose()<<std::endl;
  std::cout<<std::endl;
  std::cout<<"CoM vel x: "<<com_vel_x.transpose()<<std::endl;
  std::cout<<"CoM vel y: "<<com_vel_y.transpose()<<std::endl;
  std::cout<<"CoM vel z: "<<com_vel_z.transpose()<<std::endl;

  if (status == Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
  }
  else {
    std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
  }

  Eigen::Vector3d vertex1 = Eigen::Vector3d(1.0, 2.0, 0.0);
  Eigen::Vector3d vertex2 = Eigen::Vector3d(1.0, -2.0, 0.0);
  Eigen::Vector3d vertex3 = Eigen::Vector3d(-4.0, -3.0, 0.0);
  Eigen::Vector3d vertex4 = Eigen::Vector3d(-3.0, 2.0, 0.0);
  Eigen::Vector3d vertex5 = Eigen::Vector3d(1.0, 2.0, 5.0);
  Eigen::Vector3d vertex6 = Eigen::Vector3d(2.0, -3.0, 5.0);
  Eigen::Vector3d vertex7 = Eigen::Vector3d(-3.0, -4.0, 5.0);
  Eigen::Vector3d vertex8 = Eigen::Vector3d(-3.0, 2.0, 5.0);

  // As the SmartPtrs go out of scope, the reference count
  // will be decremented and the objects will automatically
  // be deleted.

//    while (ros::ok())
//    {
    visual_poly_->deleteAllMarkers();
    Eigen::Affine3d pose;
    pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
    pose.translation() = Eigen::Vector3d( 1.0, 1.0, 1.0 ); // translate x,y,z
    
    // Publish arrow vector of pose
    ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
    //visual_poly_->publishXYPlane(pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);

    //visual_poly_->publishArrow(pose, rviz_visual_tools::GREEN, rviz_visual_tools::LARGE);
    
    //visual_poly_->publishHexahedron(pose, vertex1, vertex2, vertex3, vertex4, vertex5, vertex6, vertex7, vertex8, true);

    //visual_poly_->publishQuadrilateral(pose, vertex1, vertex2, vertex3, vertex6, true, rviz_visual_tools::GREEN);
    
    //vertex1(0)+=0.02;
    //visual_poly_->publishTriangle(pose, Eigen::Vector3d(1.0,0.0,0.0), Eigen::Vector3d(0.0,1.0,0.0), Eigen::Vector3d(0.0,0.0,1.0));

    //visual_poly_->publishQuadrilateral(pose, Eigen::Vector3d(1.0,0.0,0.0), Eigen::Vector3d(0.0,1.0,0.0),Eigen::Vector3d(-1.0,0.0,0.0), Eigen::Vector3d(0.0,-1.0,0.0));

    visual_poly_->publishEigenPath(com_traj_x, com_traj_y, com_traj_z, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE, "com_path");
    //visual_poly_->publishEigenPath(foot_LF_x, foot_LF_y, foot_LF_z, rviz_visual_tools::RED, rviz_visual_tools::LARGE, "com_path");
    //visual_poly_->publishEigenPath(foot_RF_x, foot_RF_y, foot_RF_z, rviz_visual_tools::RED, rviz_visual_tools::LARGE, "com_path");
    //visual_poly_->publishEigenPath(foot_LH_x, foot_LH_y, foot_LH_z, rviz_visual_tools::RED, rviz_visual_tools::LARGE, "com_path");
    //visual_poly_->publishEigenPath(foot_RH_x, foot_RH_y, foot_RH_z, rviz_visual_tools::RED, rviz_visual_tools::LARGE, "com_path");
   
    // Don't forget to trigger the publisher!
    visual_poly_->trigger();
    //r.sleep();
    }

  return (int) status;
}
