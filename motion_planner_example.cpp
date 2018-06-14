// Copyright (C) 2005, 2009 International Business Machines and others.
// All Rights Reserved.
// This code is published under the Eclipse Public License.
//
// $Id$
//
// Authors:  Carl Laird, Andreas Waechter     IBM    2005-08-10

#include "IpIpoptApplication.hpp"
#include "motion_planner/motion_planner.hpp"
#include <motion_viewer/RvizMarkersPub.h>


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
  app->Options()->SetNumericValue("tol", 1e-3);
  app->Options()->SetStringValue("mu_strategy", "adaptive");
  app->Options()->SetStringValue("output_file", "ipopt.out");

  // The following overwrites the default name (ipopt.opt) of the
  // options file
  // app->Options()->SetStringValue("option_file_name", "hs071.opt");

  // Initialize the IpoptApplication and process the options
  ApplicationReturnStatus status;
  status = app->Initialize();
  if (status != Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
    return (int) status;
  }

  // Ask Ipopt to solve the problem
  status = app->OptimizeTNLP(mynlp);
  
  Eigen::VectorXd sol = mynlp->get_solution();
  //std::cout<<sol.transpose()<<std::endl;
  unsigned int knots = 9;
  unsigned int states_per_knot = 50;
  Eigen::VectorXd com_orient_x(knots), com_orient_y(knots), com_orient_z(knots);

  Eigen::VectorXd com_traj_x(knots), com_traj_y(knots), com_traj_z(knots);

  Eigen::VectorXd foot_LF_x(knots), foot_LF_y(knots), foot_LF_z(knots);
  Eigen::VectorXd foot_RF_x(knots), foot_RF_y(knots), foot_RF_z(knots);
  Eigen::VectorXd foot_LH_x(knots), foot_LH_y(knots), foot_LH_z(knots);
  Eigen::VectorXd foot_RH_x(knots), foot_RH_y(knots), foot_RH_z(knots);

  Eigen::VectorXd force_LF_x(knots), force_LF_y(knots), force_LF_z(knots);
  Eigen::VectorXd force_RF_x(knots), force_RF_y(knots), force_RF_z(knots);
  Eigen::VectorXd force_LH_x(knots), force_LH_y(knots), force_LH_z(knots);
  Eigen::VectorXd force_RH_x(knots), force_RH_y(knots), force_RH_z(knots);

  mynlp->getCoMTrajectory(com_traj_x, com_traj_y, com_traj_z);

  unsigned int LF = 0;
  unsigned int RF = 1;
  unsigned int LH = 2;
  unsigned int RH = 3;

  mynlp->getFootTrajectory(foot_LF_x, foot_LF_y, foot_LF_z, LF);//LF
  mynlp->getFootTrajectory(foot_RF_x, foot_RF_y, foot_RF_z, RF);//RF
  mynlp->getFootTrajectory(foot_LH_x, foot_LH_y, foot_LH_z, LH);//LH
  mynlp->getFootTrajectory(foot_RH_x, foot_RH_y, foot_RH_z, RH);//RH

  for(int k=0; k<knots; k++){
   // // CoM orient
   // com_orient_x(k) = sol(states_per_knot * k + 0);
   // com_orient_y(k) = sol(states_per_knot * k + 1);
   // com_orient_z(k) = sol(states_per_knot * k + 2);
   // // CoM trajectory
   // com_traj_x(k) = sol(states_per_knot * k + 3);
   // com_traj_y(k) = sol(states_per_knot * k + 4);
   // com_traj_z(k) = sol(states_per_knot * k + 5);
//
   // foot_LF_x(k) = sol(states_per_knot * k + 6);
   // foot_LF_y(k) = sol(states_per_knot * k + 7);
   // foot_LF_z(k) = sol(states_per_knot * k + 8);
//
   // foot_RF_x(k) = sol(states_per_knot * k + 9);
   // foot_RF_y(k) = sol(states_per_knot * k + 10);
   // foot_RF_z(k) = sol(states_per_knot * k + 11);
//
   // foot_LH_x(k) = sol(states_per_knot * k + 12);
   // foot_LH_y(k) = sol(states_per_knot * k + 13);
   // foot_LH_z(k) = sol(states_per_knot * k + 14);
//
   // foot_RH_x(k) = sol(states_per_knot * k + 15);
   // foot_RH_y(k) = sol(states_per_knot * k + 16);
   // foot_RH_z(k) = sol(states_per_knot * k + 17);

    force_LF_x(k) = sol(states_per_knot * k + 18);
    force_LF_y(k) = sol(states_per_knot * k + 19);
    force_LF_z(k) = sol(states_per_knot * k + 20);

    force_RF_x(k) = sol(states_per_knot * k + 21);
    force_RF_y(k) = sol(states_per_knot * k + 22);
    force_RF_z(k) = sol(states_per_knot * k + 23);

    force_LH_x(k) = sol(states_per_knot * k + 24);
    force_LH_y(k) = sol(states_per_knot * k + 25);
    force_LH_z(k) = sol(states_per_knot * k + 26);

    force_RH_x(k) = sol(states_per_knot * k + 27);
    force_RH_y(k) = sol(states_per_knot * k + 28);
    force_RH_z(k) = sol(states_per_knot * k + 29);

  }

  std::cout<<std::endl;
  std::cout<<"Solution:"<<std::endl;
  std::cout<<"CoM orient traj x: "<<com_orient_x.transpose()<<std::endl;
  std::cout<<"CoM orient traj y: "<<com_orient_y.transpose()<<std::endl;
  std::cout<<"CoM orient traj z: "<<com_orient_z.transpose()<<std::endl;
  std::cout<<std::endl;
  std::cout<<"CoM pos traj x: "<<com_traj_x.transpose()<<std::endl;
  std::cout<<"CoM pos traj y: "<<com_traj_y.transpose()<<std::endl;
  std::cout<<"CoM pos traj z: "<<com_traj_z.transpose()<<std::endl;
  std::cout<<std::endl;  
  std::cout<<"foot traj LF x: "<<foot_LF_x.transpose()<<std::endl;
  std::cout<<"foot traj LF y: "<<foot_LF_y.transpose()<<std::endl;
  std::cout<<"foot traj LF z: "<<foot_LF_z.transpose()<<std::endl;
  std::cout<<"foot traj RF x: "<<foot_RF_x.transpose()<<std::endl;
  std::cout<<"foot traj RF y: "<<foot_RF_y.transpose()<<std::endl;
  std::cout<<"foot traj RF z: "<<foot_RF_z.transpose()<<std::endl;
  std::cout<<"foot traj LH x: "<<foot_LH_x.transpose()<<std::endl;
  std::cout<<"foot traj LH y: "<<foot_LH_y.transpose()<<std::endl;
  std::cout<<"foot traj LH z: "<<foot_LH_z.transpose()<<std::endl;
  std::cout<<"foot traj RH x: "<<foot_RH_x.transpose()<<std::endl;
  std::cout<<"foot traj RH y: "<<foot_RH_y.transpose()<<std::endl;
  std::cout<<"foot traj RH z: "<<foot_RH_z.transpose()<<std::endl;
  std::cout<<std::endl;
  std::cout<<"force traj LF x: "<<force_LF_x.transpose()<<std::endl;
  std::cout<<"force traj LF y: "<<force_LF_y.transpose()<<std::endl;
  std::cout<<"force traj LF z: "<<force_LF_z.transpose()<<std::endl;
  std::cout<<"force traj RF x: "<<force_RF_x.transpose()<<std::endl;
  std::cout<<"force traj RF y: "<<force_RF_y.transpose()<<std::endl;
  std::cout<<"force traj RF z: "<<force_RF_z.transpose()<<std::endl;
  std::cout<<"force traj LH x: "<<force_LH_x.transpose()<<std::endl;
  std::cout<<"force traj LH y: "<<force_LH_y.transpose()<<std::endl;
  std::cout<<"force traj LH z: "<<force_LH_z.transpose()<<std::endl;
  std::cout<<"force traj RH x: "<<force_RH_x.transpose()<<std::endl;
  std::cout<<"force traj RH y: "<<force_RH_y.transpose()<<std::endl;
  std::cout<<"force traj RH z: "<<force_RH_z.transpose()<<std::endl;

  if (status == Solve_Succeeded) {
    std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
  }
  else {
    std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
  }

  // As the SmartPtrs go out of scope, the reference count
  // will be decremented and the objects will automatically
  // be deleted.
    while (ros::ok())
    {
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
    visual_poly_->publishEigenPath(foot_LF_x, foot_LF_y, foot_LF_z, rviz_visual_tools::RED, rviz_visual_tools::LARGE, "com_path");
    visual_poly_->publishEigenPath(foot_RF_x, foot_RF_y, foot_RF_z, rviz_visual_tools::RED, rviz_visual_tools::LARGE, "com_path");
    visual_poly_->publishEigenPath(foot_LH_x, foot_LH_y, foot_LH_z, rviz_visual_tools::RED, rviz_visual_tools::LARGE, "com_path");
    visual_poly_->publishEigenPath(foot_RH_x, foot_RH_y, foot_RH_z, rviz_visual_tools::RED, rviz_visual_tools::LARGE, "com_path");
   
    // Don't forget to trigger the publisher!
    visual_poly_->trigger();
    r.sleep();
    }

  return (int) status;
}
