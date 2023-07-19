#ifndef _KMR_IIWA_
#define _KMR_IIWA_
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <urdf/model.h> // urdf::Model
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <chrono> 
//#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class kmr_iiwa{
private:

public:
  urdf::Model model;
  KDL::Tree tree_kmr_iiwa;
  KDL::Chain kdl_chain_yaw_adapter;
  KDL::Chain kdl_chain_robot_odom_adapter;
  boost::shared_ptr<KDL::ChainJntToJacSolver> jac_kdl_solver;
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  Matrix<double,6,1> current_arm_joint_values;
  chrono::high_resolution_clock::time_point joint_states_time_received;

kmr_iiwa(); // class constructor

Matrix<double,6,10> jacobian_whole_KDL(Matrix<double, 10, 1> joint);
Matrix<double,6,10> jacobian_whole_local_KDL(Matrix<double, 10, 1> joint);
MatrixXd PseudoInverse(MatrixXd matrixInput);
Matrix<double,10,1> weighted_robust_mobile_axis_ik_diff(Matrix<double, 10, 1> joint, Matrix<double,6,1> desiredVel, Matrix<double, 10,1> sedondaryVel, double penaltyBaseX, double penaltyBaseY, double penaltyBaseYaw);

}; 
#endif 
