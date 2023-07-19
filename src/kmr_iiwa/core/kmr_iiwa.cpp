
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <kmr_iiwa.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <eigen3/Eigen/QR>    
#include <urdf/model.h> // urdf::Model
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <chrono> 
//#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

kmr_iiwa::kmr_iiwa()
{
  current_arm_joint_values << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  
  kdl_parser::treeFromFile("/home/medusa/kuka_ws/src/kmr_iiwa_carlos/kmr_iiwa_urdf/urdf/kmr_iiwa_carlos_kdl.urdf", tree_kmr_iiwa);
 
  if (!tree_kmr_iiwa.getChain("base_link_yaw", "adapter", kdl_chain_yaw_adapter)) 
  {
       cout<<"Could not initialize chain object";
  }
   if (!tree_kmr_iiwa.getChain("robot_odom", "adapter", kdl_chain_robot_odom_adapter)) 
  {
       cout<<"Could not initialize whole chain object";
  }
  jac_kdl_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain_yaw_adapter));
  fk_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_robot_odom_adapter));
}


Matrix<double,6,10> kmr_iiwa::jacobian_whole_KDL(Matrix<double, 10, 1> joint)
{
    KDL::Jacobian jacobianKdl_;
    jacobianKdl_.resize(8);
    KDL::JntArray qKdl_;
    qKdl_.resize(8);
    qKdl_.data << joint(2), joint(3), joint(4), joint(5), joint(6), joint(7), joint(8), joint(9);
    jac_kdl_solver->JntToJac(qKdl_, jacobianKdl_);
    Matrix <double, 6, 10> jacobian_whole;
    Matrix <double, 6, 2> linear;
    linear << 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
    jacobian_whole << linear, jacobianKdl_.data;
    return jacobian_whole;
}

Matrix<double,6,10> kmr_iiwa::jacobian_whole_local_KDL(Matrix<double, 10, 1> joint)
{
    Matrix <double, 6, 10> jacobian_whole_global, jacobian_whole_local;
    jacobian_whole_global =  jacobian_whole_KDL(joint);
    Matrix<double,4,4> current_Tworld_ee;
    KDL::JntArray q_whole_KDL;
    q_whole_KDL.resize(10);
    KDL::Frame cartpos;
    q_whole_KDL.data << joint(0), joint(1), joint(2), joint(3), joint(4), joint(5), joint(6), joint(7), joint(8), joint(9);
    KDL::Frame pose_ee;
    fk_solver->JntToCart(q_whole_KDL,pose_ee);    

    Matrix<double,3,3> current_rot_world_ee;
    Matrix<double,3,3> zeros_m;
    zeros_m << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    current_rot_world_ee << pose_ee(0, 0), pose_ee(0, 1), pose_ee(0, 2),\
                            pose_ee(1, 0), pose_ee(1, 1), pose_ee(1, 2),\
                            pose_ee(2, 0), pose_ee(2, 1), pose_ee(2, 2);
    Matrix<double,6,6> matrixRotAux; 
    matrixRotAux << current_rot_world_ee.transpose(), zeros_m,  zeros_m, current_rot_world_ee.transpose();
    jacobian_whole_local = matrixRotAux*jacobian_whole_global;
    return jacobian_whole_local;
}

MatrixXd kmr_iiwa::PseudoInverse(MatrixXd matrixInput)
{
    MatrixXd matrixInputInv;
    matrixInputInv = matrixInput.completeOrthogonalDecomposition().pseudoInverse();
    return matrixInputInv;
}


Matrix<double,10,1> kmr_iiwa::weighted_robust_mobile_axis_ik_diff(Matrix<double, 10, 1> joint, Matrix<double,6,1> desiredVel, Matrix<double,10,1> sedondaryVel, double penaltyBaseX, double penaltyBaseY, double penaltyBaseYaw)
{
    Matrix<double,10,1> vel_output_joint;
    DiagonalMatrix<double, 10, 10> WeightMatrix;
    Matrix<double,10,10> Wmatrix;
    WeightMatrix.diagonal() << penaltyBaseX, penaltyBaseY, penaltyBaseYaw, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    Wmatrix = WeightMatrix;
    Matrix<double, 10, 10> invWeightMatrix;
    invWeightMatrix = Wmatrix.inverse();
    double  epsilon = 0.045;
    double  delta_max = 0.06;
    Matrix<double, 6,10> localJacobian;
    localJacobian = jacobian_whole_local_KDL(joint);    
    Matrix<double, 10,6> localJacobianT;
    localJacobianT = localJacobian.transpose();    
    JacobiSVD<MatrixXd> svd(localJacobian);
    double lowest_sing_value = svd.singularValues()(5); 
    double delta;
    if (lowest_sing_value >= epsilon){
        delta = 0;
    }
    else{
        delta = (1 - pow((lowest_sing_value/epsilon), 2))*pow(delta_max, 2);
    }

    Matrix<double, 10, 6> component0;
    Matrix<double, 6, 6> component022; //probably fail here

    MatrixXd component02, component021;
    Matrix<double, 10, 6> component01;
    component01 = (invWeightMatrix*localJacobianT);
    
    component021 = (localJacobian*invWeightMatrix)*localJacobianT + pow(delta, 2)*MatrixXd::Identity(6, 6);
    component02 = PseudoInverse(component021); 

    component022 = component02; 
    component0 = component01*component022;

    Matrix<double, 10, 1> component1;
    component1 = component0*desiredVel;
    
    Matrix<double, 10, 1> component2;
    Matrix<double, 10, 10> component21, I10;
    component21 = (component0 * localJacobian);
    I10 = MatrixXd::Identity(10, 10);
    component2 = (I10 - component21) * sedondaryVel;

    vel_output_joint = component1 + component2;
    
    return vel_output_joint;

}

