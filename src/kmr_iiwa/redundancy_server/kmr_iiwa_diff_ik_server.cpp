
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <fstream> 
#include <chrono> 
#include <sensor_msgs/JointState.h>
#include <kmr_iiwa.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <tf/tf.h>
#include <urdf/model.h> // urdf::Model
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;
using namespace Eigen;

kmr_iiwa robot_lib;

Matrix<double, 10,1> secondary_vel;
Matrix<double, 6,1> primary_vel;



void primary_secondary_vel_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if ((int)msg->data.size() == 16) //Primary 6 + Secondary 10 = 16
    {
      primary_vel << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
      secondary_vel << msg->data[6], msg->data[7], msg->data[8], msg->data[9], msg->data[10], msg->data[11], msg->data[12], msg->data[13], msg->data[14], msg->data[15];
    }
}


int main(int argc, char ** argv)
{

    //Initialize and start the node
    ros::init(argc, argv, "kuka_kmr_redudancy_cpp");
    ros::NodeHandle n;
    ros::Subscriber sub_primary_secondary_vel = n.subscribe("primary_secondary_velocity", 1000, primary_secondary_vel_callback);
    ros::Publisher kmr_iiwa_pub= n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

    Matrix<double, 10,1> joint_whole, joint_vel;
    joint_whole << 0.0, 0.0, 0.0, 1.7, 1.0, 0.1, -1.6, -0.2, -1.2, 0.0;
    secondary_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    primary_vel << 0.01, 0.0, 0.0, 0.0, 0.0, 0.0;

    double incT = 0.0;      
    auto start = std::chrono::high_resolution_clock::now();
    auto finish = std::chrono::high_resolution_clock::now();
    
    double penaltyBaseX =10000000000000.0;
    double penaltyBaseY =10000000000000.0;
    double penaltyBaseYaw =10000000000000.0;





     while(ros::ok)
     {          
      //joint_vel = robot_lib.weighted_robust_mobile_axis_ik_diff(joint_whole, robot_io.primary_vel, robot_io.secondary_vel, penaltyBaseX, penaltyBaseY, penaltyBaseYaw);
      joint_vel = robot_lib.weighted_robust_mobile_axis_ik_diff(joint_whole, primary_vel, secondary_vel, penaltyBaseX, penaltyBaseY, penaltyBaseYaw);
      //INTEGRATOR
      finish = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = finish - start;
      joint_whole = joint_whole + joint_vel*elapsed.count();
      start = std::chrono::high_resolution_clock::now();
        
      sensor_msgs::JointState msg_joint;
      msg_joint.header.stamp = ros::Time::now(); 
      msg_joint.name.push_back("X");
      msg_joint.name.push_back("Y");
      msg_joint.name.push_back("Yaw");
      msg_joint.name.push_back("joint_a1");
      msg_joint.name.push_back("joint_a2");
      msg_joint.name.push_back("joint_a3");
      msg_joint.name.push_back("joint_a4");
      msg_joint.name.push_back("joint_a5");
      msg_joint.name.push_back("joint_a6");
      msg_joint.name.push_back("joint_a7");
      msg_joint.header.stamp = ros::Time::now(); 
      msg_joint.position.push_back(joint_whole(0));
      msg_joint.position.push_back(joint_whole(1));
      msg_joint.position.push_back(joint_whole(2));
      msg_joint.position.push_back(joint_whole(3));
      msg_joint.position.push_back(joint_whole(4));
      msg_joint.position.push_back(joint_whole(5));
      msg_joint.position.push_back(joint_whole(6));
      msg_joint.position.push_back(joint_whole(7));
      msg_joint.position.push_back(joint_whole(8));
      msg_joint.position.push_back(joint_whole(9));
      kmr_iiwa_pub.publish(msg_joint);
      ros::spinOnce(); 
      }
      
     }
