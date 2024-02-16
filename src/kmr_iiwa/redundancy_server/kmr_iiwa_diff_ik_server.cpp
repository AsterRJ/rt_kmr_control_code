
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
#include <future>


using namespace std;
using namespace Eigen;




ros::Publisher arm_pub;
ros::Publisher base_pub;
ros::Publisher all_pub;


class MainServer{
  public:
    bool init = false;  
    kmr_iiwa robot_lib;
    MainServer(){
      joint_whole << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      secondary_vel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      primary_vel << 0.01, 0.0, 0.0, 0.0, 0.0, 0.0;      
    }

    void init_kin_params(string param_server_info){
      robot_lib.kinematic_parameters(param_server_info);
    }

    void secondary_vel_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
      if ((int)msg->data.size() == 10) //Primary 6 + Secondary 10 = 16
        {
          secondary_vel << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7], msg->data[8], msg->data[9];
        }
    }

    void primary_vel_callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
      primary_vel << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
    }

    void joint_pose_arm_callback(const sensor_msgs::JointState::ConstPtr&msg){
      for (int i = 0;i<7;i++){joint_whole(i+3) = msg->position[i];}
    }

    void joint_pose_base_callback(const geometry_msgs::Twist::ConstPtr&msg){
      joint_whole(0) = msg->linear.x;
      joint_whole(1) = msg->linear.y;
      joint_whole(2) = msg->angular.z;
    }

    void weight_matrix_callback(const std_msgs::Float64MultiArray::ConstPtr& msg){
        robot_lib.WeightMatrix.diagonal() << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7], msg->data[8], msg->data[9];
    }

    void main_task(){
      ros::Rate r(20);
      while(ros::ok())
      {
        joint_vel = robot_lib.weighted_robust_mobile_axis_ik_diff(joint_whole, primary_vel, secondary_vel);
        std_msgs::Float64MultiArray joint_message = std_msgs::Float64MultiArray();
        for (int i = 0; i<10; i++){joint_message.data.push_back(joint_vel(i));}
        all_pub.publish(joint_message);
        r.sleep();        
      }
    }

    void run(){
      auto run_ = async(launch::async, &MainServer::main_task, this);
    }

  private:
    Matrix<double, 10,1> joint_whole, joint_vel, secondary_vel;
    Matrix<double, 6,1> primary_vel;
};




int main(int argc, char ** argv)
{

    //Initialize and start the node
    ros::init(argc, argv, "kuka_kmr_redudancy_cpp");
    ros::NodeHandle n;
    MainServer ms = MainServer();
    // params on the server not compatible with the way this has been written - quick bodge will do, but needs further examination
    ms.init_kin_params("s");

    ros::Subscriber sub_secondary_vel = n.subscribe("redundant_velocity", 1, & MainServer::secondary_vel_callback, & ms);
    ros::Subscriber sub_cartesian_vel = n.subscribe("cartesian_velocity", 1, & MainServer::primary_vel_callback, &ms);
    ros::Subscriber sub_arm_pose = n.subscribe("kmriiwa/arm/joint_states", 1, &MainServer::joint_pose_arm_callback, &ms);
    ros::Subscriber sub_base_bose = n.subscribe("odom",1, &MainServer::joint_pose_base_callback, &ms);
    ros::Subscriber sub_weight_mat = n.subscribe("joint_weights",1,&MainServer::weight_matrix_callback,&ms);

    arm_pub= n.advertise<sensor_msgs::JointState>("kmriiwa/arm/joint_vel", 1);
    base_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    all_pub = n.advertise<std_msgs::Float64MultiArray>("joint_vel_all", 1);
    ms.run();
    while (ros::ok())
    {
      ros::spinOnce();
    }
    return 1;      
}
