#include "ros/ros.h"
#include "std_msgs/String.h" 
#include <sstream>
#include <string>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include "franka_msgs/FrankaState.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>

class Collision_Detector {
   private: 
      ros::NodeHandle nh;
      ros::Subscriber sub;
   public:
      Collision_Detector();
      void print_franka_states (const franka_msgs::FrankaState &message);
      void do_stuff(const franka_msgs::FrankaState &message);
      void do_stuff();
   
};