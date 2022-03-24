// Example from moveit_tutorials, move_group_interface_tutorial.cpp 
// See https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_interface/src/move_group_interface_tutorial.cpp

#ifndef TASK_11_HEADER
#define TASK_11_HEADER

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/package.h>

#include <unistd.h>

#include <cmath>

#include <nlohmann/json.hpp>


// The circle constant tau = 2*pi. One tau is one rotation in radians.
#define two 2
#define tau two*M_PI

using nlohmann::json;

class Task11 {

    private:
        //Args
        int &argc;
        char **argv;

        // ROS
        ros::NodeHandle node_handle;
        ros::Rate loop_rate;
        ros::Publisher joints_state_pub;
        ros::Publisher markers_pub;

        // MoveIt
        robot_model_loader::RobotModelLoader robot_model_loader;
        const moveit::core::RobotModelPtr& kinematic_model; // Should be const
        planning_scene::PlanningScene planning_scene;
        robot_state::RobotState& current_state;
        const robot_model::JointModelGroup* joint_model_group; // Should be const
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;

        // Poses
        std::string joint_pose_path;
        std::string cartesian_pose_path;
        std::string output_path;
        json joint_poses;
        json cartesian_poses;
        json output;
        std::vector<double> collision_positions;

    public:
        //Constructor and destructor
        Task11(int &argc, char **argv);
        
        //Joint collision checking
        void cartesian_pose_parse();
        void test_all_positions();
        void check_collision(const std::vector<double> &arr, const int &iterator);
        json get_collision_groups(const int &iterator);
        json get_collision_contacts(const int &iterator);
        void visualize_positions();

        //Message Operations
        visualization_msgs::MarkerArray get_ma_msg(json &pose);
        sensor_msgs::JointState get_js_msg(json &pose);

        //File and IO Operations
        void get_poses();
        void output_results();
        void wait_for_input();
};

#endif