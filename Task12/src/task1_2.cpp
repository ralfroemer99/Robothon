#include <ros/ros.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <math.h>
#include <cstring>
#include <string.h>
#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/collision_detection/collision_common.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <fstream>
#include <json/value.h>
#include <json/json.h>

// BEGIN_SUB_TUTORIAL stateFeasibilityTestExample
//
// User defined constraints can also be specified to the PlanningScene
// class. This is done by specifying a callback using the
// setStateFeasibilityPredicate function. Here's a simple example of a
// user-defined callback that checks whether the "panda_joint1" of
// the Panda robot is at a positive or negative angle:
bool stateFeasibilityTestExample(const robot_state::RobotState& kinematic_state, bool verbose)
{
  const double* joint_values = kinematic_state.getJointPositions("panda_joint1");
  return (joint_values[0] > 0.0);
}
// END_SUB_TUTORIAL

// Set-up functions and const
static const std::string PLANNING_GROUP = "panda_arm";
static int count=0;
int joint_move_check(std::vector<double> q, moveit::planning_interface::MoveGroupInterface &move_group, ros::NodeHandle n, std::string ask);
int HT_move_check(Eigen::Matrix4f HT_t, moveit::planning_interface::MoveGroupInterface &move_group, ros::NodeHandle &n, std::string ask);
std::vector<geometry_msgs::Pose> alternative_path(std::vector<geometry_msgs::Pose> &waypoints);
bool pose_collision_check(moveit::planning_interface::MoveGroupInterface &move_group);


int main(int argc, char **argv)
{
    // ROS set-up
    ros::init(argc, argv, "Task1_2");

    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    //save initial joint
    std::vector<double> initial_joint = move_group.getCurrentJointValues();
    //save initial pose
    geometry_msgs::Pose initial_pose = move_group.getCurrentPose().pose;
    
    /////////////////////////////////////
    // Here is Mars custom node start //
    ///////////////////////////////////
    while (n.ok())
    {
        bool pose_coliision = pose_collision_check(move_group);
        std::system("clear");
        std::string mode;
        std::cout << "*********************************************"<<std::endl;
        std::cout << "**    Which mode do you want to try?       **"<<std::endl;
        std::cout << "**    1. Type mode(Joints)                 **"<<std::endl;
        std::cout << "**    2. Type mode(HT matrix)              **"<<std::endl;
        std::cout << "**    3. Data mode(Joints)                 **"<<std::endl;
        std::cout << "**    4. Data mode(HT matrix)              **"<<std::endl;
        std::cout << "**    Reset pose(r) / fisnish program(x)   **"<<std::endl;
        std::cout << "*********************************************"<<std::endl;
        // Check self collision for current pose
        if(pose_coliision){
            std::cout<<"--------- Current pose has self-collision"<<std::endl;
            ros::Duration(3.0).sleep();   
            return 0;
        }else{
            std::cout<<"--------- Current pose is okay for movement"<<std::endl;
        }
        std::cout << ">> ";
        std::getline(std::cin, mode);
        std::cout << std::endl;

        if(mode.compare("1")==0){
            std::string joint;
            std::cout << "Mars function - Joint MOVE Test"<<std::endl;
            std::cout << "Type 7 joints values like this ex) 1.415, 1.52, 0.9, -2.0, 0.40, 0.33, -1.03"<<std::endl;
            std::cout << ">> ";
            std::getline(std::cin, joint);
            std::cout << joint.size()<<std::endl;
            char joint_t[joint.size()];
            std::strcpy(joint_t, joint.c_str());
            std::vector<double> q;

            char *ptr = strtok(joint_t, ", ");

            while(ptr != NULL){
                q.push_back(std::stod(ptr));
                ptr = std::strtok(NULL, ", ");
            }
            if(q.size()==7){
                joint_move_check(q, move_group, n, "n");
            }else{
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                std::cout << "!!!    Initialize pose    !!!"<<std::endl;
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                ros::Duration(2.0).sleep();
            }
            //std::vector<double> q = {1.489378586467636, 0.28136031377942317, -1.5044935356531162, -2.686866944044088, 1.642650679087272, 0.25776674999732396, 0.5847876075123697};
            //joint_move_check(q, move_group, n, "n");

        }else if(mode.compare("2")==0){
            std::string HT_string;
            std::cout << "Mars function - Homogenous Transform matrix Test"<<std::endl;
            std::cout << "Type HT matrix values(4x4) like below (It's transpose of regular one)"<<std::endl;
            std::cout << "ex) 0.7103, -0.07768, 0.6996, 0, 0.06101, 0.9969, 0.04874, 0, -0.7013, 0.008062, 0.7129, 0, -0.216, 0.04661, 0.5734, 1"<<std::endl;
            std::cout << ">> ";
            std::getline(std::cin, HT_string);

            char HT_str[HT_string.size()];
            std::strcpy(HT_str, HT_string.c_str());
            std::vector<float> HT_temp;

            char *ptr = strtok(HT_str, ", ");

            while(ptr != NULL){
                HT_temp.push_back(std::stof(ptr));
                ptr = std::strtok(NULL, ", ");
            }

            if(HT_temp.size()==16){

                Eigen::Matrix4f HT_t; // Homogeneous transform matrix for movement of panda
                int r=-1, c=0;
                for(int i=0;i<HT_temp.size();i++){
                    if(i%4==0) r++;
                    c=i%4;
                    HT_t(r,c) = HT_temp[i];
                }
                HT_move_check(HT_t, move_group, n, "slow");

            }else{
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                std::cout << "!!!    Initialize pose    !!!"<<std::endl;
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                ros::Duration(2.0).sleep();
            }
            //Eigen::Matrix4f HT_t;
            //HT_t << 0.13345362611387276, 0.9889389847925516, -0.06472877284899119, 0.0, 0.9906017806351679, -0.1311326296443952, 0.03888888845133787, 0.0, 0.029970683667433672, -0.06931030082190896, -0.9971448442029288, 0.0, -0.025987168437732792, -0.24335988341465406, -0.3262082421980599, 1.0;
            //HT_t << 0.7103, -0.07768, 0.6996, 0, 0.06101, 0.9969, 0.04874, 0, -0.7013, 0.008062, 0.7129, 0, -0.216, 0.04661, 0.5734, 1;
            //HT_t << -0.5487125678022023, -0.5743739107649473, 0.6074612156907585, 0.0, -0.7391193599755831, -0.00624799183429997, -0.6735454953507767, 0.0, 0.3906623729580288, -0.818569223236874, -0.4211025256682544, 0.0, 0.2968732736228983, -0.4728232391832403, 0.19469777569505675, 1.0;
            //HT_move_check(HT_t, move_group, n, "slow");

        }else if(mode.compare("3")==0){
            std::cout << "==========================================================="<<std::endl;
            std::cout << "|  Joint Move Test from data(Json format)                 |"<<std::endl;
            std::cout << "|  Please change data if you want to upload specific data |"<<std::endl;
            std::cout << "|  It is located in (this package)/data/                  |"<<std::endl;
            std::cout << "|  file name: self_collision_specific_joint_poses.data    |"<<std::endl;
            std::cout << "==========================================================="<<std::endl;
            ros::Duration(4.0).sleep();

            std::string filepath(ros::package::getPath("Task12")+"/data/self_collision_specific_joint_poses.data");
            std::vector<std::vector<double>> q_2d;
            std::string line;

            std::ifstream input_file(filepath, std::ifstream::in|std::ifstream::binary);
            if(!input_file.is_open()){
                std::cerr<<"Can't open the file. Check file path - '"
                         << filepath<<"'"<<std::endl;
            }

            while (std::getline(input_file, line)){

                Json::Reader reader;
                Json::Value root(Json::arrayValue);
                reader.parse(line, root);
                std::vector<double> q;
                for(int j=0;j<root["q"].size();j++){
                    q.push_back(root["q"][j].asDouble());
                }
                q_2d.push_back(q);
            }
            input_file.close();
            int s_case=0;
            int fail;
            std::vector<int> edit;
            for(int i=0;i<q_2d.size();i++)
            {
                int check = joint_move_check(q_2d[i], move_group, n, "fast");
                ros::Duration(0.5).sleep();
                move_group.clearPoseTargets();
                move_group.setJointValueTarget(initial_joint);
                move_group.move();
                ros::Duration(1.0).sleep();
                if(check==1) s_case+=1;
                else if(check==0) fail+=1;
                else edit.push_back(check);
                count=0;
            }
            std::cout<<"Total   case : "<<q_2d.size()<<std::endl;
            std::cout<<"Success case : "<<s_case+edit.size()<<std::endl;
            std::cout<<"Edited  case : "<<edit.size()<<std::endl;
            //std::cout<<"Max : "<<max<<"Min : "<<min<<std::endl;
            std::cout<<"Edited : ";
            for(int j=0;j<edit.size();j++)
                std::cout<<edit[j]<<" ";
            std::cout<<std::endl;
            std::printf("Percentage   : %.2f%%\n",((float)(s_case+edit.size())/(float)q_2d.size())*100);
            ros::Duration(3.0).sleep();


        }else if(mode.compare("4")==0){
            std::cout << "==========================================================="<<std::endl;
            std::cout << "|  HT Move Test from data(Json format)                    |"<<std::endl;
            std::cout << "|  Please change data if you want to upload specific data |"<<std::endl;
            std::cout << "|  It is located in (this package)/data/                  |"<<std::endl;
            std::cout << "|  file name: self_collision_target_cartesian_poses.data  |"<<std::endl;
            std::cout << "==========================================================="<<std::endl;
            ros::Duration(4.0).sleep();
            //json file read
            std::ifstream json_data_ht;


            std::string filepath(ros::package::getPath("Task12")+"/data/self_collision_target_cartesian_poses.data");
            std::vector<Eigen::Matrix4f> HT_2d;
            std::string line;

            std::ifstream input_file(filepath, std::ifstream::in|std::ifstream::binary);
            if(!input_file.is_open()){
                std::cerr<<"Can't open the file. Check file path - '"
                         << filepath<<"'"<<std::endl;
            }

            while (std::getline(input_file, line)){

                Json::Reader reader;
                Json::Value root(Json::arrayValue);
                reader.parse(line, root);
                Eigen::Matrix4f HT_t;

                int m=-1, n=0;
                for(int j=0;j<root["O_T_EE_d"].size();j++){
                    if(j%4==0) m++;
                    n=j%4;
                    HT_t(m, n)=root["O_T_EE_d"][j].asFloat();
                }

                HT_2d.push_back(HT_t);
            }
            input_file.close();
            int fail = 0;
            int s_100 = 0;
            int s_90 = 0;
            int s_60 = 0;
            int s_0 =0;

            for(int i=0;i<HT_2d.size();i++)
            {
                int check=HT_move_check(HT_2d[i], move_group, n, "slow");
                ros::Duration(1.5).sleep();
                move_group.clearPoseTargets();
                move_group.setJointValueTarget(initial_joint);
                move_group.move();
                ros::Duration(2.0).sleep();
                if(check==1) s_100+=1;
                else if(check==2) s_90+=1;
                else if(check==3) s_60+=1;
                else if(check==4) s_0+=1;
                else fail+=1;
            }
            std::cout<<"Total   case : "<<HT_2d.size()<<std::endl;
            std::cout<<"Success case : "<<s_100+s_90+s_60+s_0<<std::endl;
            std::cout<<"Reach  100%  : "<<s_100<<std::endl;
            std::cout<<"Reach  90-100: "<<s_90<<std::endl;
            std::cout<<"Reach  60-90 : "<<s_60<<std::endl;
            std::cout<<"Reach  0-60  : "<<s_0<<std::endl;
            std::cout<<"Failure case : "<<fail<<std::endl;
            std::printf("Success rate : %.2f%%\n",((float)(s_100+s_90+s_60+s_0)/(float)HT_2d.size())*100);
            ros::Duration(3.0).sleep();


        }else if(mode.compare("r")==0){

            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
            std::cout << "!!!    Initialize pose    !!!"<<std::endl;
            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
            move_group.setJointValueTarget(initial_joint);
            move_group.move();
            ros::Duration(3.0).sleep();


        }else if(mode.compare("x")==0){
            break;

        }else{
            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
            std::cout << "!!!      Wrong input      !!!"<<std::endl;
            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
            ros::Duration(3.0).sleep();

        }

    }

    ros::shutdown();
    return 0;
}

int joint_move_check(std::vector<double> q, moveit::planning_interface::MoveGroupInterface &move_group, ros::NodeHandle n, std::string ask){
    ////////////////////////////
    // Joint value pose path //
    //////////////////////////

    // std::vector<double> joint_group_positions;
    // move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // for(int i=0;i<q.size();i++)
    //     joint_group_positions[i] = q[i];  // radians

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group.setJointValueTarget(q);

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(success){
        if(ask.compare("fast")==0){
            ros::Duration(0.5).sleep();
            move_group.move();
            ros::Duration(0.5).sleep();
        }else if(ask.compare("slow")==0){
            move_group.move();
            ros::Duration(4.0).sleep();
        }else{
            std::string move;
            std::cout << "Do you want to move it now? (y/n) : ";
            std::getline(std::cin, move);
            if(move.compare("y")==0){
                move_group.move();
                ros::Duration(3.0).sleep();
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                std::cout << "!!!     Move Success      !!!"<<std::endl;
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                ros::Duration(3.0).sleep();
            }else if(move.compare("n")==0){
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                std::cout << "!!!      Go to menu       !!!"<<std::endl;
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                ros::Duration(3.0).sleep();
            }else {
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                std::cout << "!!!      Wrong input      !!!"<<std::endl;
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                ros::Duration(3.0).sleep();
            }

        }
        return 1;
    }else {
        count++;
        int power = rand()%2;
        int random = rand()%5+2;
        for(int i=0;i<q.size();i++)
            q[i]+=(pow(-1,power)*0.01*random);
        
        int a = joint_move_check(q, move_group, n, "fast");
        if(a==1||a!=0) return count+1;
        
        // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        // std::cout << "!!!     Move Failure      !!!"<<std::endl;
        // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        // ros::Duration(5.0).sleep();
        return 0;
    }

}

int HT_move_check(Eigen::Matrix4f HT_t, moveit::planning_interface::MoveGroupInterface &move_group, ros::NodeHandle &n, std::string ask){
    ////////////////////////////////
    //  HT matrix cartesian path //
    //////////////////////////////

    Eigen::Matrix4f HT = HT_t.transpose();
    //std::cout <<"Homogenous Transformation : HT_t " << std::endl << HT_t << std::endl;
    //std::cout <<"Homogenous Transformation : HT " << std::endl << HT << std::endl;
    //std::cout << "change to quternion" << std::endl;

    Eigen::Matrix3f HT_R = HT.topLeftCorner(3,3);

    Eigen::Quaternionf HT_q = Eigen::Quaternionf(HT_R);

    //std::cout <<"Quaternion form vector : q" << std::endl <<  HT_q.vec() << std::endl;

    // Get current pose
    geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;

    
    std::vector<geometry_msgs::Pose> waypoints; //create waypoints

    move_group.setMaxVelocityScalingFactor(0.1);

    waypoints.push_back(current_pose);


    // display_publisher.publish(display_trajectory);

    geometry_msgs::Pose task1_2_target_pose; // create target pose

    //Rotate first
    task1_2_target_pose.orientation.w = static_cast<double>(HT_q.w());
    task1_2_target_pose.orientation.x = static_cast<double>(HT_q.x());
    task1_2_target_pose.orientation.y = static_cast<double>(HT_q.y());
    task1_2_target_pose.orientation.z = static_cast<double>(HT_q.z());
    //waypoints.push_back(task1_2_target_pose);
    //Move the x, y, z by cartesian path.
    task1_2_target_pose.position.x = current_pose.position.x+static_cast<double>(HT(0,3));
    task1_2_target_pose.position.y = current_pose.position.y+static_cast<double>(HT(1,3));
    task1_2_target_pose.position.z = current_pose.position.z+static_cast<double>(HT(2,3));
    waypoints.push_back(task1_2_target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.005;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    //ROS_INFO_NAMED("First Plan", "Visualizing plan 1 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    //Visualize trajectory
    Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d end = Eigen::Isometry3d::Identity();

    start.translation().x()=current_pose.position.x;
    start.translation().y()=current_pose.position.y;
    start.translation().z()=current_pose.position.z;
    Eigen::Quaterniond quaternion_start(current_pose.orientation.w, current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z);
    start.rotate(quaternion_start);

    end.translation().x()=task1_2_target_pose.position.x;
    end.translation().y()=task1_2_target_pose.position.y;
    end.translation().z()=task1_2_target_pose.position.z;
    Eigen::Quaterniond quaternion_end(task1_2_target_pose.orientation.w, task1_2_target_pose.orientation.x, task1_2_target_pose.orientation.y, task1_2_target_pose.orientation.z);
    end.rotate(quaternion_end);
    
    namespace rvt = rviz_visual_tools;
    rvt::RvizVisualTools visual_tools_("panda_link0", "visualization_marker_array", n);
    visual_tools_.loadMarkerPub();  // create publisher before waiting
    
    visual_tools_.publishLine(start, end, rvt::LIME_GREEN, rvt::MEDIUM);
    visual_tools_.trigger();
    
    ros::Duration(1.0).sleep();

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    int i=1;

    while(!success&&count<3){
        int power = rand()%2;
        int random = rand()%5 + 2;
        //waypoints = alternative_path(waypoints);
        std::vector<geometry_msgs::Pose> new_waypoints;
        moveit_msgs::RobotTrajectory new_trajectory;
        new_waypoints.push_back(current_pose);
        current_pose.position.x = current_pose.position.x+(pow(-1,power)*(0.01*random));
        current_pose.position.y = current_pose.position.y+(pow(-1,power)*(0.01*random));
        current_pose.position.z = current_pose.position.z+(pow(-1,power)*(0.01*random));
        new_waypoints.push_back(current_pose);
        task1_2_target_pose.position.x = current_pose.position.x+static_cast<double>(HT(0,3))+(pow(-1,power)*(0.01*random));
        task1_2_target_pose.position.y = current_pose.position.y+static_cast<double>(HT(1,3))+(pow(-1,power)*(0.01*random));
        task1_2_target_pose.position.z = current_pose.position.z+static_cast<double>(HT(2,3))+(pow(-1,power)*(0.01*random));
        new_waypoints.push_back(task1_2_target_pose);
        double fraction = move_group.computeCartesianPath(new_waypoints, eef_step, jump_threshold, new_trajectory);
        //ROS_INFO_NAMED("First Plan", "Visualizing plan 1 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
        count++;
        ros::Duration(4.0).sleep();
        trajectory=new_trajectory;
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        i++;
    }

    
    //my_plan.trajectory_->
    //std::cout<<"***********Plan : "<<(move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)<<std::endl;
    int s_rate = round(fraction*10000)/100;
    if(success){
        if(ask.compare("fast")==0){
            move_group.execute(trajectory);
            ros::Duration(1.0).sleep();
        }else if(ask.compare("slow")==0){
            ros::Duration(2.5).sleep();
            move_group.execute(trajectory);
            ros::Duration(1.5).sleep();
            visual_tools_.deleteAllMarkers();
        }else{
            std::string move;
            std::cout << "Do you want to move it now? (y/n) : ";
            std::getline(std::cin, move);
            if(move.compare("y")==0){
                move_group.execute(trajectory);
                ros::Duration(3.0).sleep();
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                std::cout << "!!!     Move Success      !!!"<<std::endl;
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                ros::Duration(3.0).sleep();
            }else if(move.compare("n")==0){
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                std::cout << "!!!      Go to menu       !!!"<<std::endl;
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                ros::Duration(3.0).sleep();
            }else {
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                std::cout << "!!!      Wrong input      !!!"<<std::endl;
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                ros::Duration(3.0).sleep();
            }

        }
        visual_tools_.deleteAllMarkers();
        count=0;
        if(s_rate==100) return 1;
        else if(s_rate<100 && s_rate>=90.0) return 2;
        else if(s_rate<90.0 && s_rate>=60.0) return 3;
        else if(s_rate<60.0) return 4;

    }else {
        // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        // std::cout << "!!!     Move Failure      !!!"<<std::endl;
        // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        // ros::Duration(5.0).sleep();
        visual_tools_.publishLine(start, start, rvt::CLEAR, rvt::SMALL);
        visual_tools_.deleteAllMarkers();
        count=0;
        return 0;
    }
    
}

bool pose_collision_check(moveit::planning_interface::MoveGroupInterface &move_group){
    move_group.getPoseTargets();
    // Get robot and kinematic model for planning scene
    planning_scene::PlanningScene planning_scene(move_group.getRobotModel());
    
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    collision_request.contacts = true;
    collision_request.max_contacts = 100;
    collision_request.max_contacts_per_pair = 9;
    collision_request.verbose = false;

    // Get allowed collision matrix
    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    
    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
    {
        acm.setEntry(it2->first.first, it2->first.second, true);
    }
    acm.setEntry("panda_hand","panda_link5", true);
    
    collision_result.clear();
    planning_scene.getCurrentState();
    planning_scene.checkSelfCollision(collision_request, collision_result, planning_scene.getCurrentState(), acm);

    if(collision_result.collision){
        collision_result.print();
        ros::Duration(3.0).sleep();
        collision_result.clear();
        return true;
    }else{
        collision_result.clear();
        return false;
    }
}

std::vector<geometry_msgs::Pose> alternative_path(std::vector<geometry_msgs::Pose> &waypoints){

    std::vector<geometry_msgs::Pose> new_waypoints;

    // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    // std::cout << "!!!       New plan        !!!"<<std::endl;
    // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;

    geometry_msgs::Pose target_pose2 = waypoints.back();
    //geometry_msgs::Pose target_pose1 = waypoints.back();
    geometry_msgs::Pose start_pose = waypoints.back();

    target_pose2.position.x+=0.02;
    target_pose2.position.y+=0.02;
    target_pose2.position.z+=0.02;

    start_pose.position.x+=0.02;
    start_pose.position.y+=0.02;
    start_pose.position.z+=0.02;

    new_waypoints.push_back(start_pose);
    //new_waypoints.push_back(target_pose1);
    new_waypoints.push_back(target_pose2);

    return new_waypoints;
}