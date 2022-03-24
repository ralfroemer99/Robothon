// Example from moveit_tutorials, move_group_interface_tutorial.cpp 
// See https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_interface/src/move_group_interface_tutorial.cpp

#include "task11.h"

Task11::Task11(int& argc, char** argv)
  : 
    argc(argc),
    argv(argv),
    loop_rate(0.5),
    robot_model_loader("robot_description"),
    kinematic_model(robot_model_loader.getModel()),
    planning_scene(kinematic_model),
    current_state(planning_scene.getCurrentStateNonConst()),
    joint_model_group(current_state.getJointModelGroup("panda_arm"))
  {
  this->joints_state_pub = this->node_handle.advertise<sensor_msgs::JointState>("/joint_states", 1);
  this->markers_pub = this->node_handle.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
}

void Task11::get_poses(){
  this->cartesian_pose_path = ros::package::getPath("t11")
                            + "/data/copy_self_collision_target_cartesian_poses.json";
  std::ifstream cartesian_stream(this->cartesian_pose_path);
  this->joint_pose_path = ros::package::getPath("t11") 
                        + "/data/copy_self_collision_specific_joint_poses.json";
  std::ifstream pose_stream(this->joint_pose_path);
  this->joint_poses = json::parse(pose_stream);
}

void Task11::test_all_positions(){
  int iterator = 0;
  for (const std::vector<double> &arr : this->joint_poses["poses"]){
    ++iterator;
    this->check_collision(arr, iterator);
    this->output["pose_" + std::to_string(iterator)] = {
      {"pose", arr},
      {"collision", this->collision_result.collision},
      {"valid", this->current_state.satisfiesBounds(joint_model_group)},
      {"collision_groups", this->get_collision_groups(iterator)},
      {"collision_contacts", this->get_collision_contacts(iterator)}
    };
  }
  this->output_results();

}

void Task11::check_collision(const std::vector<double> &arr, const int &iterator){
  this->collision_result.clear();
  this->collision_request.contacts = true;
  this->collision_request.max_contacts = 10;
  this->current_state.setJointGroupPositions(this->joint_model_group, arr);
  this->planning_scene.checkSelfCollision(this->collision_request, this->collision_result);
}

json Task11::get_collision_groups(const int &iterator){
  if(this->collision_result.collision){
    json collision_groups;
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = this->collision_result.contacts.begin(); it != this->collision_result.contacts.end(); ++it){
      collision_groups.push_back(json::array({it->first.first.c_str(), it->first.second.c_str()}));
    }
    return collision_groups;
  }
}

json Task11::get_collision_contacts(const int &iterator){
  if(this->collision_result.collision){
    json collision_contacts;
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = this->collision_result.contacts.begin(); it != this->collision_result.contacts.end(); ++it){
      for(auto &contact_it : it->second){
      //convert Eigen::Matrix to std::vector<double>
      // See https://stackoverflow.com/questions/26094379/typecasting-eigenvectorxd-to-stdvector
      const Eigen::Matrix<double, 3, 1> &mat = contact_it.pos;
      std::vector<double> vector(mat.data(), mat.data() + mat.rows() * mat.cols());
      collision_contacts.push_back(vector);
      }
    }
    return collision_contacts;
  }
}

void Task11::output_results(){
  this->output_path = ros::package::getPath("t11") + "/data/output.json";
  std::ofstream output_stream(this->output_path, std::ofstream::trunc);
  output_stream << std::setw(4) << this->output << std::endl;
}

visualization_msgs::MarkerArray Task11::get_ma_msg(json &pose){
  visualization_msgs::MarkerArray markers;
  bool has_been_filled = false;
  for (auto &contact : pose["collision_contacts"]){
    has_been_filled = true;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "panda_link0";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = contact[0];
    marker.pose.position.y = contact[1];
    marker.pose.position.z = contact[2];
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    markers.markers.push_back(marker);
  }
  if (!has_been_filled){
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(marker);
  }
  return markers;
}

sensor_msgs::JointState Task11::get_js_msg(json &pose){
  sensor_msgs::JointState msg;
  msg.name = joint_model_group->getJointModelNames();
  msg.name.end()[-1] = "panda_finger_joint1";
  msg.name.push_back("panda_finger_joint2");
  msg.position = pose["pose"].get<std::vector<double>>();
  msg.position.insert(msg.position.end(), {0.0, 0.0});
  msg.header.stamp = ros::Time::now();
  return msg;
}

void Task11::visualize_positions(){
  for (auto &pose : this->output){
    sensor_msgs::JointState js_msg = this->get_js_msg(pose);
    visualization_msgs::MarkerArray ma_msg = this->get_ma_msg(pose); 
    this->joints_state_pub.publish(js_msg);
    this->markers_pub.publish(ma_msg);
    ROS_INFO_STREAM("Current State: " << pose["pose"]);
    ROS_INFO_STREAM("Current Markers: " << pose["collision_contacts"]);
    ROS_INFO_STREAM("Current Collision Groups: " << pose["collision_groups"]);
    ros::spinOnce();
    this->loop_rate.sleep();
    }
}
void Task11::wait_for_input(){
  std::cout << std::endl << "###########################################" << std::endl;
  std::cout << "Press any key to start the visualization..." << std::endl;
  std::cout << "###########################################" << std::endl;
  std::cin.ignore();
}



int main(int argc, char** argv){
  ros::init(argc, argv, "task11");
  Task11 task(argc, argv);
  task.get_poses();
  task.test_all_positions();
  task.wait_for_input();
  task.visualize_positions();
}



/*int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(0.5);

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();

  //const std::vector<std::string> names = current_state.getJointModelGroupNames();
  //ROS_INFO_STREAM("Possible Joint Model Groups: "<< names);
  const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("panda_arm");
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;


  std::vector<double> joint_values = { 0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0 };
  std::vector<double> joint_coll = { 0.0, 1.04, 0.0, -3.07, 0.0, 1.57, 0.79};
  std::vector<double> q_start = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
  std::vector<double> q = {-0.015896176064967, 0.0064190366448024, -0.0075707499149116, -0.035205070178435, 0.010341694431194, 0.0093508241818754, 0.043023672042527};
  // 1. No, 2. No, 3.
  std::vector<std::vector<double>> q_examples = {{-0.5411817507994803, -1.5327192132849443, -0.9927683026414167, -1.8230752403861596, 1.87771979017059, 1.271592859382698, 1.1169918079714867},
                                    {-2.3783212199964026, 1.3806609172751285, -2.281388442457785, -2.5530324557943875, -1.887167331183505, 1.1272934809736468, 1.3750410813247251},
                                    {-2.3101277816800336, 1.4758684494183352, -2.1408592200257957, -2.2143569089975945, -1.9483915424722889, 1.2066065413686962, 0.8836448019875428}
                                  };

  ros::Publisher pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 50);
  //ros::Subscriber sub = node_handle.subscribe("/joint_states", 1, callback)
  
  ROS_INFO_STREAM("State = "<< current_state);
  
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  ROS_INFO_STREAM("State = "<< current_state);

  while(ros::ok()){
    sensor_msgs::JointState msg;
    msg.name = joint_model_group->getJointModelNames();
    msg.name.end()[-1] = "panda_finger_joint1";
    msg.name.push_back("panda_finger_joint2");

    collision_result.clear();
    current_state.setJointGroupPositions(joint_model_group, joint_coll);
    planning_scene.checkCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test : Current state is"<< (collision_result.collision ? "" : " not") << " a collision; State is "<< (current_state.satisfiesBounds(joint_model_group) ? "" : "in") << "valid "); //and  the current matrix is " << acm);
    current_state.copyJointGroupPositions(joint_model_group, msg.position);
    msg.header.stamp = ros::Time::now();
    msg.position.insert(msg.position.end(), {0.0, 0.0});
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();

    if(false){
    for(int i=0; i<q_examples.size(); i++){
      //current_state.setJointGroupPositions(joint_model_group, joint_coll);
      current_state.setJointGroupPositions(joint_model_group, q);
      collision_result.clear();

      current_state.copyJointGroupPositions(joint_model_group, msg.position);
      msg.position.insert(msg.position.end(), {0.0, 0.0});
      planning_scene.checkSelfCollision(collision_request, collision_result);
      ROS_INFO_STREAM("Test " << i << ": Current state is"<< (collision_result.collision ? "" : " not") << " a collision; State is "<< (current_state.satisfiesBounds(joint_model_group) ? "" : "in") << "valid.");
      msg.header.stamp = ros::Time::now();
      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
    }
    }  
  }
  ros::shutdown();
  return 0;
}

void callback(const sensor_msgs::JointState msg){

}*/