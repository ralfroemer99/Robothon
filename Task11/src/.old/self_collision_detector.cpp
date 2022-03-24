#include "self_collision_detector.h"

Collision_Detector::Collision_Detector(){
   //this->sub = this->nh.subscribe("/franka_state_controller/franka_states", 50, &Collision_Detector::print_franka_states, this);
   this->do_stuff();
}
void Collision_Detector::print_franka_states (const franka_msgs::FrankaState &message){
      std::cout << message << std::endl;
}

void Collision_Detector::do_stuff(){
   //static const std::string PLANNING_GROUP = "panda_arm";
   //moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
   //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
   //const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
   //namespace rvt = rviz_visual_tools;
   robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
   const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
   planning_scene::PlanningScene planning_scene(kinematic_model);
   collision_detection::CollisionRequest collision_request;
   collision_detection::CollisionResult collision_result;
   planning_scene.checkSelfCollision(collision_request, collision_result);
   ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
   moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
   current_state.setToRandomPositions();
   collision_result.clear();
   planning_scene.checkSelfCollision(collision_request, collision_result);
   ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
   collision_request.group_name = "hand";
   current_state.setToRandomPositions();
   collision_result.clear();
   planning_scene.checkSelfCollision(collision_request, collision_result);
   ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
   std::vector<double> joint_values = { 0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0 };
   const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("panda_arm");
   current_state.setJointGroupPositions(joint_model_group, joint_values);
   ROS_INFO_STREAM("Test 4: Current state is "
                     << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));
   collision_request.contacts = true;
   collision_request.max_contacts = 1000;
   collision_result.clear();
   planning_scene.checkSelfCollision(collision_request, collision_result);
   ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
   collision_detection::CollisionResult::ContactMap::const_iterator it;
   for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
   {
      ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
   }

   // Modifying the Allowed Collision Matrix
   // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //
   // The :collision_detection_class:`AllowedCollisionMatrix` (ACM)
   // provides a mechanism to tell the collision world to ignore
   // collisions between certain object: both parts of the robot and
   // objects in the world. We can tell the collision checker to ignore
   // all collisions between the links reported above, i.e. even though
   // the links are actually in collision, the collision checker will
   // ignore those collisions and return not in collision for this
   // particular state of the robot.
   //
   // Note also in this example how we are making copies of both the
   // allowed collision matrix and the current state and passing them in
   // to the collision checking function.

   collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
   moveit::core::RobotState copied_state = planning_scene.getCurrentState();

   collision_detection::CollisionResult::ContactMap::const_iterator it2;
   for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
   {
      acm.setEntry(it2->first.first, it2->first.second, true);
   }
   collision_result.clear();
   planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
   ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

   // Full Collision Checking
   // ~~~~~~~~~~~~~~~~~~~~~~~
   //
   // While we have been checking for self-collisions, we can use the
   // checkCollision functions instead which will check for both
   // self-collisions and for collisions with the environment (which is
   // currently empty).  This is the set of collision checking
   // functions that you will use most often in a planner. Note that
   // collision checks with the environment will use the padded version
   // of the robot. Padding helps in keeping the robot further away
   // from obstacles in the environment.
   collision_result.clear();
   planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
   ROS_INFO_STREAM("Test 7: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

   // Constraint Checking
   // ^^^^^^^^^^^^^^^^^^^
   //
   // The PlanningScene class also includes easy to use function calls
   // for checking constraints. The constraints can be of two types:
   // (a) constraints chosen from the
   // :kinematic_constraints:`KinematicConstraint` set:
   // i.e. :kinematic_constraints:`JointConstraint`,
   // :kinematic_constraints:`PositionConstraint`,
   // :kinematic_constraints:`OrientationConstraint` and
   // :kinematic_constraints:`VisibilityConstraint` and (b) user
   // defined constraints specified through a callback. We will first
   // look at an example with a simple KinematicConstraint.
   //
   // Checking Kinematic Constraints
   // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   //
   // We will first define a simple position and orientation constraint
   // on the end-effector of the panda_arm group of the Panda robot. Note the
   // use of convenience functions for filling up the constraints
   // (these functions are found in the :moveit_core_files:`utils.h<utils_8h>` file from the
   // kinematic_constraints directory in moveit_core).

   std::string end_effector_name = joint_model_group->getLinkModelNames().back();

   geometry_msgs::PoseStamped desired_pose;
   desired_pose.pose.orientation.w = 1.0;
   desired_pose.pose.position.x = 0.3;
   desired_pose.pose.position.y = -0.185;
   desired_pose.pose.position.z = 0.5;
   desired_pose.header.frame_id = "panda_link0";
   moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);

   // Now, we can check a state against this constraint using the
   // isStateConstrained functions in the PlanningScene class.

   copied_state.setToRandomPositions();
   copied_state.update();
   bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
   ROS_INFO_STREAM("Test 8: Random state is " << (constrained ? "constrained" : "not constrained"));

   // There's a more efficient way of checking constraints (when you want
   // to check the same constraint over and over again, e.g. inside a
   // planner). We first construct a KinematicConstraintSet which
   // pre-processes the ROS Constraints messages and sets it up for quick
   // processing.

   kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
   kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms());
   bool constrained_2 = planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
   ROS_INFO_STREAM("Test 9: Random state is " << (constrained_2 ? "constrained" : "not constrained"));

   // There's a direct way to do this using the KinematicConstraintSet
   // class.

   kinematic_constraints::ConstraintEvaluationResult constraint_eval_result = kinematic_constraint_set.decide(copied_state);
   ROS_INFO_STREAM("Test 10: Random state is " << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));

   // User-defined constraints
   // ~~~~~~~~~~~~~~~~~~~~~~~~
   //
   // CALL_SUB_TUTORIAL stateFeasibilityTestExample

   // Now, whenever isStateFeasible is called, this user-defined callback
   // will be called.

   //planning_scene.setStateFeasibilityPredicate(stateFeasibilityTestExample);
   //bool state_feasible = planning_scene.isStateFeasible(copied_state);
   //ROS_INFO_STREAM("Test 11: Random state is " << (state_feasible ? "feasible" : "not feasible"));

   // Whenever isStateValid is called, three checks are conducted: (a)
   // collision checking (b) constraint checking and (c) feasibility
   // checking using the user-defined callback.

   bool state_valid = planning_scene.isStateValid(copied_state, kinematic_constraint_set, "panda_arm");
   ROS_INFO_STREAM("Test 12: Random state is " << (state_valid ? "valid" : "not valid"));
   }

int main(int argc, char **argv)
{
   ros::init(argc, argv, "self_collision_detector");
   Collision_Detector cd;
   ros::spin();
   return 0;
}
