#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <iostream>
using namespace std;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle n;
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP_ARM = "ur5_arm";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";
  
  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Collision object
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();

  collision_object.id = "box1";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.2;
  primitive.dimensions[1] = 0.35;
  primitive.dimensions[2] = 0.35;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.5;
  box_pose.position.z = 1.23 - 1.21;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  ros::Duration(2).sleep();

  planning_scene_interface.addCollisionObjects(collision_objects);
  planning_scene_interface.applyCollisionObjects(collision_objects);

  ROS_INFO_NAMED("tutorial", "Add an object into the world");

  ros::Duration(2.0).sleep(); 

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
          move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::Pose target_pose1;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;



  // 1. Move to home position
  move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));  
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group_interface_arm.move();
  ros::Duration(1.0).sleep(); 


  // 2. Open the gripper
  move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
  success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group_interface_gripper.move();
  ros::Duration(1.0).sleep(); 


////////////////////////77
Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools visual_tools("robotiq_85_base_link");
visual_tools.deleteAllMarkers();
////////////////////////


  // 3. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
  std::vector<geometry_msgs::Pose> waypoints ={};
  current_pose = move_group_interface_arm.getCurrentPose("ee_link");
  target_pose1.orientation = current_pose.pose.orientation;
  cout <<  "current_pose.pose.orientation ";
  cout << current_pose.pose.orientation;
  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.5;
  target_pose1.position.z = 0.2;
  waypoints.push_back(target_pose1);
  geometry_msgs::Pose target_pose2 = target_pose1;
  target_pose2.orientation = current_pose.pose.orientation;
  target_pose2.position.z = target_pose2.position.z - 0.2;
  waypoints.push_back(target_pose2);
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
  move_group_interface_arm.execute(trajectory);
  ros::Duration(1.0).sleep(); 
////////////////////////////
visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
for (std::size_t i = 0; i < waypoints.size(); ++i)
  visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
visual_tools.trigger();
////////////////////////////

  // 4. Close the  gripper
  move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("closed"));
  success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group_interface_gripper.move();
  ros::Duration(1.0).sleep(); 


  // 3. Place the TCP above the plate avoiding the wall
  current_pose = move_group_interface_arm.getCurrentPose("ee_link");
  target_pose1.orientation = current_pose.pose.orientation;
  target_pose1.position.z = target_pose1.position.z + 0.1;
  target_pose1.position.x = target_pose1.position.x - 0.6;
  move_group_interface_arm.setPoseTarget(target_pose1);
  success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group_interface_arm.move();
  ros::Duration(1.0).sleep(); 


  // 8. Open the gripper
  move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
  success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group_interface_gripper.move();
  ros::Duration(1.0).sleep(); 

}
