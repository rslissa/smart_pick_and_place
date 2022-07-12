#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "opencv_services/box_and_target_position.h"

#include <iostream>
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place_opencv");
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
  
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_interface_gripper(PLANNING_GROUP_GRIPPER);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  // Allow collisions between the gripper and the box to be able to grasp it
  planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
  collision_detection::AllowedCollisionMatrix& acm = ls->getAllowedCollisionMatrixNonConst();
  acm.setEntry("box_0", "robotiq_85_left_finger_tip_link", true);
  acm.setEntry("box_0", "robotiq_85_right_finger_tip_link", true);
  acm.setEntry("box_0", "robotiq_85_right_inner_knuckle_link", true);
  acm.setEntry("box_0","robotiq_85_left_inner_knuckle_link", true);
  acm.setEntry("box_0","robotiq_85_base_link", true);
  acm.setEntry("box_0", "robotiq_85_left_finger_link", true);
  acm.setEntry("box_0", "robotiq_85_right_finger_link", true);
  acm.setEntry("box_0", "robotiq_85_left_knuckle_link", true);
  acm.setEntry("box_0", "robotiq_85_right_knuckle_link", true);
  
  acm.setEntry("box_1", "robotiq_85_left_finger_tip_link", true);
  acm.setEntry("box_1", "robotiq_85_right_finger_tip_link", true);
  acm.setEntry("box_1", "robotiq_85_right_inner_knuckle_link", true);
  acm.setEntry("box_1","robotiq_85_left_inner_knuckle_link", true);
  acm.setEntry("box_1","robotiq_85_base_link", true);
  acm.setEntry("box_1", "robotiq_85_left_finger_link", true);
  acm.setEntry("box_1", "robotiq_85_right_finger_link", true);
  acm.setEntry("box_1", "robotiq_85_left_knuckle_link", true);
  acm.setEntry("box_1", "robotiq_85_right_knuckle_link", true);
  std::cout << "\nAllowedCollisionMatrix:\n";
  acm.print(std::cout);
  moveit_msgs::PlanningScene diff_scene;
  ls->getPlanningSceneDiffMsg(diff_scene);

  planning_scene_interface.applyPlanningScene(diff_scene); 

  ros::Duration(0.1).sleep();

  // We can get a list of all the groups in the robot:
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
          move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  // 1. Move to home position
  move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
  bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group_interface_arm.move();

  // Get the box and the target position from the opencv node
  ros::ServiceClient box_and_target_position_srv_client = n.serviceClient<opencv_services::box_and_target_position>("box_and_target_position");
  opencv_services::box_and_target_position srv;
  if(box_and_target_position_srv_client.call(srv)) {
    ROS_INFO_STREAM("3d target position camera frame: x " << srv.response.target_position.x << " y " << srv.response.target_position.y << " z " << srv.response.target_position.z);
    for(int i=0; i<srv.response.boxes_position.size(); ++i )
      ROS_INFO_STREAM("3d box position camera frame: x " << srv.response.boxes_position[i].x << " y " << srv.response.boxes_position[i].y << " z " << srv.response.boxes_position[i].z);
  } else {
    ROS_INFO_STREAM("Failed to call box and target position service");
  }
  
  for(int i=0; i<srv.response.boxes_position.size(); ++i ){
    // Add the object to be grasped (the square box) to the planning scene
    
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();
    collision_object.id = "box_"+std::to_string(i);
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.05;
    primitive.dimensions[1] = 0.05;
    primitive.dimensions[2] = 0.05;
    geometry_msgs::Pose box_pose;
    
    box_pose.orientation.w = 1.0;
    box_pose.position.x = srv.response.boxes_position[i].x;
    box_pose.position.y = srv.response.boxes_position[i].y;
    box_pose.position.z = srv.response.boxes_position[i].z;
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.applyCollisionObjects(collision_objects);
    
    
    
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    ros::Duration(0.1).sleep();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

    // 3. Open the gripper
    ROS_INFO_NAMED("tutorial", "// 3. Open the gripper");
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_gripper.move();

    // 2. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
    ROS_INFO_NAMED("tutorial", "// 2. Place the TCP (Tool Center Point, the tip of the robot) above the blue box");
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("ee_link");
    geometry_msgs::Pose target_pose1;
    std::vector<geometry_msgs::Pose> waypoints ={};
    target_pose1.orientation = current_pose.pose.orientation;
    
    target_pose1.position.x = srv.response.boxes_position[i].x;
    // ROS_ERROR_STREAM("BOX POSE POSITION = "<<box_pose.position.x<<"BOX_POSE_POSITION"
    if (i==0)
      target_pose1.position.x += 0.02;
    target_pose1.position.y = srv.response.boxes_position[i].y;
    target_pose1.position.z = srv.response.boxes_position[i].z+0.4;
    waypoints.push_back(target_pose1);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    // 4. Move the TCP close to the object
    ROS_INFO_NAMED("tutorial", "// 4. Move the TCP close to the object");
    geometry_msgs::Pose target_pose2 = target_pose1;
    target_pose2.position.z = 0;
    waypoints.push_back(target_pose2);
    double fraction = move_group_interface_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);  
    move_group_interface_arm.execute(trajectory);
    move_group_interface_arm.setPoseTarget(target_pose2);
    ros::Duration(2.0).sleep(); 

    // 5. Close the  gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("closed"));
    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_gripper.move();

    // Attach the box to the gripper after it was grasped
    moveit_msgs::AttachedCollisionObject aco;
    aco.object.id = collision_object.id;
    aco.link_name = "robotiq_85_right_finger_tip_link";
    aco.touch_links.push_back("robotiq_85_left_finger_tip_link");
    aco.object.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene_interface.applyAttachedCollisionObject(aco);

    // 6. Move the TCP above the plate
    target_pose1.position.z = srv.response.target_position.z + 0.4;
    target_pose1.position.x = srv.response.target_position.x;
    target_pose1.position.y = srv.response.target_position.y;
    move_group_interface_arm.setPoseTarget(target_pose1);

    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();

    // 7. Lower the TCP above the plate
    std::vector<geometry_msgs::Pose> waypoints1 ={};
    moveit_msgs::RobotTrajectory trajectory1;
    target_pose1.position.z = 0.06;    
    waypoints1.push_back(target_pose1);
    fraction = move_group_interface_arm.computeCartesianPath(waypoints1, eef_step, jump_threshold, trajectory1);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group_interface_arm.execute(trajectory1);
    move_group_interface_arm.setPoseTarget(target_pose1);

    ros::Duration(1.0).sleep(); 

    // 8. Open the gripper
    move_group_interface_gripper.setJointValueTarget(move_group_interface_gripper.getNamedTargetValues("open"));
    success = (move_group_interface_gripper.plan(my_plan_gripper) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_gripper.move();
    
    // 1. Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();

    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.object.id = "box_"+std::to_string(i);
    detach_object.link_name = "robotiq_85_right_finger_tip_link";
    detach_object.touch_links.clear();
    detach_object.object.operation = moveit_msgs::CollisionObject::REMOVE;

    planning_scene_interface.applyAttachedCollisionObject(detach_object);

    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

  }
  ros::shutdown();
  return 0;

}