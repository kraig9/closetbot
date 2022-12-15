#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit/macros/console_colors.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // MoveIt2 Setup
    
  // static const std::string PLANNING_GROUP = "closetbot";
  static const std::string PLANNING_GROUP = "arm_group";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization

  namespace rvt = rviz_visual_tools;
  // rviz_visual_tools::RvizVisualTools visual_tools("base_link", "move_group_tutorial", move_group_node);
  rviz_visual_tools::RvizVisualTools visual_tools("base_link", "display_planned_path", move_group_node);
  visual_tools.deleteAllMarkers();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();


  // Getting Basic Information
  
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End-effector link: %s", move_group.getEndEffectorLink().c_str());

  move_group.setPoseReferenceFrame("base_link");
  RCLCPP_INFO(LOGGER, "Pose Reference Frame: %s", move_group.getPoseReferenceFrame().c_str());

  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));


  // Plan to End-Effector Pose

  std::vector<double> joint_group_positions_arm;
  joint_group_positions_arm.push_back(0); 
  joint_group_positions_arm.push_back(0);  
  joint_group_positions_arm.push_back(0);
  joint_group_positions_arm.push_back(0);
  joint_group_positions_arm.push_back(0); 
  
  geometry_msgs::msg::Pose target_pose1;

  while(true){
    // target_pose1.orientation.x = -1.0;
    // target_pose1.orientation.y = 0.00;
    // target_pose1.orientation.z = 0.00;
    // target_pose1.orientation.w = 0.00;
    // target_pose1.position.x = 0.343;
    // target_pose1.position.y = 0.132;
    // target_pose1.position.z = 0.284;
    // target_pose1.orientation.x = -1.0;
    // target_pose1.orientation.y = 0.00;
    // target_pose1.orientation.z = 0.00;
    // target_pose1.orientation.w = 0.00;
    // target_pose1.position.x = -1;
    // target_pose1.position.y = -2;
    // target_pose1.position.z = 10;
    // move_group.setPoseTarget(target_pose1, "EE");

        RCLCPP_INFO(LOGGER, "Planning to Joint Space");

      // joint_group_positions_arm.push_back(0.00);
      // if(joint_group_positions_arm[0] >= .1) joint_group_positions_arm[0] = 0; 
      // if(joint_group_positions_arm[1] >= .1) joint_group_positions_arm[1] = 0;  
      // if(joint_group_positions_arm[3] >= .1) joint_group_positions_arm[3] = 0; 


      if(joint_group_positions_arm[0] >= 1.5) joint_group_positions_arm[0] = -2.0; 
      if(joint_group_positions_arm[1] >= 1.5) joint_group_positions_arm[1] = -2.0;  
      if(joint_group_positions_arm[3] >= 1.5) joint_group_positions_arm[3] = -2.0; 

      // joint_group_positions_arm[0]+=.01; 
      // joint_group_positions_arm[1]+=.01;  
      // joint_group_positions_arm[3]+=.01; 

      joint_group_positions_arm[0]+=.5; 
      joint_group_positions_arm[1]+=.5;  
      joint_group_positions_arm[3]+=.5; 



      // joint_group_positions_arm.push_back(-1.55); 
      //joint_group_positions_arm[5] = 0.00;

      move_group.setJointValueTarget(joint_group_positions_arm);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setMaxVelocityScalingFactor(1);

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if(success) move_group.execute(my_plan);

    RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    sleep(4);
  }

  //  Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();


  rclcpp::shutdown();
  return 0;
}