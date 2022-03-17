#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "auto_crane_path_plan");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::cout << "init"<<std::endl;
  static const std::string PLANNING_GROUP = "gripper_group";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  std::cout << "adding objects"<<std::endl;
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 1;
  primitive.dimensions[primitive.BOX_Y] = 0.5;
  primitive.dimensions[primitive.BOX_Z] = 1;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0;
  box_pose.position.y = 0.95;
  box_pose.position.z = -0.5;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface.addCollisionObjects(collision_objects);


  std::cout << "setting state"<<std::endl;
  // set the start and gaol joint state
  std::vector<double> start_joint_state{-M_PI/6, 0.15, 1.15};
  std::vector<double> goal_joint_state{M_PI/6, 0.95, 1};

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setJointGroupPositions(joint_model_group, start_joint_state);


  move_group_interface.setStartState(*kinematic_state);
  move_group_interface.setJointValueTarget(goal_joint_state);


  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  move_group_interface.setMaxVelocityScalingFactor(0.05);
  move_group_interface.setMaxAccelerationScalingFactor(0.05);

  std::cout << "plan"<<std::endl;
  // Plan and excute
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group_interface.execute(my_plan);

  for(int i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); i++) {
      std::cout << my_plan.trajectory_.joint_trajectory.points[i].positions[0];
      std::cout<<"\t";
      std::cout << my_plan.trajectory_.joint_trajectory.points[i].positions[1];
      std::cout<<"\t";
      std::cout << my_plan.trajectory_.joint_trajectory.points[i].positions[2];
      std::cout<<"\n";
  }

  std::cout << "clean up"<<std::endl;
  // Now, let's remove the objects from the world.
  ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);


  ros::shutdown();
  return 0;
}