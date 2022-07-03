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

#include <jsoncpp/json/json.h>
#include <fstream>

#include <auto_crane/waypoint.h>
#include <cmath>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "auto_crane_path_plan");
  ros::NodeHandle node_handle;
  ros::Publisher waypoint_pub = node_handle.advertise<auto_crane::waypoint>("waypoint", 1000);
  ros::AsyncSpinner spinner(1);
  spinner.start();


  std::cout << "reading config file from scene.json"<<std::endl;
  Json::Value config;
  Json::Reader reader;
  std::string scene_path;
  node_handle.getParam("/plan/scene_path", scene_path);
  std::ifstream ifs(scene_path);
  if(!ifs){
    std::cerr<<"scene.json not exsist." << std::endl;
  }
  if(!reader.parse(ifs, config)){
    std::cerr<<"failed to parse szene.json"<<std::endl;
  }


  std::cout << "init"<<std::endl;
  static const std::string PLANNING_GROUP = "gripper_group";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  std::cout << "adding objects"<<std::endl;
  Json::Value obscales = config["obscales"];
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  for(int i = 0; i < obscales.size(); i++){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface.getPlanningFrame();
    // The id of the object is used to identify it.
    collision_object.id = obscales[i]["id"].asString();

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    geometry_msgs::Pose pose;
          std::cout<<obscales[i]["shape"].asString()<<std::endl;

    if(obscales[i]["shape"].asString() == "BOX"){
      std::cout<<"adding box"<<std::endl;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = obscales[i]["position"]["x"].asDouble();
      primitive.dimensions[primitive.BOX_Y] = obscales[i]["position"]["y"].asDouble();
      primitive.dimensions[primitive.BOX_Z] = obscales[i]["position"]["z"].asDouble();

      // Define a pose for the box (specified relative to frame_id)
      pose.orientation.w = obscales[i]["oritation"]["w"].asDouble();
      pose.position.x = obscales[i]["oritation"]["x"].asDouble();
      pose.position.y = obscales[i]["oritation"]["y"].asDouble();
      pose.position.z = obscales[i]["oritation"]["z"].asDouble();
    } else if (obscales[i]["shape"] == "CYLINDER") {
      primitive.type = primitive.CYLINDER;
    } 
    else {
      std::cerr<<"invalid obscales shape."<<std::endl;
    }
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);
  }
  planning_scene_interface.addCollisionObjects(collision_objects);

  std::cout << "setting state"<<std::endl;
  // set the start and gaol joint state
  Json::Value start_state = config["start_state"];
  Json::Value goal_state = config["goal_state"];
  std::vector<double> start_joint_state{
    start_state["base_arm"].asDouble(), start_state["arm_slider"].asDouble(), start_state["slider_gripper"].asDouble()
    };
  std::vector<double> goal_joint_state{
    goal_state["base_arm"].asDouble(), goal_state["arm_slider"].asDouble(), goal_state["slider_gripper"].asDouble()
  };

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
      auto_crane::waypoint msg;
      msg.base_arm = my_plan.trajectory_.joint_trajectory.points[i].positions[0];
      msg.arm_slidder = my_plan.trajectory_.joint_trajectory.points[i].positions[1];
      msg.slidder_gripper = my_plan.trajectory_.joint_trajectory.points[i].positions[2];
      std::cout << msg.base_arm <<"\t" << msg.arm_slidder <<"\t" << msg.slidder_gripper << "\n";

      msg.position[0] = msg.arm_slidder * sin(msg.base_arm);
      msg.position[1] = msg.arm_slidder * cos(msg.base_arm);
      msg.position[2] = 1.6 - msg.slidder_gripper;
      waypoint_pub.publish(msg);
  }

  ros::shutdown();
  return 0;
}