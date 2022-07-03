#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class ObscaleDetectorNode
{
  public:
    ObscaleDetectorNode(ros::NodeHandle node_handle)
        : mNode(node_handle),
        mLoopRate(15),
        mmove_group_interface("gripper_group"),
        mobscale_id(0) {
        mMarker_sub = mNode.subscribe("lxz", 1, &ObscaleDetectorNode ::callBackMarker, this);
    
    }

    ~ObscaleDetectorNode() {}
    
    void run() {
      while (ros::ok())
      {
        ros::spinOnce();
        mLoopRate.sleep();
      }
    }

  private:
    void callBackMarker(const visualization_msgs::Marker::ConstPtr& marker_msg) {

        geometry_msgs::PoseStamped marker_pose;
        marker_pose.header = marker_msg->header;
        marker_pose.pose = marker_msg->pose;

        geometry_msgs::PoseStamped obscale_pose;
        try{
            listener.transformPose(mmove_group_interface.getPlanningFrame(), marker_pose, obscale_pose);
        } 
        catch(tf::TransformException ex) {
            ROS_WARN("transfrom exception : %s",ex.what());
            return;
        }

        std::vector<moveit_msgs::CollisionObject> collision_objects;

        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = mmove_group_interface.getPlanningFrame();

        collision_object.id = mobscale_id;

        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = marker_msg->scale.x;
        primitive.dimensions[primitive.BOX_Y] = marker_msg->scale.y;
        primitive.dimensions[primitive.BOX_Z] = marker_msg->scale.z;


        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(obscale_pose.pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
        
        mplanning_scene_interface.addCollisionObjects(collision_objects);
    }

    //members
    int mobscale_id;
    ros::NodeHandle mNode;
    ros::Rate mLoopRate;
  
    tf::TransformListener listener;
    ros::Subscriber mMarker_sub;

    moveit::planning_interface::MoveGroupInterface mmove_group_interface;
    moveit::planning_interface::PlanningSceneInterface mplanning_scene_interface;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ObjectDetectorNode");
  ros::NodeHandle nh("~");


  ObscaleDetectorNode rosNode(nh);
  rosNode.run();
  return 0;
}
