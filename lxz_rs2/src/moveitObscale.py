#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from visualization_msgs.msg import Marker


class ObscaleSceneInserter:
  def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('obscale_inserter', anonymous=True)
    self.obs_idx = 0
    self.scene = moveit_commander.PlanningSceneInterface()
    self.obscale_pose_sub = rospy.Subscriber('obscale_pose', Marker, self.obscale_callback)


  def obscale_callback(self, data):

    box_name = 'obscale_{}'.format(self.obs_idx)
    self.obs_idx += 1
    scene = self.scene
  
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.obs_idx
    box_pose.pose.orientation.w = 1.0
    box_pose.pose = data.pose
    box_size = data.size
    scene.add_box(box_name, data, box_size)

    return self.wait_for_state_update(box_is_known=True)



if __name__ == '__main__':
    inserter = ObscaleSceneInserter()
    rospy.spin()