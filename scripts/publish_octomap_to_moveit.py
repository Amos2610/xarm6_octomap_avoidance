#!/usr/bin/env python3
import rospy
from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import OctomapWithPose, Octomap
from std_msgs.msg import Header

class OctomapToMoveIt:
    def __init__(self):
        rospy.init_node('octomap_to_moveit', anonymous=True)
        self.pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=10)
        rospy.Subscriber('/octomap_full', Octomap, self.octomap_callback)
        rospy.loginfo("OctomapToMoveIt node initialized")

    def octomap_to_ocotomap_with_pose(self, octomap):
        octomap_with_pose = OctomapWithPose()
        octomap_with_pose.header = Header()
        octomap_with_pose.header.frame_id = octomap.header.frame_id
        octomap_with_pose.header.stamp = rospy.Time.now()
        octomap_with_pose.octomap = octomap
        return octomap_with_pose

    def octomap_callback(self, msg):
        rospy.loginfo("Received octomap message")
        # Convert the Octomap message to OctomapWithPose
        octomap_with_pose = self.octomap_to_ocotomap_with_pose(msg)
        msg = octomap_with_pose
        
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.octomap.header.frame_id = msg.header.frame_id
        ps.world.octomap.header.stamp = rospy.Time.now()
        ps.world.octomap.origin = msg.origin
        ps.world.octomap.octomap.header.frame_id = msg.header.frame_id
        ps.world.octomap.octomap.header.stamp = rospy.Time.now()
        ps.world.octomap.octomap.binary = msg.octomap.binary
        ps.world.octomap.octomap.id = msg.octomap.id
        ps.world.octomap.octomap.resolution = msg.octomap.resolution
        ps.world.octomap.octomap.data = msg.octomap.data

        self.pub.publish(ps)
        rospy.loginfo("Published octomap to MoveIt PlanningScene")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = OctomapToMoveIt()
        node.run()
    except rospy.ROSInterruptException:
        pass
