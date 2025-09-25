#!/usr/bin/env python3
import rospy
from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import OctomapWithPose, Octomap
from std_msgs.msg import Header

class OctomapToMoveIt:
    def __init__(self):
        rospy.init_node('octomap_to_moveit', anonymous=True)
        # Get parameters
        self.source_topic = rospy.get_param('~source_topic', '/octomap_full')
        self.scene_topic  = rospy.get_param('~scene_topic',  '/planning_scene')

        self.pub = rospy.Publisher(self.scene_topic, PlanningScene, queue_size=10)
        rospy.Subscriber(self.source_topic, Octomap, self.octomap_callback)
        rospy.loginfo("OctomapToMoveIt node initialized")

    def octomap_to_octomap_with_pose(self, octomap):
        octomap_with_pose = OctomapWithPose()
        octomap_with_pose.header = Header()
        octomap_with_pose.header.frame_id = octomap.header.frame_id
        octomap_with_pose.header.stamp = rospy.Time.now()
        octomap_with_pose.octomap = octomap
        return octomap_with_pose

    def octomap_callback(self, msg):
        rospy.loginfo("Received octomap message")
        # Convert the Octomap message to OctomapWithPose
        owp = self.octomap_to_octomap_with_pose(msg)
        
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.octomap.header.frame_id = owp.header.frame_id
        ps.world.octomap.header.stamp = rospy.Time.now()
        ps.world.octomap.origin = owp.origin
        ps.world.octomap.octomap.header.frame_id = owp.header.frame_id
        ps.world.octomap.octomap.header.stamp = rospy.Time.now()
        ps.world.octomap.octomap.binary = owp.octomap.binary
        ps.world.octomap.octomap.id = owp.octomap.id
        ps.world.octomap.octomap.resolution = owp.octomap.resolution
        ps.world.octomap.octomap.data = owp.octomap.data

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
