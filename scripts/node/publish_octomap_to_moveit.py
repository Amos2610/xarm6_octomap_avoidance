#!/usr/bin/env python3
import rospy
from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import OctomapWithPose, Octomap
from std_msgs.msg import Header

class PublishOctomapToMoveIt:
    def __init__(self):
        self.source_topic = rospy.get_param("~source_topic", "/octomap_dynamic/octomap_full")
        self.scene_topic  = rospy.get_param("~scene_topic",  "/planning_scene")

        self.pub = rospy.Publisher(self.scene_topic, PlanningScene, queue_size=10)
        rospy.Subscriber(self.source_topic, Octomap, self.octomap_callback, queue_size=1)

        rospy.loginfo(f"Publishing dynamic Octomap from {self.source_topic} to {self.scene_topic}")

        rospy.on_shutdown(self.clear_octomap)

    def clear_octomap(self):
        rospy.loginfo("Clearing octomap from MoveIt PlanningScene")
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.octomap.octomap = Octomap()
        self.pub.publish(ps)

    def octomap_to_octomap_with_pose(self, octomap):
        owp = OctomapWithPose()
        owp.header = Header()
        owp.header.frame_id = octomap.header.frame_id
        owp.header.stamp = rospy.Time.now()
        owp.octomap = octomap
        return owp

    def octomap_callback(self, octomap_msg):
        owp = self.octomap_to_octomap_with_pose(octomap_msg)
        owp.header.stamp = rospy.Time.now()
        owp.octomap = octomap_msg

        ps = PlanningScene()
        ps.is_diff = True
        ps.world.octomap.header = owp.header
        ps.world.octomap.origin = owp.origin
        ps.world.octomap.octomap = owp.octomap

        self.pub.publish(ps)
        rospy.logdebug("Published dynamic octomap to MoveIt")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("publish_dynamic_octomap_to_moveit")
    try:
        node = PublishOctomapToMoveIt()
        node.run()
    except rospy.ROSInterruptException:
        pass
