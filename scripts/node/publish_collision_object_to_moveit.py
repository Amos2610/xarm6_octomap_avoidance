#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from moveit_msgs.msg import PlanningScene, CollisionObject


class PublishCollisionObjectToMoveIt:
    """
    生成済みの CollisionObject 群を受け取り、
    MoveIt の PlanningScene に送信するクラス
    """

    def __init__(self, scene_topic="/planning_scene"):
        rospy.loginfo("PublishCollisionObjectToMoveIt initialized")
        self.pub = rospy.Publisher(scene_topic, PlanningScene, queue_size=10)

    def publish(self, collision_objects):
        """
        CollisionObject のリストを PlanningScene にまとめて publish する
        """
        if not isinstance(collision_objects, list):
            rospy.logwarn("Expected a list of CollisionObjects, got %s", type(collision_objects))
            return

        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = collision_objects

        self.pub.publish(ps)
        rospy.loginfo("Published %d CollisionObjects into PlanningScene", len(collision_objects))


if __name__ == "__main__":
    rospy.init_node("publish_collision_object_to_moveit")

    # デモ用: 空のCollisionObjectリストをpublish
    demo_collision_objects = []  # ← 実際には外部から渡す
    publisher = PublishCollisionObjectToMoveIt(scene_topic="/planning_scene")
    publisher.publish(demo_collision_objects)
