#!/usr/bin/env python3
import rospy
from node.mapping_move import MoveItMapping
from node.convert_octomap_to_collision_object import OctoMap2CollisionObject
from node.publish_octomap_to_moveit import PublishOctomapToMoveIt
from node.publish_collision_object_to_moveit import PublishCollisionObjectToMoveIt
from node.yolo_to_collision_object import YoloToCollisionObject


class AutonomousWorkspaceRecognition:
    """
    xArm6/RealSenseを動かしてワークスペースを自動認識する
    1. mapping_staticのジョイント値リストで動かして静的OctoMapを作成
    2. mapping_semi_staticのジョイント値リストで動かして半静的OctoMapを作成
    """

    def __init__(self):
        self.mapping = MoveItMapping()
        self.octomap2collision_object = OctoMap2CollisionObject()
        self.publish_octomap = PublishOctomapToMoveIt()
        self.collision_object_pub = PublishCollisionObjectToMoveIt(scene_topic="/planning_scene")
        self.yolo_to_co = YoloToCollisionObject()

    def main(self):
        # 1. move for mapping and create static octomap
        self.mapping.load_joint_values(map_name="mapping_static")
        self.mapping.execute_joint_trajectories()
        # 少し待機してOctoMapを安定させる
        rospy.sleep(2.0)
        # OctoMapをCollisionObjectに変換
        static_collision_objects = self.octomap2collision_object.convert_octomap_to_collision_objects(
            centers_topic="/octomap_static/octomap_point_cloud_centers",
            frame_id="map",
            merge_size=0.15,
            min_points=5,
            id_prefix="static",
            max_objects=500,
            safety_radius=0.15
        )
        # CollisionObjectをpublish
        self.collision_object_pub.publish(static_collision_objects)

        # 2. move for mapping and create semi-static octomap
        self.mapping.load_joint_values(map_name="mapping_semi_static")
        self.mapping.execute_joint_trajectories()
        # 少し待機してOctoMapを安定させる
        rospy.sleep(2.0)
        # OctoMapをCollisionObjectに変換
        semi_static_collision_objects = self.octomap2collision_object.convert_octomap_to_collision_objects(
            centers_topic="/octomap_semi_static/octomap_point_cloud_centers",
            frame_id="map",
            merge_size=0.06,
            min_points=10,
            id_prefix="semi_static",
            max_objects=500,
            safety_radius=0.15
        )
        # CollisionObjectをpublish
        self.collision_object_pub.publish(semi_static_collision_objects)

        # 3. 対象物をYOLOで検出してAttachCollisionObjectに変換
        self.mapping.load_joint_values(map_name="mapping_object")
        self.mapping.execute_joint_trajectories()
        # 少し待機してOctoMapを安定させる
        rospy.sleep(2.0)
        # YOLO→CO→Attachを実行
        # self.yolo_to_co.run()
        rospy.loginfo("Autonomous workspace recognition completed.")

    
if __name__ == "__main__":
    rospy.init_node("autonomous_workspace_recognition")
    awr = AutonomousWorkspaceRecognition()
    awr.main()