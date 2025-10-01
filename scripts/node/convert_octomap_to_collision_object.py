#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class OctoMap2CollisionObject:
    """
    OctoMap (occupied voxel centers as PointCloud2) を
    CollisionObject のリストに変換するクラス
    """

    def __init__(self):
        rospy.loginfo("OctoMap2CollisionObject initialized")

    def convert_octomap_to_collision_objects(self,
                                             centers_topic: str,
                                             frame_id="map",
                                             merge_size=0.20,
                                             min_points=30,
                                             id_prefix="octomap",
                                             max_objects=500,
                                             safety_radius=0.15):
        """
        OctoMap(PointCloud2)をCollisionObjectのリストに変換する
        """
        # timeout付きで1回だけ受信
        try:
            msg = rospy.wait_for_message(centers_topic, PointCloud2, timeout=10.0)
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to receive PointCloud2 from {centers_topic}: {e}")
            return []

        rospy.loginfo(f"Converting {id_prefix} octomap to collision objects...")

        m = float(merge_size)
        grid = {}  # (i,j) -> [minz, maxz, count]

        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            i = int((x) / m)
            j = int((y) / m)
            key = (i, j)
            if key not in grid:
                grid[key] = [z, z, 1]
            else:
                g = grid[key]
                if z < g[0]:
                    g[0] = z
                if z > g[1]:
                    g[1] = z
                g[2] += 1

        collision_objects = []
        idx = 0
        for (i, j), (minz, maxz, cnt) in grid.items():
            if cnt < min_points:
                continue
            if idx >= max_objects:
                break

            cx = (i + 0.5) * m
            cy = (j + 0.5) * m
            height = max(maxz - minz, m)
            cz = 0.5 * (minz + maxz)

            # ロボット周辺のフィルタリング
            if (cx**2 + cy**2)**0.5 < safety_radius:
                continue

            co = CollisionObject()
            co.header.frame_id = frame_id
            co.id = f"{id_prefix}_{idx}"

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [m, m, height]

            pose = Pose()
            pose.orientation.w = 1.0
            pose.position.x = cx
            pose.position.y = cy
            pose.position.z = cz

            co.primitives = [box]
            co.primitive_poses = [pose]
            co.operation = CollisionObject.ADD

            collision_objects.append(co)
            idx += 1

            rospy.loginfo("Grid cells: %d", len(grid))

        return collision_objects


if __name__ == "__main__":
    rospy.init_node("octomap_to_collision_object")

    centers_topic = rospy.get_param("~centers_topic", "/octomap_static/octomap_point_cloud_centers")
    frame_id      = rospy.get_param("~frame_id", "map")
    merge_size    = float(rospy.get_param("~merge_size", 0.15))
    min_points    = int(rospy.get_param("~min_points", 15))
    id_prefix     = rospy.get_param("~id_prefix", "static")
    pub_topic     = "/collision_objects"
    max_objects   = int(rospy.get_param("~max_objects", 500))
    safety_radius = float(rospy.get_param("~safety_radius", 0.15))

    pub = rospy.Publisher(pub_topic, CollisionObject, queue_size=50)
    converter = OctoMap2CollisionObject()

    # OctoMap完成後のPointCloud2を1回受け取る
    collision_objects = converter.convert_octomap_to_collision_objects(
        centers_topic=centers_topic,
        frame_id=frame_id,
        merge_size=merge_size,
        min_points=min_points,
        id_prefix=id_prefix,
        max_objects=max_objects,
        safety_radius=safety_radius
    )

    for co in collision_objects:
        pub.publish(co)
    rospy.loginfo("Published %d CollisionObjects (prefix=%s)", len(collision_objects), id_prefix)
