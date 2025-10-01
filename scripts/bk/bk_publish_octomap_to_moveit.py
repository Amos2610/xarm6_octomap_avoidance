#!/usr/bin/env python3
import rospy
from moveit_msgs.msg import PlanningScene
from octomap_msgs.msg import OctomapWithPose, Octomap
from std_msgs.msg import Header

class PublishOctomapToMoveIt:
    def __init__(self):
        self.scene_topic  = rospy.get_param('~scene_topic',  '/planning_scene')

        # Publisher (共通)
        self.pub = rospy.Publisher(self.scene_topic, PlanningScene, queue_size=10)

        # レイヤー設定を読み込み
        layers = rospy.get_param('~layers', [])
        rospy.loginfo(f"Loaded layers config: {layers}")
        if not layers:
            rospy.logwarn("No layers defined. Nothing will be published.")
            exit(0)
        else:
            rospy.loginfo(f"Configuring {len(layers)} layers")

        # レイヤーごとの状態を保持
        self.layers = []
        for layer in layers:
            name   = layer.get("name", "unnamed")
            topic  = layer.get("source_topic", "/octomap_full")
            period = float(layer.get("update_rate_seconds", 1.0))  # 秒周期
            publish_on_change = bool(layer.get("publish_on_change_only", True))

            state = {
                "name": name,
                "topic": topic,
                "period": period,
                "publish_on_change": publish_on_change,
                "latest_owp": None,
                "latest_rx_stamp": rospy.Time(0),
                "last_pub_rx_stamp": rospy.Time(0),
            }
            # Subscriber
            rospy.Subscriber(topic, Octomap, self._make_callback(state), queue_size=1)
            # Timer
            rospy.Timer(rospy.Duration(period), self._make_timer(state), oneshot=False)

            self.layers.append(state)
            rospy.loginfo(f"[{name}] subscribed to {topic}, update every {period:.2f}s")

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

    def _make_callback(self, state):
        def cb(msg):
            owp = self.octomap_to_octomap_with_pose(msg)
            state["latest_owp"] = owp
            state["latest_rx_stamp"] = rospy.Time.now()
        return cb

    def _make_timer(self, state):
        def timer_cb(_evt):
            if state["latest_owp"] is None:
                return
            if state["publish_on_change"] and (state["latest_rx_stamp"] <= state["last_pub_rx_stamp"]):
                return

            owp = state["latest_owp"]
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
            state["last_pub_rx_stamp"] = state["latest_rx_stamp"]
            rospy.logdebug(f"[{state['name']}] Published octomap to MoveIt")
        return timer_cb

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('publish_octomap_to_moveit')
    try:
        node = PublishOctomapToMoveIt()
        node.run()
    except rospy.ROSInterruptException:
        pass
