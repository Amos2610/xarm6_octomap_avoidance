#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PointStamped, Point
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject

import tf2_ros
import tf2_geometry_msgs
from octomap_msgs.srv import BoundingBoxQuery


MODEL_PATH = "yolo11n.pt"
DEVICE = "cpu"        # GPU使えるなら 0
IMG_SIZE = 640
CONF_THRES = 0.50


class YoloToCollisionObject:
    """
    YOLOで対象を検出 → Depthで3D化 → AABBを
      1) CollisionObject(Box) として PlanningScene に ADD（または）
      2) 同じ AABB で dynamic OctoMap を clear_bbx（必須）
      3) 必要に応じて AttachedCollisionObject としてロボットに ATTACH
    可視化は /yolo/annotated に出力
    """

    def __init__(self):
        # ---- params ----
        self.color_topic  = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic  = rospy.get_param("~depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.info_topic   = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_color_optical_frame")
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.scene_topic  = rospy.get_param("~scene_topic", "/planning_scene")

        # 検出クラスと公開挙動
        self.target_classes = rospy.get_param("~target_classes", ["bottle"])
        self.publish_world_co = rospy.get_param("~publish_world_co", False)  # True: まずWorldにADD
        self.attach_on_detect = rospy.get_param("~attach_on_detect", True)   # True: 検出直後にAttach

        # アタッチ先と接触許可
        self.attach_link = rospy.get_param("~attach_link", "link_eef")
        self.touch_links = rospy.get_param("~touch_links", [
            "xarm_gripper_base_link",
            "left_finger", "right_finger",
            "left_inner_knuckle", "right_inner_knuckle",
            "left_outer_knuckle", "right_outer_knuckle"
        ])

        # OctoMap クリアサービス
        self.clear_bbx_service = rospy.get_param("~clear_bbx_service",
                                                 "/octomap_dynamic/server_dynamic/clear_bbx")
        self.clear_expand = float(rospy.get_param("~clear_expand", 0.012))  # [m] 少し広めに消す

        # COサイズ調整
        self.co_scale = float(rospy.get_param("~co_scale", 1.0))  # COは5%縮める（接触性を上げる）

        # 3D化時のロバスト化
        self.roi_shrink = float(rospy.get_param("~roi_shrink", 0.85))
        self.depth_band = float(rospy.get_param("~depth_band", 0.08))
        self.inlier_k   = float(rospy.get_param("~inlier_k", 3.0))

        # ---- model / buffers ----
        self.model = YOLO(MODEL_PATH).to(DEVICE)
        self.bridge = CvBridge()
        self.depth_img = None
        self.K = None  # (fx, fy, cx, cy)

        # ---- TF buffer ----
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf)

        # ---- pubs/subs ----
        self.sub_color = rospy.Subscriber(self.color_topic, Image, self.cb_color, queue_size=1, buff_size=2**24)
        self.sub_depth = rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1, buff_size=2**24)
        self.sub_info  = rospy.Subscriber(self.info_topic, CameraInfo, self.cb_info, queue_size=1)

        self.pub_anno  = rospy.Publisher("/yolo/annotated", Image, queue_size=1)
        self.pub_scene = rospy.Publisher(self.scene_topic, PlanningScene, queue_size=10)

        # 連打抑止：直近にアタッチ済みのID
        self._attached_ids = set()

        rospy.loginfo("YoloToCollisionObject started. "
                      f"color={self.color_topic}, depth={self.depth_topic}, info={self.info_topic}, "
                      f"target_frame={self.target_frame}, attach_link={self.attach_link}")

    # ---------- subscribers ----------
    def cb_info(self, msg: CameraInfo):
        self.K = (msg.K[0], msg.K[4], msg.K[2], msg.K[5])  # fx, fy, cx, cy

    def cb_depth(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # 16UC1(mm) or 32FC1(m) を m に正規化
        self.depth_img = (img.astype(np.float32) / 1000.0) if img.dtype == np.uint16 else img.astype(np.float32)

    def cb_color(self, msg: Image):
        if self.K is None or self.depth_img is None:
            return  # 未同期

        if not self.tfbuf.can_transform(self.target_frame, self.camera_frame, rospy.Time(0), rospy.Duration(0.5)):
            rospy.logdebug("TF not ready %s -> %s", self.camera_frame, self.target_frame)
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model.predict(source=frame, imgsz=IMG_SIZE, conf=CONF_THRES, device=DEVICE, verbose=False)
        r = results[0]

        annotated = frame.copy()
        cos_world_add = []   # worldにADDするCO（必要時）
        acoes_attach  = []   # attach用のAttachedCollisionObject

        names = self.model.names  # id->label
        if r.boxes is None or len(r.boxes) == 0:
            # 可視化だけ流す
            out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out.header = msg.header
            self.pub_anno.publish(out)
            return

        xyxy = r.boxes.xyxy.cpu().numpy()
        conf = r.boxes.conf.cpu().numpy()
        cls  = r.boxes.cls.cpu().numpy().astype(int)

        det_total = len(xyxy)
        det_kept = 0
        det_aabb = 0
        det_attached = 0

        for i in range(det_total):
            label = names[int(cls[i])] if isinstance(names, (list, tuple)) else names.get(int(cls[i]), str(int(cls[i])))
            if self.target_classes and (label not in self.target_classes):
                continue
            det_kept += 1

            x1, y1, x2, y2 = xyxy[i].astype(int)
            score = float(conf[i])

            # AABB（map座標）を推定
            aabb = self.bbox3d_from_yolo(x1, y1, x2, y2,
                                         self.depth_img, self.K,
                                         self.camera_frame, self.target_frame)
            if aabb is None:
                rospy.logdebug("AABB None (depth/points/TF failed)")
                continue
            det_aabb += 1

            obj_id = f"{label}_{i}"

            # 1) OctoMapの該当領域をBBXクリア（少し広めに）
            self.try_clear_bbx(aabb)

            # 2) CollisionObject（ワールド or アタッチ）の作成
            co = self.make_co_from_aabb(aabb, obj_id=obj_id, frame_id=self.target_frame)

            if self.publish_world_co:
                # 古い同IDをワールドから除去 → 新規にADD
                cos_world_add.extend(self._replace_world_co(co))

            if self.attach_on_detect:
                if obj_id not in self._attached_ids:
                    aco = self._make_attached_from_co(co, link_name=self.attach_link, touch_links=self.touch_links)
                    acoes_attach.append(aco)
                    self._attached_ids.add(obj_id)
                    det_attached += 1

            # 可視化
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f"{label} {score:.2f}", (x1, max(0, y1-5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

        # 画像配信
        out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = msg.header
        self.pub_anno.publish(out)

        # PlanningSceneへ（diff）
        if cos_world_add or acoes_attach:
            ps = PlanningScene()
            ps.is_diff = True

            if cos_world_add:
                ps.world.collision_objects = cos_world_add

            if acoes_attach:
                # アタッチ時は、同IDのWorld COを念の為REMOVEして競合回避
                for aco in acoes_attach:
                    rm = CollisionObject()
                    rm.header.frame_id = aco.object.header.frame_id
                    rm.id = aco.object.id
                    rm.operation = CollisionObject.REMOVE
                    ps.world.collision_objects.append(rm)

                ps.robot_state.attached_collision_objects = acoes_attach

            self.pub_scene.publish(ps)
            rospy.loginfo("PS diff published: world_co=%d, attached=%d, kept=%d, aabb=%d",
                          len(cos_world_add), len(acoes_attach), det_kept, det_aabb)

    # ---------- helpers ----------
    def bbox3d_from_yolo(self, x1, y1, x2, y2, depth_img, K, src_frame, dst_frame):
        fx, fy, cx, cy = K

        # 1) ROI縮小
        w = x2 - x1; h = y2 - y1
        sx = int(w * (1.0 - self.roi_shrink) * 0.5)
        sy = int(h * (1.0 - self.roi_shrink) * 0.5)
        x1s, y1s = x1 + sx, y1 + sy
        x2s, y2s = x2 - sx, y2 - sy
        x1s = max(0, x1s); y1s = max(0, y1s)
        x2s = min(depth_img.shape[1]-1, x2s)
        y2s = min(depth_img.shape[0]-1, y2s)
        if x2s <= x1s or y2s <= y1s:
            return None

        # 2) 深度取り出し
        patch = depth_img[y1s:y2s+1, x1s:x2s+1].astype(np.float32)
        valid = np.isfinite(patch) & (patch > 0.05)
        if not np.any(valid):
            return None
        z_med = float(np.median(patch[valid]))
        zmin = z_med - self.depth_band
        zmax = z_med + self.depth_band
        band = valid & (patch >= zmin) & (patch <= zmax)
        if not np.any(band):
            band = valid

        # 3) 画素→カメラ3D
        ys, xs = np.where(band)
        us = (x1s + xs).astype(np.float32)
        vs = (y1s + ys).astype(np.float32)
        zs = patch[ys, xs]
        X = (us - cx) * zs / fx
        Y = (vs - cy) * zs / fy
        Z = zs

        # 4) 外れ値除去（MAD）
        pts = np.stack([X, Y, Z], axis=1)
        med = np.median(pts, axis=0)
        mad = np.median(np.abs(pts - med), axis=0) + 1e-6
        inlier = np.all(np.abs(pts - med) <= self.inlier_k * mad, axis=1)
        pts = pts[inlier]
        if pts.shape[0] < 10:
            return None

        # 5) カメラ座標のAABB → mapへTF
        min_cam = pts.min(axis=0); max_cam = pts.max(axis=0)

        if not self.tfbuf.can_transform(dst_frame, src_frame, rospy.Time(0), rospy.Duration(0.5)):
            return None

        def tf_point_cam_to(dst, xyz):
            p = PointStamped()
            p.header.frame_id = src_frame
            p.header.stamp = rospy.Time(0)
            p.point.x, p.point.y, p.point.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
            return self.tfbuf.transform(p, dst, rospy.Duration(0.5))

        try:
            pmin = tf_point_cam_to(dst_frame, min_cam)
            pmax = tf_point_cam_to(dst_frame, max_cam)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            return None

        minx = min(pmin.point.x, pmax.point.x); maxx = max(pmin.point.x, pmax.point.x)
        miny = min(pmin.point.y, pmax.point.y); maxy = max(pmin.point.y, pmax.point.y)
        minz = min(pmin.point.z, pmax.point.z); maxz = max(pmin.point.z, pmax.point.z)
        return (minx, miny, minz, maxx, maxy, maxz)

    def make_co_from_aabb(self, aabb, obj_id="target_0", frame_id="map"):
        (minx, miny, minz, maxx, maxy, maxz) = aabb
        cx = 0.5*(minx+maxx); cy = 0.5*(miny+maxy); cz = 0.5*(minz+maxz)
        sx = (maxx-minx) * self.co_scale
        sy = (maxy-miny) * self.co_scale
        sz = (maxz-minz) * self.co_scale
        minx2, maxx2 = cx - sx/2.0, cx + sx/2.0
        miny2, maxy2 = cy - sy/2.0, cy + sy/2.0
        minz2, maxz2 = cz - sz/2.0, cz + sz/2.0

        co = CollisionObject()
        co.header.frame_id = frame_id
        co.id = obj_id

        box = SolidPrimitive(); box.type = SolidPrimitive.BOX
        box.dimensions = [maxx2-minx2, maxy2-miny2, maxz2-minz2]

        pose = Pose(); pose.orientation.w = 1.0
        pose.position.x, pose.position.y, pose.position.z = cx, cy, cz

        co.primitives = [box]
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD
        return co

    def _replace_world_co(self, co):
        """
        同一IDの古いワールドCOをREMOVEし、その後ADDする2メッセージを返す。
        """
        rm = CollisionObject()
        rm.header.frame_id = co.header.frame_id
        rm.id = co.id
        rm.operation = CollisionObject.REMOVE
        return [rm, co]

    def _make_attached_from_co(self, co, link_name, touch_links):
        aco = AttachedCollisionObject()
        aco.link_name = link_name
        aco.object = co
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = touch_links if isinstance(touch_links, list) else [touch_links]
        return aco

    def try_clear_bbx(self, aabb):
        (minx, miny, minz, maxx, maxy, maxz) = aabb
        # クリアは少しだけ広げる
        minx -= self.clear_expand; miny -= self.clear_expand; minz -= self.clear_expand
        maxx += self.clear_expand; maxy += self.clear_expand; maxz += self.clear_expand
        srv = self.clear_bbx_service
        try:
            rospy.wait_for_service(srv, timeout=0.5)
            proxy = rospy.ServiceProxy(srv, BoundingBoxQuery)
            req = BoundingBoxQuery._request_class()
            req.min = Point(minx, miny, minz); req.max = Point(maxx, maxy, maxz)
            resp = proxy(req)
            rospy.logdebug("clear_bbx ok: %s", getattr(resp, "success", True))
            return True
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"clear_bbx failed: {e}")
            return False

    def run(self):
        rospy.loginfo("YOLO → CO(+clear_bbx) → Attach node started.")
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("yolo_to_collision_object")
    node = YoloToCollisionObject()
    node.run()