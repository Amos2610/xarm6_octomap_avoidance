#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

MODEL_PATH = "yolo11n.pt"
DEVICE = 'cpu'  # GPU使えるなら 0
IMG_SIZE = 640
CONF_THRES = 0.50   

class YoloNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_PATH).to(DEVICE)
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.cb, queue_size=1, buff_size=2**24)
        self.pub = rospy.Publisher("/yolo/annotated", Image, queue_size=1)
        self.last = rospy.Time.now()

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model.predict(source=frame, imgsz=IMG_SIZE, conf=CONF_THRES, device=DEVICE, verbose=False)
        annotated = results[0].plot()
        # # 画面表示（任意）
        # cv2.imshow("YOLO ROS", annotated); cv2.waitKey(1)
        # 配信（任意）
        out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = msg.header
        self.pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("yolo_realsense_node")
    YoloNode()
    rospy.loginfo("YOLO node started.")
    rospy.spin()
    cv2.destroyAllWindows()
