#! /usr/bin python

import rospy 
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
from sensor_msgs.msg import Image

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class YoloDetector():
    def __init__(self):
        rospy.init_node('yolo_detector')

        self.bridge = CvBridge()

        rospy.loginfo("Loading Yolov5 model...")
        # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', device='cpu')
        self.model_path = os.environ['HOME'] + '/catkin_ws/src/vision_detector/models/yolov5s.pt'
        self.yolo_path = os.environ['HOME'] + '/catkin_ws/src/vision_detector/yolov5'
        self.model = torch.hub.load(self.yolo_path, 'custom', path=self.model_path, device=device, force_reload=True, source='local')
        rospy.loginfo("Finish loading Yolov5 model!")

        self.img_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.detect_cb)
        self.dct_pub = rospy.Publisher('/yolo_detector/detections', Image, queue_size=30)


    def detect_cb(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")    
        except CvBridgeError as e:
            print(e)

        # cv_image = cv2.imread('/home/dyq/catkin_ws/src/vision_detector/yolov5/data/images/zidane.jpg')

        results = self.model(cv_image)

        output_img = results.render()[0]

        try:
            output_msg = self.bridge.cv2_to_imgmsg(output_img, "bgr8")
            self.dct_pub.publish(output_msg)
        except CvBridgeError as e:
            print(e)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    detector = YoloDetector()
    detector.run()