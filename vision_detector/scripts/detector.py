#! /usr/bin python

import rospy 
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
from sensor_msgs.msg import Image, PointCloud2
from vision_detector.msg import Detection, DetectionArray
import sensor_msgs.point_cloud2 as pc2
import message_filters

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
        self.names = self.model.names
        rospy.loginfo("Finish loading Yolov5 model!")

        self.img_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        self.pc_sub = message_filters.Subscriber('/camera/depth/points', PointCloud2)
        
        ts = message_filters.ApproximateTimeSynchronizer([self.img_sub, self.pc_sub], 10, 1, allow_headerless=True)
        ts.registerCallback(self.callback)

        self.img_pub = rospy.Publisher('/yolo_detector/annotations', Image, queue_size=30)
        self.dct_pub = rospy.Publisher('/yolo_detector/detections', DetectionArray, queue_size=30)
        # self.pose_pub = rospy.Publisher('/yolo_detector/object_pose', ObjectPose, queue_size=30)


    def callback(self, image_sub: Image, pc_sub: PointCloud2):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_sub, "bgr8")    
        except CvBridgeError as e:
            print(e)

        # cv_image = cv2.imread('/home/dyq/catkin_ws/src/vision_detector/yolov5/data/images/zidane.jpg')

        results = self.model(cv_image)

        output_img = results.render()[0]
        # xmin    ymin    xmax   ymax  confidence  class    name
        output_bbox = results.xyxy[0].cpu().numpy()

        # Generage messages for publication
        try:
            output_img_msg = self.bridge.cv2_to_imgmsg(output_img, "bgr8")
            output_img_msg.header = image_sub.header
        except CvBridgeError as e:
            print(e)

        dect_array = DetectionArray()
        dect_array.header = image_sub.header
        for *xyxy, conf, Cls in output_bbox:
            dect = Detection()
            dect.Class = int(Cls)
            dect.name = self.names[int(Cls)]
            dect.confidence = conf
            dect.xmin = int(xyxy[0])
            dect.ymin = int(xyxy[1])
            dect.xmax = int(xyxy[2])
            dect.ymax = int(xyxy[3])

            dect_array.detections.append(dect)

        # Publish messages
        self.img_pub.publish(output_img_msg)
        self.dct_pub.publish(dect_array)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    detector = YoloDetector()
    detector.run()