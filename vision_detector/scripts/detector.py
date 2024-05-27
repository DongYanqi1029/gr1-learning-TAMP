#! /usr/bin python

import rospy 
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from vision_detector.msg import Detection, DetectionArray, ObjectPose, ObjectPoseArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import sensor_msgs.point_cloud2 as pc2
import message_filters
import struct
import tf.transformations as tf_trans
import vision_detector.aux as aux

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

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
        # self.depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        
        ts = message_filters.ApproximateTimeSynchronizer([self.img_sub, self.pc_sub], 10, 1, allow_headerless=True)
        ts.registerCallback(self.callback)

        self.img_pub = rospy.Publisher('/yolo_detector/annotations', Image, queue_size=30)
        self.dct_pub = rospy.Publisher('/yolo_detector/detections', DetectionArray, queue_size=30)
        self.objectpose_pub = rospy.Publisher('/yolo_detector/object_poses', ObjectPoseArray, queue_size=30)


    def callback(self, image_sub: Image, pc_sub: PointCloud2):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(image_sub, "bgr8")
            # depth_image = self.bridge.imgmsg_to_cv2(depth_sub, "32FC1")  
        except CvBridgeError as e:
            rospy.logerr(e)

        # cv_image = cv2.imread('/home/dyq/catkin_ws/src/vision_detector/yolov5/data/images/zidane.jpg')

        results = self.model(rgb_image)

        output_img = results.render()[0]
        output_bbox = results.xyxy[0].cpu().numpy() # xmin    ymin    xmax   ymax  confidence  class    name

        # Generage messages for publication

        # Annotated image
        try:
            output_img_msg = self.bridge.cv2_to_imgmsg(output_img, "bgr8")
            output_img_msg.header = image_sub.header
        except CvBridgeError as e:
            rospy.logerr(e)

        # Detection bounding boxes 
        # Estimated object pose relative to the world
        dect_array = DetectionArray()
        dect_array.header = image_sub.header
        dect_array.detections = []

        objectpose_array = ObjectPoseArray()
        objectpose_array.header = pc_sub.header
        objectpose_array.objects = []

        for *xyxy, conf, Cls in output_bbox:
            dect = Detection()
            objectpose = ObjectPose()

            dect.Class = int(Cls)
            dect.name = self.names[int(Cls)]
            dect.confidence = conf
            dect.xmin = int(xyxy[0])
            dect.ymin = int(xyxy[1])
            dect.xmax = int(xyxy[2])
            dect.ymax = int(xyxy[3])

            bbox_center = [(dect.xmin+dect.xmax)//2, (dect.ymin+dect.ymax)//2]
            objectpose.name = self.names[int(Cls)]
            objectpose.confidence = conf
            objectpose.pose = self.estimate_pose(bbox_center, pc_sub, rgb_image.shape[1]) 

            dect_array.detections.append(dect)
            objectpose_array.objects.append(objectpose)

        # Publish messages
        self.img_pub.publish(output_img_msg)
        self.dct_pub.publish(dect_array)
        self.objectpose_pub.publish(objectpose_array)

    def estimate_pose(self, bbox_center, pc_sub:PointCloud2, image_width):
        # Generate relative pose of the object to the vehicle
        u = bbox_center[0]
        v = bbox_center[1]

        point_idx = v * image_width + u
        position_3D = list(pc2.read_points(pc_sub, skip_nans=True, field_names={'x', 'y', 'z'}))[point_idx]

        # Get object pose in camera
        pose_object_in_camera = Pose()
        pose_object_in_camera.position.x = position_3D[0]
        pose_object_in_camera.position.y = position_3D[1]
        pose_object_in_camera.position.z = position_3D[2]
        # Assume the orientation is not needed
        pose_object_in_camera.orientation.x = 0
        pose_object_in_camera.orientation.y = 0
        pose_object_in_camera.orientation.z = 0
        pose_object_in_camera.orientation.w = 1

        # Get vehicle pose in world from amcl
        vehicle_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped).pose.pose

        # Get object pose in world
        pose_object_in_world = aux.compute_object_pose_in_world(vehicle_pose, pose_object_in_camera)

        return pose_object_in_world

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    detector = YoloDetector()
    detector.run()