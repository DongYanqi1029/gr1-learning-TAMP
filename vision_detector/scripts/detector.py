#! /usr/bin python

import rospy, rospkg
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import torch
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from vision_detector.msg import Detection, DetectionArray, ObjectPose, ObjectPoseArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import sensor_msgs.point_cloud2 as pc2
import message_filters
import struct
import tf
import tf.transformations as tf_trans
import vision_detector.aux as aux
import time
from itertools import islice

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

class YoloDetector():
    def __init__(self):
        rospy.init_node('yolo_detector', log_level=rospy.INFO)

        self.bridge = CvBridge()

        rospy.loginfo("Loading Yolov5 model...")
        # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', device='cpu')
        self.model_name = "yolov5n.pt"
        self.model_path = rospkg.RosPack().get_path('vision_detector') + '/models/' + self.model_name
        self.yolo_path = rospkg.RosPack().get_path('vision_detector') + '/yolov5/'
        self.model = torch.hub.load(self.yolo_path, 'custom', path=self.model_path, device=device, force_reload=True, source='local')
        self.names = self.model.names
        rospy.loginfo("Finish loading Yolov5 model!")

        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)

        self.img_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        self.pc_sub = message_filters.Subscriber('/camera/depth/points', PointCloud2)
        self.depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        
        ts = message_filters.ApproximateTimeSynchronizer([self.img_sub, self.pc_sub, self.depth_sub], 10, 1, allow_headerless=True)
        ts.registerCallback(self.callback)

        self.img_pub = rospy.Publisher('/yolo_detector/annotations', Image, queue_size=30)
        self.dct_pub = rospy.Publisher('/yolo_detector/detections', DetectionArray, queue_size=30)
        self.objectpose_pub = rospy.Publisher('/yolo_detector/object_poses', ObjectPoseArray, queue_size=30)

    # Camera calbback for both object detection and pose estimation
    def callback(self, image_sub: Image, pc_sub: PointCloud2, depth_sub: Image):
        cb_start_time = time.time()
        yolo_start_time = time.time() 
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(image_sub, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_sub, "32FC1")  
        except CvBridgeError as e:
            rospy.logerr(e)

        # cv_image = cv2.imread('/home/dyq/catkin_ws/src/vision_detector/yolov5/data/images/zidane.jpg')

        results = self.model(rgb_image)
        output_img = results.render()[0]
        output_bbox = results.xyxy[0].cpu().numpy() # xmin    ymin    xmax   ymax  confidence  class    name

        # # Get point cloud 2 data
        # pc_data = pc2.read_points(pc_sub, skip_nans=False, field_names={'x', 'y', 'z'})

        # Generage messages for publication
        # Annotated image
        try:
            output_img_msg = self.bridge.cv2_to_imgmsg(output_img, "bgr8")
            output_img_msg.header = image_sub.header
        except CvBridgeError as e:
            rospy.logerr(e)

        yolo_end_time = time.time()

        pose_start_time = time.time()
        # Detection bounding boxes 
        # Estimate object pose relative to the world
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

            bbox = [int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])]
            objectpose.name = self.names[int(Cls)]
            objectpose.confidence = conf
            # pose = self.estimate_pose_pc(bbox, pc_sub) 
            pose = self.estimate_pose_depth(bbox, depth_image) 
            objectpose.isValid = not pose is None
            if pose is None:
                objectpose.pose = Pose()
            else:
                objectpose.pose = pose

            dect_array.detections.append(dect)
            objectpose_array.objects.append(objectpose)

        pose_end_time = time.time()

        # Publish messages
        self.img_pub.publish(output_img_msg)
        self.dct_pub.publish(dect_array)
        self.objectpose_pub.publish(objectpose_array)

        cb_end_time = time.time()
        print("callback time:{}, yolo time:{}, pose time:{}".format(cb_end_time-cb_start_time, yolo_end_time-yolo_start_time, pose_end_time-pose_start_time))

    # Generate relative pose of the object to the vehicle
    def estimate_pose_pc(self, bbox, pc_sub):
        u = (bbox[0]+bbox[2])//2
        v = (bbox[1]+bbox[3])//2

        # point_idx = v * image_shape[1] + u

        # Get object pose from point cloud data
        step = 10
        sample_raidus = 1
        points = []
        for dx in range(-(sample_raidus * step), (sample_raidus * step)+1, step):
            for dy in range(-(sample_raidus * step), (sample_raidus * step)+1, step):
                if (u+dx) >= bbox[0] and (u+dx) <= bbox[2] and (v+dy) >= bbox[1] and (v+dy) <= bbox[3]:
                    points.append([u+dx, v+dy])


        points = np.array(points).tolist()
        vectors_3D = []

        try:
            # Get the corresponding 3D location of sampled points
            for pt_count, dt in enumerate(pc2.read_points(pc_sub, field_names={'x', 'y', 'z'}, skip_nans=False, uvs=points)):
                # If any point returns nan, skip
                if np.any(np.isnan(dt)):
                    pass
                else:
                    vectors_3D.append(dt)
        except struct.error as err:
            rospy.loginfo(err)

        if len(vectors_3D) == 0:
            return None

        position_3D = np.mean(np.array(vectors_3D), axis=0)

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

        # Get object pose in world
        pose_object_in_world = self.compute_pose_object_in_world(pose_object_in_camera)

        return pose_object_in_world

    def estimate_pose_depth(self, bbox, depth_image):
        u = (bbox[0]+bbox[2])//2
        v = (bbox[1]+bbox[3])//2

        # point_idx = v * image_shape[1] + u

        # Get object pose from point cloud data
        step = 10
        sample_raidus = 2
        points = []
        for dx in range(-(sample_raidus * step), (sample_raidus * step)+1, step):
            for dy in range(-(sample_raidus * step), (sample_raidus * step)+1, step):
                if (u+dx) >= 0 and (u+dx) < depth_image.shape[1] and (v+dy) >= 0 and (v+dy) <= depth_image.shape[0]:
                    if not np.isnan(depth_image[v+dy, u+dx]):
                        points.append([u+dx, v+dy, depth_image[v+dy, u+dx]])

        # No useful pixel in sampled points
        if len(points) == 0:
            return None

        point = np.mean(np.array(points), axis=0)
        x = int(point[0])
        y = int(point[1])
        depth = point[2]

        # Get camera inner parameters
        camera_info = rospy.wait_for_message("/camera/rgb/camera_info", CameraInfo, timeout=3)
        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]

        # Object pose in camera
        X = (x - cx) * depth / fx
        Y = (y - cy) * depth / fy
        Z = depth

        # Get object pose in camera
        pose_object_in_camera = Pose()
        pose_object_in_camera.position.x = X
        pose_object_in_camera.position.y = Y
        pose_object_in_camera.position.z = Z
        # Assume the orientation is not needed
        pose_object_in_camera.orientation.x = 0
        pose_object_in_camera.orientation.y = 0
        pose_object_in_camera.orientation.z = 0
        pose_object_in_camera.orientation.w = 1

        # Get object pose in world
        pose_object_in_world = self.compute_pose_object_in_world(pose_object_in_camera)

        return pose_object_in_world

    def compute_pose_object_in_world(self, pose_object_in_camera:Pose)->Pose:
        pose_vehicle_in_world = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped).pose.pose
        pose_vehicle_in_world = aux.pose_msg_to_pose(pose_vehicle_in_world)
        T_pose_vehicle_in_world = aux.pose_to_matrix(pose_vehicle_in_world[0], pose_vehicle_in_world[1])

        target_frame = "base_footprint"
        source_frame = "camera_rgb_optical_frame"
        pose_camera_in_vehicle = aux.get_pose_from_tf(self.tf_listener, target_frame, source_frame)
        T_pose_camera_in_vehicle = aux.pose_to_matrix(pose_camera_in_vehicle[0], pose_camera_in_vehicle[1])

        pose_object = aux.pose_msg_to_pose(pose_object_in_camera)
        T_pose_object_in_camera = aux.pose_to_matrix(pose_object[0], pose_object[1])

        T_pose_object_in_world = np.dot(np.dot(T_pose_vehicle_in_world, T_pose_camera_in_vehicle), T_pose_object_in_camera)
        trans, rot = aux.matrix_to_pose(T_pose_object_in_world)
        pose_object_in_world = aux.pose_to_pose_msg(trans, rot)

        return pose_object_in_world

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    detector = YoloDetector()
    detector.run()