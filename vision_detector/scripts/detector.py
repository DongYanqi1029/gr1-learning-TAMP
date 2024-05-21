#! /usr/bin python

import rospy 
import os
import cv2
import tf
from cv_bridge import CvBridge, CvBridgeError
import torch
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from vision_detector.msg import Detection, DetectionArray
from geometry_msgs.msg import Pose, PoseArray, Point
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
        self.pose_pub = rospy.Publisher('/yolo_detector/object_pose', PoseArray, queue_size=30)


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

        # Annotated image
        try:
            output_img_msg = self.bridge.cv2_to_imgmsg(output_img, "bgr8")
            output_img_msg.header = image_sub.header
        except CvBridgeError as e:
            print(e)

        # Detection bounding boxes 
        # Estimated object pose relative to the world
        dect_array = DetectionArray()
        dect_array.header = image_sub.header

        pose_array = PoseArray()
        pose_array.header = image_sub.header

        for *xyxy, conf, Cls in output_bbox:
            dect = Detection()
            pose = Pose()

            dect.Class = int(Cls)
            dect.name = self.names[int(Cls)]
            dect.confidence = conf
            dect.xmin = int(xyxy[0])
            dect.ymin = int(xyxy[1])
            dect.xmax = int(xyxy[2])
            dect.ymax = int(xyxy[3])

            pose = self.estimate_pose(xyxy, pc_sub)

            dect_array.detections.append(dect)
            pose_array.poses.append(pose)

        # Publish messages
        self.img_pub.publish(output_img_msg)
        self.dct_pub.publish(dect_array)
        self.pose_pub.publishe(pose_array)

    def estimate_pose(self, bbox, pc_sub:PointCloud2):
        bbox = np.array(bbox)

        # Compute the center, the mid point of the right
        # and top segment of the bounding box
        c = (bbox[0] + bbox[2]) // 2
        x = (bbox[2] + bbox[3]) // 2
        y = (bbox[0] + bbox[3]) // 2

        points = np.array([c, x, y]).tolist()
        vectors_3D = np.zeros((3, 3))

        try:
            # Get the corresponding 3D location of c, x, y
            for pt_count, dt in enumerate(pc2.read_points(pc_sub, field_names={'x', 'y', 'z'}, skip_nans=False, uvs=points)):
                # If any point returns nan, return
                if np.any(np.isnan(dt)):
                    if object_name in self.object_pose_info.keys():
                        del self.object_pose_info[object_name]
                    rospy.loginfo('No corresponding 3D point found')
                    return None
                else:
                    vectors_3D[pt_count] = dt
                    if pt_count == 2:
                        self.vectors_3D = vectors_3D
        except struct.error as err:
            rospy.loginfo(err)
            return None

        try:
            # 3D position of the object
            c_3D = self.vectors_3D[0]

            # Center the vectors to the origin
            x_vec = self.vectors_3D[1] - c_3D
            x_vec /= norm(x_vec)

            y_vec = self.vectors_3D[2] - c_3D
            y_vec /= norm(y_vec)
            # Take the cross product of x and y vector
            # to generate z vector.
            z_vec = np.cross(x_vec, y_vec)
            z_vec /= norm(z_vec)

            # Recompute x vector to make it truly orthognal
            x_vec_orth = np.cross(y_vec, z_vec)
            x_vec_orth /= norm(x_vec_orth)

        except RuntimeWarning as w:
            rospy.loginfo(w)
            return None

        # if self.viz_pose:
        #     self.draw_pose(object_name, np.vstack((self.vectors_3D, z_vec)))

        # Compute Euler angles i.e. roll, pitch, yaw
        roll = np.arctan2(y_vec[2], z_vec[2])
        pitch = np.arctan2(-x_vec_orth[2], np.sqrt(1 - x_vec_orth[2]**2))
        # pitch = np.arcsin(-x_vec_orth[2])
        yaw = np.arctan2(x_vec_orth[1], x_vec_orth[0])

        [qx, qy, qz, qw] = self.euler_to_quaternion(roll, pitch, yaw)

        # Generate relative pose to the vehicle
        rel_pose = Pose()

        rel_pose.position.x = c_3D[0]
        rel_pose.position.y = c_3D[1]
        rel_pose.position.z = c_3D[2]
        # Make sure the quaternion is valid and normalized
        rel_pose.orientation.x = qx
        rel_pose.orientation.y = qy
        rel_pose.orientation.z = qz
        rel_pose.orientation.w = qw

        rel_quat = (qx, qy, qz, qw)

        # self.object_pose_info[object_name] = {
        #         'position': c_3D.tolist(),
        #         'orientation': [qx, qy, qz, qw]
        #     }

        # get vehicle pose from amcl
        vehicle_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped).pose.pose

        vehicle_quat = (
            vehicle_pose.orientation.x,
            vehicle_pose.orientation.y,
            vehicle_pose.orientation.z,
            vehicle_pose.orientation.w
        )

        # 旋转rel_pose的位置
        rel_pose_transformed = tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(vehicle_quat, (rel_pose.position.x, rel_pose.position.y, rel_pose.position.z, 0)),
            tf.transformations.quaternion_conjugate(vehicle_quat)
        )

        # 加上vehicle_pose的位置得到object在世界坐标系下的位置
        position_world = Point()
        position_world.x = rel_pose_transformed[0] + vehicle_pose.position.x
        position_world.y = rel_pose_transformed[1] + vehicle_pose.position.y
        position_world.z = rel_pose_transformed[2] + vehicle_pose.position.z

        # 组合旋转部分
        orientation_world = tf.transformations.quaternion_multiply(vehicle_quat, rel_quat)

        # 得到object在世界坐标系下的最终位姿
        pose_world = Pose()
        pose_world.position = position_world
        pose_world.orientation.x = orientation_world[0]
        pose_world.orientation.y = orientation_world[1]
        pose_world.orientation.z = orientation_world[2]
        pose_world.orientation.w = orientation_world[3]

        return pose_world

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)  # noqa: E501
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)  # noqa: E501
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)  # noqa: E501
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)  # noqa: E501

        return [qx, qy, qz, qw]

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    detector = YoloDetector()
    detector.run()