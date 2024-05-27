import tf.transformations as tf_trans
import numpy as np
from geometry_msgs.msg import Pose

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)  # noqa: E501
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)  # noqa: E501
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)  # noqa: E501
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)  # noqa: E501

    return [qx, qy, qz, qw]

def pose_to_homogeneous_matrix(pose):
    position = pose.position
    orientation = pose.orientation

    trans = tf_trans.translation_matrix([position.x, position.y, position.z])
    quat = tf_trans.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])

    return np.dot(trans, quat)

def homogeneous_matrix_to_pose(matrix):
    trans = tf_trans.translation_from_matrix(matrix)
    quat = tf_trans.quaternion_from_matrix(matrix)

    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose

def compute_object_pose_in_world(pose_vehicle_in_world, pose_object_in_camera):
    T_vehicle_in_world = pose_to_homogeneous_matrix(pose_vehicle_in_world)
    T_object_in_camera = pose_to_homogeneous_matrix(pose_object_in_camera)

    T_object_in_world = np.dot(T_vehicle_in_world, T_object_in_camera)

    pose_object_in_world = homogeneous_matrix_to_pose(T_object_in_world)

    return pose_object_in_world
