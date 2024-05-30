import tf.transformations as tf_trans
import tf
import numpy as np
from geometry_msgs.msg import Pose
import time 
import rospy

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)  # noqa: E501
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)  # noqa: E501
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)  # noqa: E501
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)  # noqa: E501

    return [qx, qy, qz, qw]

def pose_to_pose_msg(trans, rot):
    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]

    return pose

def pose_msg_to_pose(pose):
    trans_vec = np.array([pose.position.x, pose.position.y, pose.position.z])
    rot_vec = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    return (trans_vec, rot_vec)

# Transform pose vector to 4x4 transformation matrix
def pose_to_matrix(trans_vec, quat_vec):
    trans_mat = tf_trans.translation_matrix(trans_vec)
    quat_mat = tf_trans.quaternion_matrix(quat_vec)

    return np.dot(trans_mat, quat_mat)

# Transform 4x4 transformation matrix to pose vector
def matrix_to_pose(matrix):
    trans_vec = tf_trans.translation_from_matrix(matrix)
    quat_vec = tf_trans.quaternion_from_matrix(matrix)

    return (trans_vec, quat_vec)

def get_pose_from_tf(listener, target_frame, source_frame):
    try:
        trans_vec, rot_vec = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
        return (np.array(trans_vec), np.array(rot_vec))
    except:
        rospy.logerr("Camera transform lookup failed!")
        return (None, None)


# def compute_object_pose_transformed_from_tf(target_frame, source_frame, pose_object_in_source_frame):
    
#     rot_mat, trans_vec = get_transform(listener, target_frame, source_frame)

#     if not rot_mat is None and not trans_vec is None:
#         x = pose_object_in_source_frame.position.x
#         y = pose_object_in_source_frame.position.y
#         z = pose_object_in_source_frame.position.z

#         # Assume orientation of object in target frame is not needed
#         pose_xyz_object_in_target_frame = rot_mat.dot(np.array[x, y, z]) + trans_vec
#         pose_object_in_target_frame = Pose()
#         pose_object_in_target_frame.position.x = pose_xyz_object_in_target_frame[0]
#         pose_object_in_target_frame.position.y = pose_xyz_object_in_target_frame[0]
#         pose_object_in_target_frame.position.z = pose_xyz_object_in_target_frame[0]
#         pose_object_in_target_frame.orientation.x = 0
#         pose_object_in_target_frame.orientation.y = 0
#         pose_object_in_target_frame.orientation.z = 0
#         pose_object_in_target_frame.orientation.w = 1

#         return pose_object_in_target_frame

#     else:
#         rospy.logerr("Fail to get transform for object from {} to {}".format(source_frame, target_frame))
#         return None

    # T_vehicle_in_world = pose_to_homogeneous_matrix(pose_vehicle_in_world)
    # T_object_in_camera = pose_to_homogeneous_matrix(pose_object_in_camera)

    # dot_start_time = time.time()
    # T_object_in_world = np.dot(T_vehicle_in_world, T_object_in_camera)
    # dot_end_time = time.time()

    # print("compute time: dot:{}".format(dot_end_time-dot_start_time))

    # pose_object_in_world = homogeneous_matrix_to_pose(T_object_in_world)

    # return pose_object_in_world

# def compute_object_pose_transformed_from_pose(pose_source_frame_in_target_frame, pose_object_in_source_frame):
#     T_source_frame_in_target_frame = pose_to_matrix(pose_source_frame_in_target_frame[0], pose_source_frame_in_target_frame[1])
#     T_object_in_source_frame = pose_to_matrix(pose_object_in_source_frame[0], pose_object_in_source_frame[1])

#     T_object_in_target_frame = np.dot(T_source_frame_in_target_frame, T_object_in_source_frame)
#     pose_object_in_target_frame = matrix_to_pose(T_object_in_target_frame)

#     return pose_object_in_target_frame
    