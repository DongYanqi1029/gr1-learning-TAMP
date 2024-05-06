#! /usr/bin python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool, Header

def getRegionPose(name:str):
    # Acquire semantic markers
    markers = None
    while markers is None:
        markers = rospy.wait_for_message("/semantic_map", MarkerArray, timeout=5)

    # Search for target region info 
    for marker in markers.markers:
        if marker.ns == name:
            return marker

    # Fail to find region with name 'name'
    return None

def navToGoal(goal_marker:Marker) -> bool:
    # Using ROS Navigation Stack for navigation and motion planning
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    msg = PoseStamped(goal_marker.header, goal_marker.pose)

    goal_pub.publish(msg)
    rospy.loginfo("Successful goal pub")
    

    # Check for reach status
    reach = False
    while not reach:
        msg = rospy.wait_for_message('/nav/reach', Bool, timeout=3)
        reach = msg.data

    return reach

def gotoRegion(goal_name:str) -> bool: 

    pose = getRegionPose(goal_name)

    if pose is None:
        rospy.logerr("Fail to find region with name {}".format(goal_name))
    else:
        rospy.loginfo("Find position for target region: {}".format(goal_name))
        return navToGoal(pose)
        # return True



