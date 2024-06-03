import rospy
from numpy import inf
from geometry_msgs.msg import Pose, PoseStamped, Twist
from vision_detector.msg import ObjectPoseArray

def setAction(value:bool)->bool:

    # Do nothing
    if not value:
        return True
    # Get person pose in the world
    try:
        poses = rospy.wait_for_message('/yolo_detector/object_poses', ObjectPoseArray, timeout=3)
    except Exception as e:
        rospy.logerr(str(e))
        rospy.logerr('Reading object poses failed')
        return False

    person = None
    conf = -inf
    for obj in poses.objects:
        if obj.name == 'person' and obj.confidence >= conf and obj.isValid:
            person = obj
            conf = obj.confidence

    if person is None:
        rospy.logerr("Fail to follow since no person is detected!")
        return False

    # Follow person using navigation module 

    # Stop the vehicle and set new goal
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=20)

    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    cmd_pub.publish(twist)

    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    obj_pose = PoseStamped(poses.header, person.pose)
    goal_pub.publish(obj_pose)
    rospy.loginfo("Object pose as navigation goal set!")

    # If nothing goes wrong, return True
    return True
