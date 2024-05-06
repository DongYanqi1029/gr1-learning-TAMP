#! /usr/bin python

import rospy
import cmath
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose, PoseStamped
from std_msgs.msg import Bool

import drl_nav.utils as utils


class drl_planner():
    def __init__(self, model_name, model_path):
        # init drl model
        self.model_path = model_path
        self.model_name = model_name

        self.policy = utils.Policy(self.model_name, self.model_path)
        self.obs = utils.Observation()

        self.goal = None # geometry_msgs/Pose

    def goal_cb(self, data):
        self.goal = data.pose
        
        reach = Bool()
        reach.data = False
        self.reach_pub.publish(reach)
        rospy.loginfo("DRL Planner Goal Set!")

    # Check reach target position
    def reach_traget(self, pose):
        target = list([self.goal.position.x, self.goal.position.y])

        relative_x = target[0] - pose[0]
        relative_y = target[1] - pose[1]
        
        distance = abs(complex(relative_x, relative_y))

        if distance < utils.THRESHOLD_GOAL:
            return True
        else:
            return False


    def _motion_step(self):
        # get vehicle pose from amcl
        pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped).pose.pose
        pose = list([pose.position.x, pose.position.y, pose.orientation])

        # Check for goal
        if self.goal is None:
            reach = Bool()
            reach.data = False
            self.reach_pub.publish(reach)

            rospy.logwarn("Please set goal!")
            return None

        # Check reach target 
        if self.reach_traget(pose):
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self.cmd_pub.publish(twist)

            reach = Bool()
            reach.data = True
            self.reach_pub.publish(reach)
            
            rospy.logwarn("Current global plan completed, please set new goal!")
        else:
            # get current traget point
            target = self.goal

            # motion plannig using drl
            self.obs.pose = pose
            self.obs.target = list([target.position.x, target.position.y])
            self.obs.last_action = list(self.policy.last_action)

            state = self.obs.get_state()
            action = self.policy.get_action(state) # linear_vel, angular_vel

            rospy.logwarn("Current pose: " + str(self.obs.pose) + " Current traget: " + str(self.obs.target))

            # send control command to /cmd_vel
            twist = Twist()
            twist.linear.x = action[0]
            twist.angular.z = action[1]
            try:
                # Publish the command to the robot
                self.cmd_pub.publish(twist)

                # Publish reach status
                reach = Bool()
                reach.data = False
                self.reach_pub.publish(reach)
            except:
                print('Error publishing Twist Command')

    def run(self):
        rospy.init_node("drl_planner")
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=20)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        self.reach_pub = rospy.Publisher("/nav/reach", Bool, queue_size=10)

        r = rospy.Rate(30) # 5Hz
        while not rospy.is_shutdown():
            # drl control
            self._motion_step()
            r.sleep()

        rospy.spin()


if __name__ == "__main__":
    model_path = "/home/dongyanqi/catkin_ws/src/myvehicle_navigation/models"
    model_name = "lidar_td3_nav.pt"

    planner = drl_planner(model_name, model_path)
    planner.run()