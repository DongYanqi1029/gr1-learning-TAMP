#! /usr/bin python

import rospy, rospkg
import cmath
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Pose, PoseStamped
from std_msgs.msg import Bool

import drl_nav.utils as utils

class drl_local_planner():
    def __init__(self, model_name, model_path):
        # init drl model
        self.model_path = model_path
        self.model_name = model_name

        self.policy = utils.Policy(self.model_name, self.model_path)
        self.obs = utils.Observation()

        self.global_plan = [] # [geometry_msgs/Pose]
        self.current_goal = -1

    # update gobal plan
    def path_cb(self, data):
        waypoints = [p.pose for p in data.poses] 
        
        if waypoints != self.global_plan:
            self.global_plan = waypoints
            self.current_goal = 0
            rospy.loginfo("Global Plan updated!")
            rospy.loginfo("Executing global path plan...")

            reach = Bool()
            reach.data = False
            self.reach_pub.publish(reach)

    # Check reach target position
    def reach_current_traget(self, pose):
        if self.current_goal < 0 or self.current_goal >= len(self.global_plan):
            return False

        target = list([self.global_plan[self.current_goal].position.x, self.global_plan[self.current_goal].position.y])

        relative_x = target[0] - pose[0]
        relative_y = target[1] - pose[1]
        
        distance = abs(complex(relative_x, relative_y))

        if distance < utils.THRESHOLD_GOAL:
            return True
        else:
            return False

    def local_plan(self)->bool:
        # get vehicle pose from amcl
        pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped).pose.pose
        pose = list([pose.position.x, pose.position.y, pose.orientation])

        # check if global plan exists
        if self.current_goal < 0:
            # rospy.logwarn("Please set goal!")

            reach = Bool()
            reach.data = True
            self.reach_pub.publish(reach)
            
            return True

        # Check reach target and get current target point
        while self.reach_current_traget(pose):
            self.current_goal += 1

        # Check if path plan is completed
        if self.current_goal >= len(self.global_plan):
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self.cmd_pub.publish(twist)

            reach = Bool()
            reach.data = True
            self.reach_pub.publish(reach)
            return True
        else:
            # get current traget point
            target = self.global_plan[self.current_goal]

            # local plannig using drl
            self.obs.pose = pose
            self.obs.target = list([target.position.x, target.position.y])
            self.obs.last_action = list(self.policy.last_action)

            state = self.obs.get_state()
            action = self.policy.get_action(state) # linear_vel, angular_vel

            # rospy.loginfo("Current pose: " + str(self.obs.pose) + " Current traget: " + str(self.obs.target))

            # send control command to /cmd_vel
            twist = Twist()
            twist.linear.x = action[0]
            twist.angular.z = action[1]
            try:
                # Publish the command to the robot
                self.cmd_pub.publish(twist)

                reach = Bool()
                reach.data = False
                self.reach_pub.publish(reach)
                return False
            except:
                rospy.logerr('Error publishing Twist Command')

    def run(self):
        rospy.init_node("drl_local_planner")
        self.path_sub = rospy.Subscriber("/planner/planner/plan", Path, self.path_cb)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=20)
        self.reach_pub = rospy.Publisher("/nav/reach", Bool, queue_size=10)

        plan_completed = False

        r = rospy.Rate(10) # 5Hz
        while not rospy.is_shutdown():
            # drl control
            res = self.local_plan()
            if res and plan_completed == False:
                rospy.logwarn("Current global plan completed, please set new goal!")
            plan_completed = res
            r.sleep()

        rospy.spin()


if __name__ == "__main__":
    model_path = rospkg.RosPack().get_path('myvehicle_navigation') + "/models"
    model_name = "lidar_td3_nav.pt"

    planner = drl_local_planner(model_name, model_path)
    planner.run()