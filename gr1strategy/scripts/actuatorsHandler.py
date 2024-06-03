#! /usr/bin python

import rospy
from gr1strategy.srv import SetActuators, SetActuatorsResponse
import importlib
import json
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool

def handle_set_actuators(req):
    props_values = json.loads(req.actuators_values)
    res = True

    for name, value in props_values.items():
        rospy.loginfo("Executing action proposition: {}...".format(name))

        # Navigation module for path and motion planning
        if name.startswith('goto_'):
            region_name = name[5:]
            if value:
                # function call for navigation module
                nav = importlib.import_module('.nav', package='gr1strategy.actuators')
                reach = nav.gotoRegion(region_name)

                if not reach:
                    res = False
                    rospy.loginfo("Navigating to region {} ...".format(region_name))
                else:
                    rospy.loginfo("Navigation to region {} execution Completed!".format(region_name))
            else:
                pass
        
        # Other actuator module 
        else:
            if value:
                try:
                    actuator = importlib.import_module('.{}'.format(name), package='gr1strategy.actuators')
                    feedback = actuator.setAction(value)

                    if not feedback:
                        res = False
                        rospy.loginfo("Executing action {} ...".format(name))
                    else:
                        rospy.loginfo("Execution of action {} completed!".format(name))
                except Exception as e:
                    # do nothing
                    rospy.logerr(str(e))
                    rospy.logwarn("Actuator '{}' not implemented!".format(name))
            else:
                pass

    resp = SetActuatorsResponse(bool(res))

    return resp

if __name__ == "__main__":
    rospy.init_node("actuators_handler", log_level=rospy.INFO)

    srv = rospy.Service("set_actuators", SetActuators, handle_set_actuators)
    rospy.loginfo("Ready to handle set actuators.")

    rospy.spin()