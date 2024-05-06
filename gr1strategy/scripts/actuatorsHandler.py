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
        # rospy.loginfo("Virtual Execution of action: {}".format(name))

        # Navigation module for path and motion planning
        if name.startswith('goto_'):
            region_name = name[5:]
            if value:
                # function call for navigation module
                nav = importlib.import_module('.nav', package='gr1strategy.actuators')
                reach = nav.gotoRegion(region_name)

                if not reach:
                    res = False
                    rospy.logerr("Navigation to region {} execution failed!".format(region_name))
                else:
                    rospy.loginfo("Navigation to region {} execution Completed!".format(region_name))
            else:
                pass
        
        # Other actuator module 
        else:
            # Function call for ROS Service request of corresponding module
            # setAction = getattr(actuators, name + ".setAction")
            # feedback = setAction(value)
            # if not feedback:
            #     res = False
            #     rospy.logerr("Actuator {} execution failed!".format(name))
            pass


    resp = SetActuatorsResponse(bool(res))

    return resp

if __name__ == "__main__":
    rospy.init_node("actuators_handler")

    srv = rospy.Service("set_actuators", SetActuators, handle_set_actuators)
    rospy.loginfo("Ready to handle set actuators.")

    rospy.spin()