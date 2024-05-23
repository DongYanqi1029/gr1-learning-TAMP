#! /usr/bin python

import rospy
from gr1strategy.srv import GetSensors, GetSensorsResponse
import gr1strategy.sensors as sensors
import json
from collections import defaultdict
import importlib

def handle_get_sensors(req):
    props = req.sensors
    sensors_values = defaultdict(bool)

    for name in props:
        rospy.loginfo("Processing sensor proposition: {}...".format(name))

        # Vision detection module for object detection
        if name.startswith('see_'):
            object_name = name[4:]

            # function call for vision detection module
            yolo = importlib.import_module('.yolo', package='gr1strategy.sensors')
            objects = yolo.detect()

            if object_name in objects:
                sensors_values[name] = True
                rospy.loginfo("{} detected!".format(object_name))
            else:
                sensors_values[name] = False
                rospy.loginfo("{} not detected!".format(object_name))
        
        else:
            try:
                sensor = importlib.import_module('.{}'.format(name), package='gr1strategy.sensors')
                value = sensor.getSensor()

                sensors_values[name] = value
            except:
                rospy.logerr("sensor '{}' not implemented!".format(name))
                sensors_values[name] = False 

    resp = GetSensorsResponse(json.dumps(sensors_values))

    return resp

if __name__ == "__main__":
    rospy.init_node("sensors_handler")

    srv = rospy.Service("get_sensors", GetSensors, handle_get_sensors)
    rospy.loginfo("Ready to handle get sensors values.")

    rospy.spin()