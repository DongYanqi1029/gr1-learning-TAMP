#! /usr/bin python

import rospy
from gr1strategy.srv import GetSensors, GetSensorsResponse
import gr1strategy.sensors as sensors
import json

def handle_get_sensors(req):
    props = req.sensors
    sensors_values = {}

    for s in props:
        # getValue = getattr(sensors, s + ".getValue")
        # value = getValue()
        # sensors_values[s] = value
        pass

    resp = GetSensorsResponse(json.dumps(sensors_values))

    return resp

if __name__ == "__main__":
    rospy.init_node("sensors_handler")

    srv = rospy.Service("get_sensors", GetSensors, handle_get_sensors)
    rospy.loginfo("Ready to handle get sensors values.")

    rospy.spin()