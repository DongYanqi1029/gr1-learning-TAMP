#! /usr/bin python

import rospy
import yaml
import sys, getopt
from rospy_message_converter import message_converter
from visualization_msgs.msg import Marker, MarkerArray

class SemanticMapServer():
    def __init__(self):
        # Initial node and semantic_map data publisher 
        rospy.init_node("semantic_server")
        self.semantic_map_pub = rospy.Publisher("/semantic_map", MarkerArray, queue_size=10)

    def load_semantic_map(self, filename):
        # Load semantic_map yaml 
        with open(filename, 'r') as f:
            data = yaml.safe_load(f)
            f.close()

        # Load markers info 
        self.semantic_markers = MarkerArray()
        for m in data['markers']:
            marker = message_converter.convert_dictionary_to_ros_message('visualization_msgs/Marker', m)
            self.semantic_markers.markers.append(marker)

    def spin(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.semantic_map_pub.publish(self.semantic_markers)
            r.sleep()

        rospy.spin()

        
if __name__ == "__main__":
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'f:', ["file="])
    except getopt.Getopterror:
        rospy.logerr("Run semantic_map_server node with the format 'semantic_map_server.py -f <file-path>'")

    filename = ""
    for opt, arg in opts:
        if opt in ('-f', '--file'):
            filename = arg
    
    server = SemanticMapServer()
    server.load_semantic_map(filename)
    server.spin()
