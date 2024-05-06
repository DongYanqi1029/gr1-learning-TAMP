#! /usr/bin python

import yaml
import rospy
import sys, getopt
from visualization_msgs.msg import MarkerArray
from rospy_message_converter import message_converter

class SemanticMapSaver():
    def __init__(self):
        rospy.init_node("semantic_map_saver")
        self.semantic_markers = rospy.wait_for_message("/semantic_map", MarkerArray, timeout=5)

    def save_semantic_map(self, filename):
        marker_list = self.semantic_markers.markers

        data = {'markers':[]}
        
        for m in marker_list:
            marker_dict = message_converter.convert_ros_message_to_dictionary(m)
            data['markers'].append(marker_dict)
        
        # save markers in yaml
        with open(filename, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
            f.close()

        rospy.loginfo(filename + " file for semantic map saved!")


if __name__ == "__main__":
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'f:', ['file='])
    except getopt.Getopterror:
        print("Run semantic_map_saver node with the format 'semantic_saver.py -f <file-path-prefix>'")

    filename = ""
    for opt, arg in opts:
        if opt in ('-f', '--file'):
            filename = arg

    saver = SemanticMapSaver()
    saver.save_semantic_map(filename + "_semantic.yaml")