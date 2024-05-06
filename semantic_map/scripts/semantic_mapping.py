#! /usr/bin python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

class SemanticMapping():
    def __init__(self):
        self.semantic_id = 1
        self.semantic_markers = MarkerArray()

        rospy.init_node("semantic_mapping")
        rospy.Subscriber("/clicked_point", PointStamped, self._click_cb)
        self.markers_pub = rospy.Publisher("/semantic_map", MarkerArray, queue_size=10)

    # Create a semantic marker for clicked point in rviz
    def _click_cb(self, data):
        rospy.loginfo("A semantic location is tagged in the Grid Map!")

        # Create TEXT marker
        marker = Marker()
        marker.type = Marker.TEXT_VIEW_FACING

        marker.header = data.header
        marker.ns = "Region" + str(self.semantic_id)
        marker.text = "Region" + str(self.semantic_id)
        marker.id = self.semantic_id
        self.semantic_id += 1

        marker.pose.position.x = data.point.x
        marker.pose.position.y = data.point.y
        marker.pose.position.z = data.point.z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        # Set the scale of the marker (only z is use in TEXT marker)
        marker.scale.x = 0.0
        marker.scale.y = 0.0
        marker.scale.z = 0.3

        # Set the color -- set alpha to something non-zero!
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        # Set transparency -- Marker is fully transparent if color.a is 0.0.
        marker.color.a = 0.5
        # 不加参数就是0，标识生命期为永久
        marker.lifetime = rospy.Duration()

        # Add marker to MarkerArray
        self.semantic_markers.markers.append(marker)

    def spin(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.markers_pub.publish(self.semantic_markers)
            r.sleep()

        rospy.spin()

    
if __name__ == "__main__":
    mapping = SemanticMapping()
    mapping.spin()

