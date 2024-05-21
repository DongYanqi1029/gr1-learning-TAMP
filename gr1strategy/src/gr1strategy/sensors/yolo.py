import rospy
import message_filters
from vision_detector.msg import Detection, DetectionArray


def detect()->list[Detection]:
    dcts = rospy.wait_for_message('/yolo_detector/detections', DetectionArray, timeout=1)

    objects = set(map(lambda x:x.name, dcts.detections))

    return objects



