import rospy
import message_filters
from vision_detector.msg import Detection, DetectionArray


def detect()->list[Detection]:
    objects = []

    try:
        dcts = rospy.wait_for_message('/yolo_detector/detections', DetectionArray, timeout=5)
        objects = set(map(lambda x:x.name, dcts.detections))
    except:
        rospy.logerr("Waiting for Yolo detections timeout!")

    return objects



