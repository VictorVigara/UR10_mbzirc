
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

import pyrealsense2 as rs
import cv2
import imutils
import numpy as np

from geometry_msgs.msg import Pose

import torch

print("Before load find_target")
model = torch.hub.load('src/Perception/perception/src/target_node/yolov5-master', 'custom', path='src/Perception/perception/src/target_node/best.pt', force_reload=True, source='local')
print("After load find_target")

from detection import get_target_pos

class MyNode(object):

    def __init__(self):
        rospy.init_node('my_node')
        self.first = True
        self.subscriber_ = rospy.Subscriber('mission_status', Bool, self.callback, queue_size=10)
        self.publisher_ = rospy.Publisher('position', Pose, queue_size=10)

    def callback(self, msg):
        result_msg = Vector3()
        if msg.data and self.first:
            x, y, z = get_target_pos()
            pose_msg = Pose()
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z
            pose_msg.orientation.x = 0.0
            pose_msg.orientation.y = 0.0
            pose_msg.orientation.z = 0.0
            pose_msg.orientation.w = 1.0
            result_msg.x = x
            result_msg.y = y
            result_msg.z = z
            self.publisher_.publish(pose_msg)

            self.first = False

node = MyNode()
rospy.spin()
