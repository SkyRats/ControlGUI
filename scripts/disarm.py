import pygame

import rospy
import mavros_msgs
from mavros_msgs import srv
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import String
from mavros_msgs.msg import State
import time

arm = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
for i in range(300):
    arm(False)
