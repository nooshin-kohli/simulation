#! /usr/bin/env python3.6
from pickle import NONE
import numpy as np
from numpy.core.fromnumeric import transpose
import rospy
from numpy import ndarray
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64, String, Int32, Int32MultiArray, MultiArrayLayout, MultiArrayDimension
from robot_class import ROBOT
from scipy.interpolate import InterpolatedUnivariateSpline as intp
import math
import time
import matplotlib.pyplot as plt
import sys
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
import apriltag

rospy.init_node("node", anonymous=True)
# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
bridge = CvBridge()

# Define a function to show the image in an OpenCV Window
def show_image(img,imgGray):
    cv2.imshow("Image Window", img)
    gray = cv2.imshow("Image Window_2", imgGray)
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(imgGray)
    print("[INFO] {} total AprilTags detected".format(len(results)))
    # arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    # arucoParams = cv2.aruco.DetectorParameters_create()
    # (corners, ids, rejected) = cv2.aruco.detectMarkers(imgGray, arucoDict,parameters=arucoParams)


    # ret, thresh = cv2.threshold(imgGray, 127, 255, 0)
    # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(img, contours, -1, (0,255,0), 10)
    # arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    # arucoParam = aruco.DetectorParameters_create()
    # bbox, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam, cameraMatrix = matrix_coefficients, distCoeff = distortion_coefficients)
    cv2.waitKey(3)
    # print("id:",ids)

# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        imgGray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # global matrix_coefficients
        # global distortion_coefficients
        # arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        # arucoParam = aruco.DetectorParameters_create()
        # bbox, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam, cameraMatrix = matrix_coefficients,
        #                                                                 distCoeff = distortion_coefficients)
    except CvBridgeErrora and e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Show the converted image
    show_image(cv_image,imgGray)

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

# Initialize an OpenCV Window named "Image Window"
cv2.namedWindow("Image Window", 1)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()
