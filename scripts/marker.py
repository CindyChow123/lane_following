#!/usr/bin/env python

from __future__ import print_function # Python 2/3 compatibility
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np

ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
  }

desired_aruco_dictionary = "DICT_ARUCO_ORIGINAL"
this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
this_aruco_parameters = cv2.aruco.DetectorParameters_create()


def aruco_detect(frame):
  
  # Detect ArUco markers in the video frame
  (corners, ids, rejected) = cv2.aruco.detectMarkers(
    frame, this_aruco_dictionary, parameters=this_aruco_parameters) 
  if ids is not None:
	print(ids)
  cv2.imshow("frame",frame)
  cv2.waitKey(0)


if __name__ == '__main__':
  
  # The different ArUco dictionaries built into the OpenCV library. 

  
  # Check that we have a valid ArUco marker
  if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported".format(
      args["type"]))
    sys.exit(0)

  frame = cv2.imread('701.png')
  aruco_detect(frame)
  
  
     

