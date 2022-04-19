#!/usr/bin/env python

from __future__ import print_function # Python 2/3 compatibility
import rospy, cv2, cv_bridge, numpy, time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np

desired_aruco_dictionary = "DICT_6X6_250"
cv_file = cv2.FileStorage("camera.yaml", cv2.FILE_STORAGE_READ)
camera_matrix = cv_file.getNode("camera_matrix").mat()
dist_matrix = cv_file.getNode("distortion_coefficients").mat()
cv_file.release()

class Follower:

  def __init__(self):

    self.this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[desired_aruco_dictionary])
    self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()
    self.cam_m = camera_matrix
    self.dist_m = dist_matrix
    self.stopping = False
    self.stopped = False
    self.isnext = False
    self.start_stop = 0
    self.end_stop = 0

    self.bridge = cv_bridge.CvBridge()

    self.image_sub = rospy.Subscriber('camera/image',
            Image, self.image_callback)

    self.cmd_vel_pub = rospy.Publisher('cmd_vel',
            Twist, queue_size=10)

    self.twist = Twist()

  def image_callback(self, msg):

    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    detect = self.aruco_detect(image)

    lower_yellow = numpy.array([ 26, 43, 46])
    upper_yellow = numpy.array([34, 255, 255])

    lower_white = numpy.array([0, 0, 221])
    upper_white = numpy.array([0, 0, 255])
    
    mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask2 = cv2.inRange(hsv, lower_white, upper_white)

    h, w, d = image.shape
    search_top = 2*h/3
    mask1[0:search_top, 0:w] = 0
    mask2[0:search_top, 0:w] = 0

    M1 = cv2.moments(mask1)
    M2 = cv2.moments(mask2)

    if M1['m00'] > 0:
      cx1 = int(M1['m10']/M1['m00'])
      cy1 = int(M1['m01']/M1['m00'])

      cx2 = int(M2['m10']/M2['m00'])
      cy2 = int(M2['m01']/M2['m00'])

      fpt_x = (cx1 + cx2)/2
      fpt_y = (cy1 + cy2)/2 + 2*h/3

      cv2.circle(image, (cx1, cy1), 10, (0,255,255), -1)
      cv2.circle(image, (cx2, cy2), 10, (255,255,255), -1)
      cv2.circle(image, (fpt_x, cy1), 10, (34,144,255), -1)

      err = w/2 - fpt_x
     # print("detect:",detect," stopping:",self.stopping," stopped:",self.stopped)
      # not detect and not stopped / detect and stopped: continue moving

      if self.stopping:
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        if self.isnext:
          self.end_stop = time.time()
         # print(self.end_stop-self.start_stop)
          if ((self.end_stop-self.start_stop)%60>=10):
            self.isnext = False
            self.stopping = False
            self.stopped = True
      else:
        self.twist.linear.x = 0.3
        # if not detect:
        #   self.stopped = False
        self.twist.angular.z = (err*90.0/160)/15

      if (detect and not self.stopped and not self.stopping):
        self.isnext = True
        self.start_stop = time.time()
        self.stopping = True
      elif(not detect and self.stopped):
        self.stopped = False
      
      self.cmd_vel_pub.publish(self.twist)
    cv2.imshow("window", image)
    # cv2.imshow("mask1", mask1)
    # cv2.imshow("mask2", mask2)
    cv2.waitKey(1)

  def aruco_detect(self,frame):
    detect = False
    # cv2.imshow("frame",frame)
    # Detect ArUco markers in the video frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(
      frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters) 
    if ids is not None:
        print(ids)
        revc,tvec= cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.cam_m,self.dist_m)
        if (tvec[0][0][2]<1.5):
          detect = True
         # print(tvec[0][0][2],'cm')
        
    # Check that at least one ArUco marker was detected
    if len(corners) > 0:
      # Flatten the ArUco IDs list
      ids = ids.flatten()
        
      # Loop over the detected ArUco corners
      for (marker_corner, marker_id) in zip(corners, ids):
        
        # Extract the marker corners
        corners = marker_corner.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners
          
        # Convert the (x,y) coordinate pairs to integers
        top_right = (int(top_right[0]), int(top_right[1]))
        bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
        bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
        top_left = (int(top_left[0]), int(top_left[1]))
          
        # Draw the bounding box of the ArUco detection
        cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
        cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
        cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
        cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
          
    #     # Calculate and draw the center of the ArUco marker
    #     center_x = int((top_left[0] + bottom_right[0]) / 2.0)
    #     center_y = int((top_left[1] + bottom_right[1]) / 2.0)
    #     cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
          
    #     # Draw the ArUco marker ID on the video frame
    #     # The ID is always located at the top_left of the ArUco marker
        cv2.putText(frame, str(marker_id), 
          (top_left[0], top_left[1] - 15),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5, (0, 255, 0), 2)

    #   # Display the resulting frame
    #   cv2.imshow('frame',frame)
    return detect

if __name__ == '__main__':
  rospy.init_node('lane_follower')
  
  # The different ArUco dictionaries built into the OpenCV library. 
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
  
  # Check that we have a valid ArUco marker
  if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported".format(
      args["type"]))
    sys.exit(0)
     
  # Load the ArUco dictionary
  # print("[INFO] detecting '{}' markers...".format(
  #   desired_aruco_dictionary))

  
  follower = Follower()
  rospy.spin()
