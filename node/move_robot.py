#! /usr/bin/env python3

import roslib
import rospy
import sys
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
  def __init__(self):
    rospy.init_node('line_follower')

    self.vel_pub = rospy.Publisher('/cmd_vel',Twist)
    self.bridge = CvBridge()
    rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback)
    
    self.lin_speed = 0.4
    self.rot_speed = 3.0

    #pid
    self.kp = 10
    self.kd = 2
    self.last_err = 0


    
  def callback(self,data):

    cv_image = self.bridge.imgmsg_to_cv2(data)
    error = self.move(cv_image)

    p = self.kp * error
    d = self.kd * (error - self.last_err)

    self.drive(p+d)

    self.last_err = error


  def move(self, img):

    w = int(img.shape[1])
    h = int(img.shape[0])
    midpoint = int(w/2)
    buff = int(4*h/5)
    res = (w, h)

    blur = cv2.GaussianBlur(img, (5, 5), 0) #blur for background smoothing
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY) #gray for binary mask easier
    ret, mask = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV) #white track, black background
    crop = mask[buff:h, 0:w] #only search below buff

    #create blob detection by colour
    params = cv2.SimpleBlobDetector_Params()

    params.filterByColor = True
    params.blobColor = 255
    params.filterByArea = False
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False

    #search with detector and report center of track
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(crop)
    if keypoints:
      x = int(keypoints[0].pt[0])
      y = int(keypoints[0].pt[1]) + buff

    errX = midpoint - x
    return errX/w

  def drive(self, error):
    move = Twist()

    move.linear.x = self.lin_speed
    move.angular.z = self.rot_speed * error

    self.vel_pub.publish(move)

  def start(self):
    rospy.spin()

if __name__ == '__main__':
  try:
    line_follower = image_converter()
    line_follower.start()
  except rospy.ROSInterruptException:
    pass


