#!/usr/bin/env python
import roslib
roslib.load_manifest('stereo_color_tracker')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_track:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=5)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/left_cam/image_raw",Image,self.callback)

    # Trackbar stuff
    self.lower_threshold = np.array([66, 97, 180])
    self.upper_threshold = np.array([96, 222, 255])

    cv2.namedWindow("Control", cv2.CV_WINDOW_AUTOSIZE); # Threshold Controller window
    cv2.namedWindow("Thresholded Image", cv2.CV_WINDOW_AUTOSIZE); # Threshold image window

    cv2.createTrackbar("LowH", "Control", self.lower_threshold[0], 255, self.updateLowH); # Hue (0 - 255)
    cv2.createTrackbar("HighH", "Control", self.upper_threshold[0], 255, self.updateHighH);
    cv2.createTrackbar("LowS", "Control", self.lower_threshold[1], 255, self.updateLowS); # Saturation (0 - 255)
    cv2.createTrackbar("HighS", "Control", self.upper_threshold[1], 255, self.updateHighS);
    cv2.createTrackbar("LowV", "Control", self.lower_threshold[2], 255, self.updateLowV); # Value (0 - 255)
    cv2.createTrackbar("HighV", "Control", self.upper_threshold[2], 255, self.updateHighV);

  def updateLowH(self, value):
    self.lower_threshold[0] = value
  def updateHighH(self, value):
    self.upper_threshold[0] = value
  def updateLowS(self, value):
    self.lower_threshold[1] = value
  def updateHighS(self, value):
    self.upper_threshold[1] = value
  def updateLowV(self, value):
    self.lower_threshold[2] = value
  def updateHighV(self, value):
    self.upper_threshold[2] = value
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    # Get HSV image
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Threshold image to range
    mask = cv2.inRange(hsv, self.lower_threshold, self.upper_threshold)

    # Erode/Dilate mask to remove noise
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ERODE, (3,3) ))
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_DILATE, (3,3) ))

    # Mask image
    cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Thresholded Image", cv_image)
    k = cv2.waitKey(3) & 0xFF
    if k == 113 or k == 27: # Escape key = 27, 'q' = 113
      rospy.signal_shutdown("User Exit")

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
      print e

def main(args):
  ic = image_track()
  rospy.init_node('image_track', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()
  print "Finished."

if __name__ == '__main__':
    main(sys.argv)