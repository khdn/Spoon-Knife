#!/usr/bin/env python
import roslib
import rospy
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher(rospy.get_param("~resultImage"),Image, queue_size = 10)
    self.rois_pub = rospy.Publisher('rois', String, queue_size=10)
    rospy.logdebug(rospy.get_param("~sourceImage"))
    
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(rospy.get_param("~sourceImage"),Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
    except CvBridgeError, e:
      print e

#start there

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    #ret, cv_image = cv2.threshold(cv_image, 127, 255, cv2.THRESH_BINARY)


    face_cascade = cv2.CascadeClassifier('/home/z/Downloads/OpenCV/haarcascades/haarcascade_frontalface_default.xml')

    #cap = cv2.VideoCapture(0)

    #if not cap.isOpened():
    #print "lalka"
    #if not face_cascade.empty() :
    #print "lolka"

#img = cv2.imread('/home/z/Downloads/1111.jpg')

#while(True):
    # Capture frame-by-frame
    #ret, img = cap.read()

    # Our operations on the frame come here
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 10)
    for (x,y,w,h) in faces:
        cv2.rectangle(gray,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = gray[y:y+h, x:x+w]
        

    # Display the resulting frame
    #cv2.imshow('img',cv_image)
    cv2.imshow('frame',gray)
    cv2.waitKey(1)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
        #break

    hello_str = "hella world %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    self.rois_pub.publish(hello_str)


#stop there
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(gray, 'mono8'))
    except CvBridgeError, e:
      print e

def main(args):
  rospy.init_node('image_converter', anonymous=True) 
  ic = image_converter()  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
