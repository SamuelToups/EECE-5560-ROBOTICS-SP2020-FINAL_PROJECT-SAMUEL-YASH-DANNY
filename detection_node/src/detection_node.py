#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from duckietown_msgs.msg import BoolStamped, Pose2DStamped
from sensor_msgs.msg import CompressedImage, Image, Joy
from cv_bridge import CvBridge

class DetectionNode:
        def __init__(self):
                self.bridge = CvBridge()
		self.count = 0
		self.firstDetection = None
                rospy.Subscriber("~image", CompressedImage, self.detection, queue_size=1, buff_size=2**24)

		self.detection_image = rospy.Publisher("~detection_image", Image, queue_size=1)
                self.duckie_detected = rospy.Publisher("~duckie_detected", BoolStamped, queue_size=1)
		self.duckie_pose = rospy.Publisher("~pose",Pose2DStamped, queue_size=1) 
		self.mask = rospy.Publisher("~mask", Image, queue_size=1)

		self.low_range = np.array([25,180,180])
                self.high_range = np.array([35,255,255])

        def detection(self, msg):
                cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                cv_crop = cv_img[np.floor(cv_img.shape[0]*.35):cv_img.shape[0],0:cv_img.shape[1]]
                img_hsv = cv2.cvtColor(cv_crop, cv2.COLOR_BGR2HSV)

                mask = cv2.inRange(img_hsv, self.low_range, self.high_range)

                params = cv2.SimpleBlobDetector_Params()
                params.filterByColor = False
                params.filterByArea = True
                params.minArea = 120
                params.filterByInertia = True
		params.minInertiaRatio = 0.7
                params.filterByConvexity = False
                params.filterByCircularity = False
                dectector = cv2.SimpleBlobDetector_create(params)

                keypoints = dectector.detect(mask)

		if len(keypoints) > 0:
			self.count += 1
			if self.firstDetection is not None:
				self.firstDetection = rospy.get_time()
			if self.count >= 3:
				self.count = 0
				self.firstDetection = None
				detect_msg = BoolStamped()
				detect_msg.header.stamp = rospy.Time.now()
				detect_msg.data = True
				pose_msg = Pose2DStamped()
				pose_msg.header.stamp = rospy.Time.now()
				pose_msg.x = keypoints[0].pt[0]
				pose_msg.y = keypoints[0].pt[1]
				pose_msg.theta = 0
				self.duckie_detected.publish(detect_msg)
				self.duckie_pose.publish(pose_msg)

		if self.firstDetection is not None:
			if (rospy.get_time() - self.firstDetection) > 5:
				count = 0

                img_keypoints = cv2.drawKeypoints(cv_crop, keypoints, cv_crop, color=(0,0,255), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		img_keypoints_out = self.bridge.cv2_to_imgmsg(cv_crop, "bgr8")
		img_mask_out = self.bridge.cv2_to_imgmsg(mask, "mono8")
		self.detection_image.publish(img_keypoints_out)
		self.mask.publish(img_mask_out)

if __name__ == '__main__':
	rospy.init_node('detection_node', anonymous=False)
	detection = DetectionNode()
	rospy.spin()

