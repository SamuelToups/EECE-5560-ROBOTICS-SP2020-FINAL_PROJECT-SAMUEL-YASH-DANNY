#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import BoolStamped, Twist2DStamped, Pose2DStamped
import numpy as np

#attempts to steer around objects
#assumes that there is no other traffic and don't need to use caution when leaving lane

#find distance from center to duck, and steer

#steer until duck is no longer detected

class RemoteController:

    def __init__(self):

        #dimensions of image used by detection node
        self.IMAGE_WIDTH = 256.0
        self.IMAGE_HEIGHT = 67.0

        #cmd velocities
        self.omega = 0.8
        self.v = 0.5

        self.steering_enabled = False

        #subscribers
        rospy.Subscriber("remote_steering_start", BoolStamped, self.start_steering_cb)
        rospy.Subscriber("pose", Pose2DStamped, self.avoidance_cb)

        #publishsers
        self.pub_done = rospy.Publisher("remote_steering_complete", BoolStamped, queue_size=1)
        self.pub_car_cmd = rospy.Publisher("lane_controller_node/car_cmd", Twist2DStamped, queue_size=1)

    def start_steering_cb(self, msg):
        self.steering_enabled = msg

    def avoidance_cb(self, msg):

        #return if not active
        if not self.steering_enabled:
            return


        #amount to turn is the distance between the center of the blob and the center of the image

        #avoidance sequence
        #turn away from the object
        #move forward some amount
	#turn back towards original angle
        #turn an additional amount to point back towards original path
        #move forward some amount to return to original line
        #turn back towards original angle

        #amount to turn is pose.x - image_width, lessened by the distance from the bottom of the image
        #deviation_amount is duration of turn
        #omega will be +- some constant

        #while turning the velocity will be 0, while moving the omega will be 0

        #rotate out of lane
        #drive out of lane
        #rotate back towards lane
        #drive back into lane
        #rotate straight into lane
        #stop
        #complete

        deviation_amount = np.floor( msg.x - ( self.IMAGE_WIDTH / 2.0 ) * ( 1.0 - ( msg.y / self.IMAGE_HEIGHT ) ) )

        dt = abs(deviation_amount)

	rotation_direction = np.sign(deviation_amount)

        if rotation_direction == 0:
            rotation_direction = 1

        rotate_out_msg = Twist2DStamped()
        drive_out_msg = Twist2DStamped()
        rotate_back_msg = Twist2DStamped()
        drive_back_msg = Twist2DStamped()
        rotate_straight_msg = Twist2DStamped()
        stop_msg = Twist2DStamped()
        complete_msg = BoolStamped()


        rotate_out_msg.v = 0
        rotate_out_msg.omega = self.omega * rotation_direction

        drive_out_msg.v = self.v
        drive_out_msg.omega = 0

        rotate_back_msg.v = 0
        rotate_back_msg.omega = -1 * self.omega * rotation_direction

        drive_back_msg.v = self.v
        drive_back_msg.omega = 0

        rotate_straight_msg.v = 0
        rotate_straight_msg.omega = self.omega * rotation_direction

        stop_msg.v = 0
        stop_msg.omega = 0

        complete_msg.data = True

        pub_car_cmd.publish(rotate_out_msg)
        rospy.sleep(dt)
        pub_car_cmd.publish(drive_out_msg)
        rospy.sleep(dt)
        pub_car_cmd.publish(rotate_back_msg)
        rospy.sleep(2*dt)
        pub_car_cmd.publish(drive_back_msg)
        rospy.sleep(dt)
        pub_car_cmd.publish(rotate_straight_msg)
        rospy.sleep(dt)
        pub_car_cmd.publish(stop_msg)

        pub_done.publish(complete_msg)

if __name__ == '__main__':
        rospy.init_node('remote_control_node', anonymous=False)
        controller = RemoteController()
        rospy.spin()
