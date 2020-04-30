#!/usr/bin/env python
# adapted from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

import rospy
from duckietown_msgs.msg import BoolStamped, Twist2DStamped

class DuckieController:

    def __init__(self):
        # block lane follow override message
        self.blockLfOverrideMsg = False

        # stop lane follow (from: object detect node)
        rospy.Subscriber("lane_follow_override", BoolStamped, self.lane_follow_override_cb)

        # object steering completed(from: remote control node)
        rospy.Subscriber("remote_steering_complete", BoolStamped, self.remote_steering_complete_cb)

        # start object steering routine (to: remote control node)
        self.pub_steering = rospy.Publisher("remote_steering_start", BoolStamped, queue_size=1)

        # publish to start/stop lane follow node
        self.pub_joy_override = rospy.Publisher("joy_mapper_node/joystick_override", BoolStamped, queue_size=1)

        # car switch (stop the bot)
        self.pub_car_cmd = rospy.Publisher("lane_controller_node/car_cmd", Twist2DStamped, queue_size=1)

    def lane_follow_override_cb(self, data):

        # this should check and also set the block incomming flag
        if data.data == True and self.blockLfOverrideMsg == False:
            # prevent other msgs from invoking remote control node
            self.blockLfOverrideMsg = True

            # publish message to stop lane follow
            override_msg = BoolStamped()
            override_msg.header.stamp = data.header.stamp
            override_msg.data = False
            self.pub_joy_override.publish(override_msg)

            # publish message to stop the bot
            car_msg = Twist2DStamped()
            car_msg.v = 0.0
            car_msg.omega = 0.0
            self.pub_car_cmd.publish(car_msg)

            # Consider creating a new message to notify romote
            # control to start and send send it the Pose2D from
            # the object detector, instead of object detection
            # sending it to two different nodes. This would
            # eliminate possible race condition..
            # msg: BoolStamped
            #      Posed2D
            #
            # Note this if implemented, this would replace the
            # message below..

            # publish message to start remote control mode.
            remote_ctrl_msg = BoolStamped()
            remote_ctrl_msg.header.stamp = data.header.stamp
            remote_ctrl_msg.data = data.data
            self.pub_steering.publish(remote_ctrl_msg)

    def remote_steering_complete_cb(self, data):

        # this should clear the block incomming flag

        if data.data == True:
            # allow lf override msgs
            self.blockLfOverrideMsg = False

            # publish message to start lane follow
            override_msg = BoolStamped()
            override_msg.header.stamp = data.header.stamp
            override_msg.data = data.data
            self.pub_joy_override.publish(override_msg)

            # Something to consider:
            # publish message to notify object detector node
            # to scan lane so that the object detector node
            # knows when to send and not send messages to us.
            # Other alternative is to keep track of what states
            # we're end to avoid processing obj det msg when in
            # remote mode

if __name__ == '__main__':
    rospy.init_node('duckie_ctrl_node', anonymous=True)
    DuckieController()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
