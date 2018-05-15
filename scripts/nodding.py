#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def callback(data):
    rospy.loginfo(rospy.get_call_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('js_listener', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
