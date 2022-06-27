#!/usr/bin/env python
"""
Python script used to control the vibration robot (pink)
"""

####################################################
from Utils import UtilFunctions
import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Range, Imu

####################################################
num_of_bots = int(sys.argv[1])
rob = UtilFunctions(vacuum=False, stop_dist=10., threshold_angle=65, num=num_of_bots, target_id='12')


def key(i):
    if i < 10:
        key = str(0) + str(i)
    else:
        key = str(i)
    return key


def listner():
    rospy.Subscriber("state", String, rob.state_callback)
    for i in range(num_of_bots):
        rospy.Subscriber("bot%s/dist/data" % key(i), Range, rob.update_range_sensors)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('motion_planner', anonymous=True)
    listner()
    rob.test("stop")
    rob.get_data()
    # Run some tests
    # rob.start_vacuum()
