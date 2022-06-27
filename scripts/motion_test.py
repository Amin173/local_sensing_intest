#!/usr/bin/env python
"""
Python script used to control the vibration robot (pink)
"""

####################################################
from local_sensing_intest.Utils import UtilFunctions

import matplotlib.pyplot as plt
import rospy
import sys
from std_msgs.msg import String
from pyparticleio.ParticleCloud import ParticleCloud
import numpy as np


def test(test="vac"):
    """ Method to test different part of the vacuum robot """
    num = 12
    bot_ids = range(num)
    msg = {"00": (0, 0)}

    if test == "v":
        for i in bot_ids:
            msg[key(i)] = (2, 255)

    elif test == "leg+":
        for i in bot_ids:
            msg[key(i)] = (-1, 255)

    elif test == "leg-":
        for i in bot_ids:
            msg[key(i)] = (1, 255)

    elif test == "stop":
        for i in bot_ids:
            msg[key(i)] = (np.random.randint(3, 9), 0)

    elif test == "vac":
        for i in bot_ids:
            msg[key(i)] = (2, 255)

    elif test == "sleep":
        getattr(particle_cloud, str("ID10_LEG")).publish("sleep")

    return str(msg)


def key(i):
    if i < 10:
        key = str(0) + str(i)
    else:
        key = str(i)
    return key


if __name__ == '__main__':
    rospy.init_node('motion_test', anonymous=True)
    cmd_pub = rospy.Publisher("control_cmd", String, queue_size=10)
    particle_cloud = ParticleCloud("8269d62ca4178e500166245f1835a2e96f4a0de4")
    rate = rospy.Rate(1)
    arg = sys.argv[1]
    while not rospy.is_shutdown():
        cmds = test(arg)
        cmd_pub.publish(cmds)
        rate.sleep()
    for i in range(10):
        cmd_pub.publish(test("stop"))
