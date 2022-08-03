#!/usr/bin/env python

"""
Python script used to use the april tags
tracking system from : https://github.com/duckietown/apriltags3-py
"""

####################################################
import sys
import rospy
from std_msgs.msg import String
from numpy import array, cos, sin, pi

sys.path.append('./JameobaApriltags/')
sys.path.append('./apriltags3py/')
import datetime

####################################################


def main(num_of_tags):
    # Say if you want to lot the data after the test
    plot_data = False
    animate_data = True
    date = datetime.datetime.now()

    # Create data dictionary and fake data for now
    data = {}

    # Generate offset point
    
    p0 = array([300., 300., 0.])
    data[num_of_tags] = tuple(p0)

    # Create all node data
    for i in range(int(num_of_tags)):

        # Generate tag
        idx = str(i)
        if len(idx) == 1:
            idx = '0' + idx
        
        # Generate position
        alf = 2 * pi * i / int(num_of_tags)
        p = 30. * array([cos(alf), sin(alf), 0.]) + p0

        data[idx] = tuple(p)


    # Setup ROS
    pub = rospy.Publisher('state', String, queue_size=10)
    rospy.init_node('AprilTags', anonymous=False)


    rate = rospy.Rate(5) # in Hz

    while not rospy.is_shutdown():
        pub.publish(str(data))
        rate.sleep()


if __name__ == '__main__':
    try:
        main(sys.argv[1])
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass
