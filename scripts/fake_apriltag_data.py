#!/usr/bin/env python
# license removed for brevity
import rospy
from pyparticleio.ParticleCloud import ParticleCloud
import sys
from std_msgs.msg import String

def update():
    rospy.init_node('fake_april_data', anonymous=True)
    rate = rospy.Rate(50)  # 20hz
    pub = rospy.Publisher('state', String, queue_size=10)
    while not rospy.is_shutdown():
        data = "{\"00\":(0,0,0), \"01\":(0,0,0), \"02\":(0,0,0), \"03\":(0,0,0), \"04\":(0,0,0), \"05\":(0,0,0), \"06\":(0,0,0), \"07\":(0,0,0), \"08\":(0,0,0), \"09\":(0,0,0)}"
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        update()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass
