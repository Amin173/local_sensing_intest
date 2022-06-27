#!/usr/bin/env python
import rospy
import sys
import netifaces as ni
from pyparticleio.ParticleCloud import ParticleCloud

# I can change this code to subscribe to another topic of the particle cloud and receive
# a msg when each device server ip is updated.

ni.ifaddresses('wlp4s0')
ip = ni.ifaddresses('wlp4s0')[ni.AF_INET][0]['addr']

dev_id_str = '11'
dev_id = int(dev_id_str)

rospy.init_node('publish_particle_cloud', anonymous=True)
rate = rospy.Rate(0.5)
timeout = 30
start_time = rospy.Time.now()
while (rospy.Time.now() - start_time) < rospy.Duration.from_sec(timeout):
    particle_cloud = ParticleCloud("8269d62ca4178e500166245f1835a2e96f4a0de4")
    rospy.sleep(1)
    getattr(particle_cloud, str("ID" + dev_id_str + "_LEG")).publish("ip_number", str(ip))
    rate.sleep()



# def callback(event_data):
#     particle_cloud = ParticleCloud("8269d62ca4178e500166245f1835a2e96f4a0de4")
#     particle_cloud.ID06_LEG.publish("ip_number", str(ip))
#     rospy.sleep(1)
#
#
#
# if __name__ == '__main__':
#     rospy.init_node('publish_particle_cloud', anonymous=True)
#     particle_cloud = ParticleCloud("8269d62ca4178e500166245f1835a2e96f4a0de4")
#     particle_cloud.ID06_LEG.subscribe("ip_request", callback)
#
#     rospy.spin()
