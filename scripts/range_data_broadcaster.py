#!/usr/bin/env python2.7


import rospy
from pyparticleio.ParticleCloud import ParticleCloud
import sys
from sensor_msgs.msg import Range

class distData:
    def __init__(self, device):
        self.particle_cloud = ParticleCloud("8269d62ca4178e500166245f1835a2e96f4a0de4")
        self.device = device
        rospy.init_node('get_dist_data', anonymous=True)
        rate = rospy.Rate(10)
        self.pubDist = rospy.Publisher("bot%s/dist/data" % self.device, Range, queue_size=10)
        self.seq = 0
        time0 = rospy.get_time()
        while not rospy.is_shutdown():
            dist_measurement = self.parse_incoming_data()
            self.sensor_data_updated(dist_measurement)
            # rospy.loginfo(1 / float(rospy.get_time() - time0))
            time0 = rospy.get_time()
            rate.sleep()

    def parse_incoming_data(self):
        dist_measurement = float(self.get_data(self.particle_cloud, self.device))/1000
        return dist_measurement

    def sensor_data_updated(self, dist_measurement):
        """
        Function to transform a received dist measurement into a ROS dist message

        """
        dist_msg = Range()
        h = rospy.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'bot' + self.device  # @sa: tf_broadcaster.py
        h.seq = self.seq
        # increase sequence
        self.seq += 1
        # add header to IMU message
        dist_msg.header = h
        dist_msg.radiation_type = 1
        dist_msg.field_of_view = 0.2
        dist_msg.min_range = 0
        dist_msg.max_range = 4
        dist_msg.range = dist_measurement
        # h.seq = self.seq
        # # increase sequence
        # self.seq += 1
        # add header to Range message

        self.pubDist.publish(dist_msg)

    def get_data(self, particle_cloud, device):
        return getattr(particle_cloud, str("ID" + device + "_LEG")).dist_data


if __name__ == '__main__':
    try:
        data = distData(sys.argv[1])
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass
