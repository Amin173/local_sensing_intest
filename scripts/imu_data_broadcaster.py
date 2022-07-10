#!/usr/bin/env python
# license removed for brevity
# https://www.sparkfun.com/products/14686

import rospy
from pyparticleio.ParticleCloud import ParticleCloud
import sys
from sensor_msgs.msg import Imu
import tf_conversions


class imuData:
    def __init__(self, device):
        self.particle_cloud = ParticleCloud("8269d62ca4178e500166245f1835a2e96f4a0de4")
        self.device = device
        rospy.init_node('get_imu_data', anonymous=True)
        rate = rospy.Rate(20)
        self.pubImu = rospy.Publisher("bot%s/imu/data" % self.device, Imu, queue_size=1)
        self.seq = 0
        time0 = rospy.get_time()
        while not rospy.is_shutdown():
            imu_measurement = self.parse_incoming_data()
            self.sensor_data_updated(imu_measurement)
            rospy.loginfo(1/float(rospy.get_time()-time0))
            time0 = rospy.get_time()
            rate.sleep()

    def parse_incoming_data(self):
        data = self.get_data(self.particle_cloud, self.device)
        imu_measurement = eval(data)
        return imu_measurement

    def sensor_data_updated(self, imu_measurement):
        """
        Function to transform a received imu measurement into a ROS Imu message

        :param imu_measurement: carla imu measurement object
        :type imu_measurement: carla.IMUMeasurement
        """
        imu_msg = Imu()
        h = rospy.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'bot'+self.device  # @sa: tf_broadcaster.py
        h.seq = self.seq
        # increase sequence
        self.seq += 1
        # add header to IMU message
        imu_msg.header = h
        #  (X forward, Y left, Z up).
        imu_msg.angular_velocity.x = -imu_measurement["gyroscope"][0]
        imu_msg.angular_velocity.y = imu_measurement["gyroscope"][1]
        imu_msg.angular_velocity.z = -imu_measurement["gyroscope"][2]

        imu_msg.linear_acceleration.x = imu_measurement["accelerometer"][0]
        imu_msg.linear_acceleration.y = -imu_measurement["accelerometer"][1]
        imu_msg.linear_acceleration.z = imu_measurement["accelerometer"][2]

        q = tf_conversions.transformations.quaternion_from_euler(imu_measurement["rotation"][0],
                                                                 imu_measurement["rotation"][1],
                                                                 imu_measurement["rotation"][2])
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        self.pubImu.publish(imu_msg)

    def get_data(self, particle_cloud, device):
        return getattr(particle_cloud, str("ID" + device + "_LEG")).imu_data


if __name__ == '__main__':
    try:
        data = imuData(sys.argv[1])
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass
