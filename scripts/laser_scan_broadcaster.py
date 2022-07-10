#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from sensor_msgs.msg import LaserScan, Range
from tf2_geometry_msgs import PointStamped
import tf2_ros
import numpy as np

class distData:
    def __init__(self, num_points):
        rospy.init_node('laser_scan', anonymous=True)
        rate = rospy.Rate(10)
        # self.pubDist = rospy.Publisher("bot%s/scan" % self.device, LaserScan, queue_size=1)
        self.pubDist = rospy.Publisher("scan2", LaserScan, queue_size=1)
        self.seq = 0
        self.num_readings = 360
        self.laser_frequency = 1/6.1
        self.num_points = int(num_points)
        self.stacking_factor = 30
        self.ps = PointStamped()
        self.ps_transformed = PointStamped()
        self.dist_measurement = []
        self.points = []
        self.tfBuffer = tf2_ros.Buffer(rospy.Duration(5))
        tf2_ros.TransformListener(self.tfBuffer)
        for i in range(self.num_points):
            if i < 10:
                key = str(0) + str(i)
            else:
                key = str(i)
            rospy.Subscriber("bot%s/dist/data" % key, Range, self.update_cloud_matrix, i)

        while not rospy.is_shutdown():
            self.sensor_data_update(self.dist_measurement)
            rate.sleep()

    def update_cloud_matrix(self, msg, bot_id):
        if bot_id < 10:
            frame_id = 'bot0' + str(bot_id) + "_analyt"
        else:
            frame_id = 'bot' + str(bot_id) + "_analyt"
        self.ps.header.frame_id = frame_id
        self.ps.header.stamp = rospy.Time(0)
        self.ps.point.x = msg.range
        self.ps.point.y = 0
        self.ps.point.z = 0

        try:
            self.ps_transformed = self.tfBuffer.transform(self.ps, "odom", timeout=rospy.Duration(1))
            pt = [self.ps_transformed.point.x, self.ps_transformed.point.y, self.ps_transformed.point.z]
            if np.size(self.points) == 0:
                self.points = pt
            else:
                self.points = np.vstack((self.points, pt))
            if len(self.points) >= (self.stacking_factor * self.num_points):
                x = self.points[:, 0]
                y = self.points[:, 1]
                th = np.arctan2(y, x)
                zipped_x = zip(th, x)
                zipped_y = zip(th, y)
                sorted_zipped_x = sorted(zipped_x)
                sorted_zipped_y = sorted(zipped_y)
                sorted_x = np.array([element for _, element in sorted_zipped_x])
                sorted_y = np.array([element for _, element in sorted_zipped_y])
                sorted_th = np.sort(th)
                r = sorted_x**2 + sorted_y**2
                th_interp = np.arange(-np.pi, np.pi, np.pi/180)
                r_interp = np.interp(th_interp, sorted_th, r)
                print(sorted_th)
                dist_measurement = np.array(r).astype(float)
                self.dist_measurement = dist_measurement.tolist()
                self.points = []

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo('waiting for transform msgs.')
            pass


    def sensor_data_update(self, dist_measurement):
        """
        Function to transform a received dist measurement into a ROS dist message

        """
        scan = LaserScan()
        h = rospy.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'odom' # @sa: tf_broadcaster.py
        h.seq = self.seq
        self.seq += 1
        scan.header = h
        scan.angle_min = -np.pi
        scan.angle_max = np.pi
        scan.angle_increment = 0.0174533
        scan.time_increment = (1.0 / self.laser_frequency) / self.num_readings
        scan.range_min = 0.0
        scan.range_max = 400.0
        scan.ranges = []
        if np.size(dist_measurement) > 0:
            scan.ranges[:] = [float(x) for x in dist_measurement]
            # h.seq = self.seq
            # # increase sequence
            # self.seq += 1
            # add header to Range message

            self.pubDist.publish(scan)


if __name__ == '__main__':
    try:
        data = distData(sys.argv[1])
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass
