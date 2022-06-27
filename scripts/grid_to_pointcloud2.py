#!/usr/bin/env python2.7

import rospy
import tf
from sensor_msgs.msg import PointCloud2, Range, LaserScan, PointField
from geometry_msgs.msg import PointStamped, Point32
from nav_msgs.msg import OccupancyGrid
import numpy as np

class Cloud:
    def __init__(self):

        self.cloud = PointCloud2()
        self.cloud.header.frame_id = 'map'
        self.cloud_defined = False
        self.points = []

    def callback(self, msg):
        if not self.cloud_defined:
            width = msg.info.width
            height = msg.info.height
            # free_thresh = msg.info.free_thresh
            # occupied_thresh = msg.info.occupied_thresh
            data = msg.data
            x = np.linspace(0, 1.95, width)
            y = np.linspace(0, 1.03, height)
            X = np.tile(x, (height, 1))
            Y = np.tile(y.reshape(height,1), (1, width))
            xp = X[np.array(data).reshape(height, width) >= 0.005]
            yp = Y[np.array(data).reshape(height, width) >= 0.005]
            xp = xp[0:-1:25]
            yp = yp[0:-1:25]
            for i, x in enumerate(xp):
                self.points.append([xp[i], yp[i], 0.0, xp[i]*255, yp[i]*255, 0, 255])
            self.points = self.voxelize(self.points)
            self.cloud = self.point_cloud(np.array(self.points), "map")
            self.cloud_defined = False
        else:
            pass

    def voxelize(self, points):
        voxel_size = 0.05
        points = np.array(points)
        sampled_points = []
        x_max, y_max, _, _, _, _, _ = np.max(points, axis=0)
        x_min, y_min, _, _, _, _, _ = np.min(points, axis=0)
        n_x = int((x_max-x_min)/voxel_size)
        n_y = int((y_max-y_min)/voxel_size)
        dx = (x_max - x_min) / n_x
        dy = (y_max - y_min) / n_y
        xv, yv = np.meshgrid(np.linspace(x_min, x_max, n_x), np.linspace(y_min, y_max, n_y))
        xv = xv.reshape(-1, 1)
        yv = yv.reshape(-1, 1)
        X = xv - points[:, 0].reshape(1, -1)
        Y = yv - points[:, 1].reshape(1, -1)
        d = (abs(X) < dx / 2) * (abs(Y) < dy / 2)
        a = np.nonzero(d)
        ocp_grids = np.unique(a[0])
        for grid in ocp_grids:
            if np.size(sampled_points) == 0:
                sampled_points = np.mean(points[a[1][a[0] == grid]], axis=0)
            else:
                sampled_points = np.vstack((sampled_points, np.mean(points[a[1][a[0] == grid]], axis=0)))
        return sampled_points.tolist()

    def point_cloud(self, points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx7 array of xyz positions (m) and rgba colors (0..1)
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        """
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        data = points.astype(dtype).tobytes()

        fields = [PointField(
            name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        header = PointCloud2().header
        header.frame_id = parent_frame
        header.stamp = rospy.Time.now()
        return PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 7),
            row_step=(itemsize * 7 * points.shape[0]),
            data=data
        )


if __name__ == '__main__':
    rospy.init_node('map_to_point_cloud')
    cloud = Cloud()
    rospy.Subscriber("/map", OccupancyGrid, cloud.callback)
    cloud_pub = rospy.Publisher('cloud2_map', PointCloud2, queue_size=1)
    rate = rospy.Rate(20.0)
    seq = 0
    while not rospy.is_shutdown():
        try:
            cloud.cloud.header.seq = seq
            cloud.cloud.header.stamp = rospy.Time.now()
            cloud_pub.publish(cloud.cloud)
            seq += 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
