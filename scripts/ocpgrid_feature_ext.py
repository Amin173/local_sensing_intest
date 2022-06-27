#!/usr/bin/env python2.7

#Working in progress...(not functional)

import rospy
import tf
from sensor_msgs.msg import PointCloud2, Range, LaserScan, PointField
from geometry_msgs.msg import PointStamped, Point32
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2

class OcpGrid:
    def __init__(self):
        self.ocp_grid = OccupancyGrid()
        self.ocp_grid.header.frame_id = 'map'
        self.cloud_defined = False
        self.points = []

    def callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        # free_thresh = msg.info.free_thresh
        # occupied_thresh = msg.info.occupied_thresh
        data = msg.data
        x = np.linspace(0, 1.95, width)
        y = np.linspace(0, 1.03, height)
        vis = np.array(data, dtype=np.uint8).reshape(height, width)
        img = np.array(vis * 255, dtype=np.uint8)
        thresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 0)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=3)

        minLineLength = 50
        maxLineGap = 10
        lines = cv2.HoughLinesP(close, rho=1, theta=np.pi / 180, threshold=100, minLineLength=minLineLength,
                                maxLineGap=maxLineGap)


        # for line in lines:
        #     c1 = (lines[:,:,0:2]-line[0:2])**2

        if lines is not None:
            for line in lines:
                # line = line * 1.1 * 2.5 / (5 * 96)
                for x1, y1, x2, y2 in line:
                    # print("points:", x1, y1, x2, y2)
                    cv2.line(img, (x1, y1), (x2, y2), (36, 255, 12), 3)
        self.ocp_grid.data = np.asarray(img, dtype=np.int8).reshape(-1).tolist()
        print(np.shape(self.ocp_grid.data))
        self.ocp_grid.info = msg.info
        # cv2.imshow('thresh', thresh)
        # cv2.imshow('close', close)
        #
        # cv2.imshow('image', img)
        #
        # cv2.waitKey()
        # cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('map_hoghman_filter')
    ocp = OcpGrid()
    rospy.Subscriber("/map", OccupancyGrid, ocp.callback)
    ocp_pub = rospy.Publisher('map_hoghman', OccupancyGrid, queue_size=1)
    rate = rospy.Rate(20.0)
    seq = 0
    while not rospy.is_shutdown():
        try:
            ocp.ocp_grid.header.seq = seq
            ocp.ocp_grid.header.stamp = rospy.Time.now()
            ocp_pub.publish(ocp.ocp_grid)
            seq += 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
