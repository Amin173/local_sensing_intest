#!/usr/bin/env python

import rospy
import sys
import socket
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import String
import netifaces as ni
from scipy.spatial.transform import Rotation as R
from scipy.linalg import block_diag
from numpy import array, cos, sin, pi


dev_id_str = sys.argv[1]
dev_id = int(dev_id_str)

#ang_2_q = lambda alf: R.from_matrix(block_diag(array([[cos(alf), sin(alf)], [-sin(alf), cos(alf)]]), [[1.]])).as_quat()
ang_2_q = lambda alf: R.from_euler('z', -alf).as_quat()
off = pi / 2
if dev_id%2 == 0:
    off = 0

default_dict = {"acc": [0., 0., -1.], "rot": ang_2_q(2 * pi * dev_id / 12. - off), "dist": 300}

rospy.init_node('tcp_connection', anonymous=True)
range_pub = rospy.Publisher("bot%s/dist/data" % dev_id_str, Range, queue_size=1)
imu_pub = rospy.Publisher("bot%s/imu/data" % dev_id_str, Imu, queue_size=1)
data_pub = rospy.Publisher("bot%s/sensor/data" % dev_id_str, String, queue_size=1)


def publish_imu(imu_measurement, seq):
    imu_msg = Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = 'bot' + dev_id_str
    imu_msg.header.seq = seq
    imu_msg.linear_acceleration.x = imu_measurement["acc"][0]
    imu_msg.linear_acceleration.y = -imu_measurement["acc"][1]
    imu_msg.linear_acceleration.z = imu_measurement["acc"][2]
    imu_msg.orientation.x = imu_measurement["rot"][0]
    imu_msg.orientation.y = imu_measurement["rot"][1]
    imu_msg.orientation.z = imu_measurement["rot"][2]
    imu_msg.orientation.w = imu_measurement["rot"][3]
    imu_pub.publish(imu_msg)


def publish_range(range_measurement, seq):
    dist_msg = Range()
    h = rospy.Header()
    h.stamp = rospy.Time.now()
    h.frame_id = 'bot' + dev_id_str
    h.seq = seq
    dist_msg.header = h
    dist_msg.radiation_type = 1
    dist_msg.field_of_view = 0.2
    dist_msg.min_range = 0
    dist_msg.max_range = 4
    dist_msg.range = float(range_measurement['dist']) / 1000
    range_pub.publish(dist_msg)



# Code starts here:

def callback(msg):
    global motion_cmd
    data_dict = eval(msg.data)
    motion_cmd = str(data_dict[dev_id_str][1]) + "_" + str(data_dict[dev_id_str][0])


rospy.Subscriber("control_cmd", String, callback)
# Receive the data in small chunks and retransmit it
data_buffer = ""
motion_cmd = ""
past_motion_cmd = ""
seq = 0
rate = rospy.Rate(20)
while not rospy.is_shutdown():

    seq += 1

    data_dict = default_dict

    publish_imu(data_dict, seq)
    publish_range(data_dict, seq)

    rate.sleep()