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
from numpy.random import random


# Get initial arguments, this should include csv file location to run from simulated data
# number of tags, number of robots + 1
num_tags = int(sys.argv[1])

def num_2_str(num):
    if num > 9:
        return str(num)
    else:
        return '0' + str(num)

# Define dummy data for case where csv is not used
ang_2_q = lambda alf: R.from_euler('z', alf, degrees=False).as_quat()
default_dict = lambda n: {"acc": [0., 0., -1.], "rot": ang_2_q((2 * pi * n / (num_tags - 1)) + pi / 2 * (1 - (n%2))), "dist": 300 + (random()*100 - 50)}

# Initialize node
rospy.init_node('tcp_connection', anonymous=True)

# Define publishers: num_tags * 3
range_pubs = []
imu_pubs = []
data_pubs = []
for i in range(num_tags):

    range_pub = rospy.Publisher("bot%s/dist/data" % num_2_str(i), Range, queue_size=1)
    imu_pub = rospy.Publisher("bot%s/imu/data" % num_2_str(i), Imu, queue_size=1)
    data_pub = rospy.Publisher("bot%s/sensor/data" % num_2_str(i), String, queue_size=1)

    range_pubs.append(range_pub)
    imu_pubs.append(imu_pub)
    data_pubs.append(data_pub)


# Function to publish imu data
def publish_imu(publisher, dev_id_str, imu_measurement, seq):
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
    publisher.publish(imu_msg)

# Function to publish range data
def publish_range(publisher, dev_id_str, range_measurement, seq):
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
    publisher.publish(dist_msg)



# Code starts here:

def callback(msg):
    global motion_cmd
    data_dict = eval(msg.data)
    #motion_cmd = str(data_dict[dev_id_str][1]) + "_" + str(data_dict[dev_id_str][0])


rospy.Subscriber("control_cmd", String, callback)

# Define rate at which to run simulation
seq = 0
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    # increment sequence time
    seq += 1

    # for each tag generate data and publish
    for i, (imu_p, range_p) in enumerate(zip(imu_pubs, range_pubs)):
        # TODO: read from csv to generate data_dict for each tag
        data_dict = default_dict(i)
        dev_id_str = num_2_str(i)

        # Publish data for tag
        publish_imu(imu_p, dev_id_str, data_dict, seq)
        publish_range(range_p, dev_id_str, data_dict, seq)

    rate.sleep()