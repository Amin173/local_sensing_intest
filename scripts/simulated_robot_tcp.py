#!/usr/bin/env python

import rospy
import sys
import socket
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import String
import netifaces as ni
from scipy.spatial.transform import Rotation as R
from scipy.linalg import block_diag
from numpy import array, cos, sin, pi, loadtxt
from numpy.random import random
import ast
import numpy as np

# Get initial arguments, this should include csv file location to run from simulated data
# number of tags, number of robots + 1
num_tags = int(sys.argv[1])
num_bots = int(sys.argv[2])
csv_filename = sys.argv[3]

# Load data

csv_data = loadtxt('/opt/ros/overlay_ws/src/local_sensing_intest/simulated_csv/' + csv_filename, delimiter=',')
max_time = csv_data[-1, 0]

def num_2_str(num):
    if num > 9:
        return str(num)
    else:
        return '0' + str(num)

# Define dummy data for case where csv is not used
ang_2_q = lambda alf: R.from_euler('z', -alf, degrees=False).as_quat()
default_dict = lambda n: {"acc": [0., 0., -1.], "rot": ang_2_q((2 * pi * n / (num_tags - 1)) + pi / 2 * (n%2)), "dist": 300 + (random()*100 - 50)}

# Returns the positions, angles and ranges at a given time for all bots
def getTime_data(time, num_bots, data):
    # get data timestep
    file_dt = data[1, 0] - data[0, 0]

    # calculate index for given time and timestep:
    idx = int(np.round(time / file_dt, 0))

    tmp = data[idx, 1:].reshape((num_bots, 7))

    positions = tmp[:, :2]
    velocities = tmp[:, 2:4]
    norms = tmp[:, 4:6]
    angles = np.arctan2(norms[:, 1], norms[:, 0])
    ranges = tmp[:, 6]

    # return needed data only
    return positions, angles, ranges

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
# time to which send csv data
glob_time = -1

def callback(msg):
    global motion_cmd
    data_dict = eval(msg.data)
    #motion_cmd = str(data_dict[dev_id_str][1]) + "_" + str(data_dict[dev_id_str][0])

def callbackTime(msg):
    global glob_time
    tmp = ast.literal_eval(msg.data)
    glob_time = tmp["time"]

rospy.Subscriber("control_cmd", String, callback)

# Callback to send range data
rospy.Subscriber("state", String, callbackTime)

# Define rate at which to run simulation
seq = 0
rate = rospy.Rate(20)
now = -1.
while not rospy.is_shutdown() and now < max_time:
    # increment sequence time
    seq += 1

    if glob_time >= 0.:

        now = rospy.get_rostime()
        now = now.secs + now.nsecs * 1e-9 - glob_time
        
        positions, angles, ranges = getTime_data(now, num_bots, csv_data)
        #ranges = np.flip(ranges)
        

        # for each tag generate data and publish
        for i, (imu_p, range_p, imu_d, range_d) in enumerate(zip(imu_pubs, range_pubs, angles, ranges)):
            
            data_dict = {"acc": [0., 0., -1.], "rot": ang_2_q((imu_d + pi / 2 * (i%2))), "dist": range_d * 1000}
            
            dev_id_str = num_2_str(i)

            # Publish data for tag
            publish_imu(imu_p, dev_id_str, data_dict, seq)
            publish_range(range_p, dev_id_str, data_dict, seq)
    rate.sleep()