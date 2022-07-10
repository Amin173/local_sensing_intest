#!/usr/bin/env python

import rospy
import sys
import socket
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import String
import netifaces as ni

dev_id_str = sys.argv[1]
dev_id = int(dev_id_str)

rospy.init_node('tcp_connection', anonymous=True)
range_pub = rospy.Publisher("bot%s/dist/data" % dev_id_str, Range, queue_size=1)
imu_pub = rospy.Publisher("bot%s/imu/data" % dev_id_str, Imu, queue_size=1)
data_pub = rospy.Publisher("bot%s/sensor/data" % dev_id_str, String, queue_size=1)

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to the port
ni.ifaddresses('wlp4s0')
ip = ni.ifaddresses('wlp4s0')[ni.AF_INET][0]['addr']
port = 10000 + dev_id
server_address = (ip, port)
print('starting up on ip %s - port %s' % server_address)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(server_address)
# sock.settimeout(2)
# Listen for incoming connections
sock.listen(1)


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


# Wait for a connection
print('waiting for a connection')
connection, client_address = sock.accept()
connection.settimeout(5)
try:
    print('connection from', client_address)


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
        try:
            data_buffer = data_buffer + str(connection.recv(120))
        except:
            break

        if '/' in data_buffer:
            pkg_data = data_buffer.split('/')[0]
            data_buffer = data_buffer[len(pkg_data) + 1:]
            if "_" in  pkg_data:
                pkg_data = pkg_data.split('_')[1]
            try:
                data_dict = eval(pkg_data)
                publish_imu(data_dict, seq)
                publish_range(data_dict, seq)
                seq += 1
            except:
                print("Error: improper data format received for bot id: %s" % dev_id_str)
                print("raw data:", pkg_data)
                # raise
        # if motion_cmd != past_motion_cmd:
        if "_" in motion_cmd:
            connection.sendall(motion_cmd + "/")
            past_motion_cmd = motion_cmd

        rate.sleep()

finally:
    # Clean up the connection
    connection.shutdown(socket.SHUT_RDWR)
    connection.close()
