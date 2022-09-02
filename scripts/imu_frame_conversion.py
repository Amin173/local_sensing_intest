#!/usr/bin/env python

import rospy
import tf_conversions
# from tensorflow.keras.models import load_model
from tf.transformations import *
import sys
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose2D, PoseStamped, Pose, Quaternion
from nav_msgs.msg import Odometry
import tf
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import String
import numpy as np
import traceback


# def publish_imu(msg):
#     imu_msg = Imu()
#     imu_msg.header.stamp = rospy.Time.now()
#     imu_msg.header.frame_id = str(msg.header.frame_id) + "_analyt"
#     imu_msg.header.seq = msg.header.seq
#     id_num = int(str(msg.header.frame_id)[3:])
#     imu_msg.orientation.x = msg.orientation.x
#     imu_msg.orientation.y = msg.orientation.y
#     imu_msg.orientation.z = msg.orientation.z
#     imu_msg.orientation.w = msg.orientation.w

#     if id_num % 2 == 0:
#         imu_msg.linear_acceleration.x = -msg.linear_acceleration.y / 100
#         imu_msg.linear_acceleration.y = msg.linear_acceleration.x / 100
#         imu_msg.linear_acceleration.z = 0
#     else:
#         imu_msg.linear_acceleration.x = -msg.linear_acceleration.x / 100
#         imu_msg.linear_acceleration.y = -msg.linear_acceleration.y / 100
#         imu_msg.linear_acceleration.z = 0
#     imu_pub[key(id_num)].publish(imu_msg)

#     def compute_heading_angle_change(self, bot_id, msg, max_rot=np.pi/3):
#         prev_msg = self.imu_msgs[self.key(bot_id)]
#         qr = tf.transformations.quaternion_multiply([msg.orientation.x, msg.orientation.y,
#                                                     msg.orientation.z, msg.orientation.w], 
#                                                     [prev_msg.orientation.x, prev_msg.orientation.y,
#                                                     prev_msg.orientation.z, -prev_msg.orientation.w])
#         self.euler_change[bot_id, :] = euler_from_quaternion(qr)
#         filtered_euler = self.filter_imu_data(self.euler_change)
#         self.q_change[bot_id, :] = quaternion_from_euler(filtered_euler[bot_id, 0], filtered_euler[bot_id, 1], filtered_euler[bot_id, 2])
#         self.imu_msgs[self.key(bot_id)] = msg
#         self.imu_quaternions[bot_id, :] = tf.transformations.quaternion_multiply(self.q_change[bot_id, :],
#                                                                     self.imu_quaternions[bot_id, :])    

class AnalyticModel:
    def __init__(self, frame_id, num_of_bots, origin_tag_id):
        self.num_of_bots = int(num_of_bots)
        self.listener = tf.TransformListener()
        self.origin_tag = origin_tag_id
        self.rel_rotation_set = [0] * self.num_of_bots
        self.cam_trnsfrm_recieved = False
        self.heading_angles = np.array([])
        self.imu_quaternions = np.zeros((self.num_of_bots, 4))
        self.imu_quaternion_offsets = np.zeros((self.num_of_bots, 4))
        self.heading_angles_rel = np.zeros(self.num_of_bots)
        self.heading_angle_offsets = np.array([0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90])
        self.imu_pose_pub = {}
        for i in range(self.num_of_bots):
            self.imu_pose_pub[self.key(i)] = rospy.Publisher("bot%s/imu_pose" % self.key(i), PoseStamped, queue_size=10)
        self.data_dict = {'12': (0, 0, 0)}
        self.all_imus_initiated = False
        self.imu_initilization_steps = 20
        self.imu_msgs = {}
        self.q_change = np.zeros([self.num_of_bots, 4])
        self.euler_change = np.zeros([self.num_of_bots, 3])
        self.heading_angled_change = np.zeros(self.num_of_bots)
        self.last = rospy.Time.now().to_sec()

    def compute_heading_angle_change(self, bot_id, msg, max_rot=np.pi/3):
        prev_msg = self.imu_msgs[self.key(bot_id)]
        qr = tf.transformations.quaternion_multiply([msg.orientation.x, msg.orientation.y,
                                                    msg.orientation.z, msg.orientation.w], 
                                                    [prev_msg.orientation.x, prev_msg.orientation.y,
                                                    prev_msg.orientation.z, -prev_msg.orientation.w])
        self.euler_change[bot_id, :] = euler_from_quaternion(qr)
        filtered_euler = self.filter_imu_data(self.euler_change)
        self.q_change[bot_id, :] = quaternion_from_euler(filtered_euler[bot_id, 0], filtered_euler[bot_id, 1], filtered_euler[bot_id, 2])
        self.imu_msgs[self.key(bot_id)] = msg
        self.imu_quaternions[bot_id, :] = tf.transformations.quaternion_multiply(self.q_change[bot_id, :],
                                                                    self.imu_quaternions[bot_id, :])   

    def filter_imu_data(self, dt):
        max_rot = 1
        yaws = dt[:, 2]
        yaws = np.array(yaws)
        yaws[0] = yaws[0] * (abs(yaws[0])<max_rot) + 0 * (abs(yaws[0])>=max_rot)
        dt[:, 2] = yaws
        return dt

    def imu_callback(self, msg):
        bot_id = msg.header.frame_id
        bot_number = int(bot_id[3:])

        imu_pose = PoseStamped()
        imu_pose.header.stamp = rospy.Time.now()
        imu_pose.header.frame_id = "odom"
        imu_pose.header.seq = msg.header.seq
        id_num = int(str(msg.header.frame_id)[3:])

        try:
            (trans, rot) = listener.lookupTransform('odom', str(msg.header.frame_id)+"_analyt", rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            imu_pose.pose.position.x = trans[0]
            imu_pose.pose.position.y = trans[1]
            imu_pose.pose.position.z = trans[2]
        except:
            pass
        self.rel_rotation_set[bot_number] += 1
        self.all_imus_initiated =  all(np.array(self.rel_rotation_set) >self.imu_initilization_steps)
        # self.all_imus_initiated = False
        if not all(np.array(self.rel_rotation_set) >self.imu_initilization_steps) and self.cam_trnsfrm_recieved:
            try:
                if bot_number % 2 == 0:
                    q = tf_conversions.transformations.quaternion_from_euler(0, 0, -(self.data_dict[bot_id[3:]][2] - self.data_dict[self.origin_tag][2])*np.pi/180)
                else:
                    q = tf_conversions.transformations.quaternion_from_euler(0, 0, -(self.data_dict[bot_id[3:]][2] - self.data_dict[self.origin_tag][2])*np.pi/180-np.pi/2)
                q1_inv = [msg.orientation.x, msg.orientation.y, msg.orientation.z, - msg.orientation.w]
                q2 = [q[0], q[1], q[2], q[3]]
                self.imu_quaternion_offsets[bot_number, :] = tf.transformations.quaternion_multiply(q2, q1_inv)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"{e}")
        if all(np.array(self.rel_rotation_set) >20):
        # if False:
            self.compute_heading_angle_change(bot_number, msg)
        else:
            self.imu_quaternions[bot_number, :] = tf.transformations.quaternion_multiply(self.imu_quaternion_offsets[bot_number, :],
                                                            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            self.imu_msgs[self.key(bot_number)] = msg
        imu_pose.pose.orientation.x = self.imu_quaternions[bot_number, 0]
        imu_pose.pose.orientation.y = self.imu_quaternions[bot_number, 1]
        imu_pose.pose.orientation.z = self.imu_quaternions[bot_number, 2]
        imu_pose.pose.orientation.w = self.imu_quaternions[bot_number, 3]
        self.imu_pose_pub[self.key(id_num)].publish(imu_pose)

    def update_aprilt_state_values(self, data):
        self.cam_trnsfrm_recieved = True
        data_dict = eval(data.data)
        self.data_dict = data_dict
        if not self.all_imus_initiated:
            for i in range(self.num_of_bots):
                # self.heading_angles_rel[i] = (data_dict[self.key(i)][2] - data_dict['12'][2])
                self.heading_angles_rel[i] = (data_dict[self.key(i)][2])

    def update_heading_angle(self, msg):
        self.prev_heading = self.heading
        data_dict = eval(msg.data)
        for i, key in enumerate(data_dict.keys()):
            self.heading[i] = data_dict[key]

    def key(self, i):
        if i < 10:
            key = str(0) + str(i)
        else:
            key = str(i)
        return key

def bot_id(i):
    if i < 10:
        id1 = "bot" + str(0) + str(i)
    else:
        id1 = "bot" + str(i)
    return id1

if __name__ == '__main__':
    rospy.init_node('imu_frame_conversion', anonymous=True)
    listener = tf.TransformListener()
    num_of_bots = sys.argv[1]
    frame_id = sys.argv[2]
    origin_tag = sys.argv[3]
    child_frames = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
    broadcaster = AnalyticModel(frame_id, num_of_bots, origin_tag)
    
    for i in child_frames:
        rospy.Subscriber('%s/imu/data' % bot_id(i), Imu, broadcaster.imu_callback)
    rospy.Subscriber("state", String, broadcaster.update_aprilt_state_values)
    heading_pub = rospy.Publisher("imu_heading_angles", String, queue_size=10)
    imu_quat_pub = rospy.Publisher("imu_quaternions", String, queue_size=10)

    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        try:
            for i in range(broadcaster.num_of_bots):
                euler = tf.transformations.euler_from_quaternion(broadcaster.imu_quaternions[i, :].reshape(4))
                broadcaster.heading_angles_rel[i] = -euler[2] * 180 / np.pi - broadcaster.heading_angle_offsets[i] + broadcaster.data_dict[broadcaster.origin_tag][2]

            heading_angles_dict = {}
            imu_quats_dict = {}
            for i in range(broadcaster.num_of_bots):
                heading_angles_dict[broadcaster.key(i)] = broadcaster.heading_angles_rel[i]
                imu_quats_dict[broadcaster.key(i)] = broadcaster.imu_quaternions[i, :]

            if broadcaster.all_imus_initiated:
                heading_pub.publish(str(heading_angles_dict))
                imu_quat_pub.publish(str(imu_quats_dict))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
