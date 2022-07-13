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


class AnalyticModel:
    def __init__(self, frame_id, num_of_bots, origin_tag_id):
        self.num_of_bots = int(num_of_bots)
        self.listener = tf.TransformListener()
        self.origin_tag = origin_tag_id
        x0 = float(rospy.get_param('initial_pose/x'))
        y0 = float(rospy.get_param('initial_pose/y'))
        th0 = float(rospy.get_param('initial_pose/a'))
        self.base_link_pose_x = x0
        self.base_link_pose_y = y0
        self.base_link_orientation = th0
        self.rel_rotation_set = [False] * self.num_of_bots
        self.cam_trnsfrm_recieved = False
        self.heading_angles = np.array([])
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.t.header.frame_id = frame_id
        self.imu_quaternions = np.zeros((self.num_of_bots, 4))
        self.imu_quaternion_offsets = np.zeros((self.num_of_bots, 4))
        self.heading_angles_rel = np.zeros(self.num_of_bots)
        self.heading_angle_offsets = np.array([0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90])
        self.x_rel = np.zeros(self.num_of_bots)
        self.y_rel = np.zeros(self.num_of_bots)
        self.pose_offset_pub = rospy.Publisher("bot00_pose_offset", Pose, queue_size=50)

    def transform_broadcaster(self, bot_id):
        self.t.child_frame_id = "bot" + bot_id + "_analyt"
        bot_number = int(bot_id)
        self.t.header.stamp = rospy.Time.now()
        self.t.transform.translation.x = self.x_rel[bot_number] + self.base_link_pose_x
        self.t.transform.translation.y = self.y_rel[bot_number] + self.base_link_pose_y
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, (
                -self.heading_angle_offsets[bot_number] - self.heading_angles_rel[
            bot_number]) * np.pi / 180)
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]
        # self.t.transform.rotation.x = self.imu_quaternions[bot_id, 0]
        # self.t.transform.rotation.y = self.imu_quaternions[bot_id, 1]
        # self.t.transform.rotation.z = self.imu_quaternions[bot_id, 2]
        # self.t.transform.rotation.w = self.imu_quaternions[bot_id, 3]
        self.br.sendTransform(self.t)

    def base_link_pose(self, msg):
        data_dict = eval(msg.data)
        self.base_link_pose_x = (data_dict['00'][0] - data_dict[self.origin_tag][0]) / 100
        self.base_link_pose_y = -(data_dict['00'][1] - data_dict[self.origin_tag][1]) / 100
        self.base_link_orientation = -(data_dict['00'][2] - data_dict[self.origin_tag][2]) * 3.14 / 180

    def base_link_robot_loc(self, msg):
        self.base_link_pose_x = msg.transform.rotation.x
        self.base_link_pose_y = msg.transform.rotation.y
        e = tf_conversions.transformations.euler_from_quaternion_from(msg.transform.rotation.x,
                                                                      msg.transform.rotation.y,
                                                                      msg.transform.rotation.z,
                                                                      msg.transform.rotation.w)
        self.base_link_orientation = e[2]

    def base_link_odom(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.base_link_pose_x = msg.pose.pose.position.x
        self.base_link_pose_y = msg.pose.pose.position.y
        self.base_link_orientation = yaw
        # self.base_link_pose_x += msg.twist.twist.linear.x/1000
        # self.base_link_pose_y += -msg.twist.twist.linear.y/1000
        # self.base_link_orientation += msg.twist.twist.angular.z/10

    def update_imu_quaternions(self, msg):
        bot_id = msg.header.frame_id
        bot_number = int(bot_id[3:])
        if (not self.rel_rotation_set[bot_number]) and self.cam_trnsfrm_recieved:
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, -self.heading_angles_rel[
                bot_number] * np.pi / 180 + self.base_link_orientation)
            q1_inv = [msg.orientation.x, msg.orientation.y, msg.orientation.z, - msg.orientation.w]
            q2 = [q[0], q[1], q[2], q[3]]
            self.imu_quaternion_offsets[bot_number, :] = tf.transformations.quaternion_multiply(q2, q1_inv)
            self.rel_rotation_set[bot_number] = True
        self.imu_quaternions[bot_id, :] = tf.transformations.quaternion_multiply(self.qr,
                                                                                 [msg.orientation.x, msg.orientation.y,
                                                                                  msg.orientation.z, msg.orientation.w])

    def analyt_model(self, X):
        L = []
        Theta = []
        lmax = 20
        X = X.reshape(12)
        for n in range(12):
            if n < 11:
                l = lmax * np.abs(np.sin((X[1 + n] - X[n] + 180) * np.pi / 360))
            else:
                l = lmax * np.abs(np.sin((X[0] - X[11] + 180) * np.pi / 360))
            L = np.append(L, l)
        for n in range(12):
            offset0 = 0 * (n % 2 == 0) + 90 * (n % 2 > 0)
            offset1 = 0 * ((n + 1) % 2 == 0) + 90 * ((n + 1) % 2 > 0)
            if n < 11:
                Theta = np.append(Theta, np.arctan2(
                    np.sin((X[n + 0] + 90 + offset0) * np.pi / 180) + np.sin(
                        (X[n + 1] + 90 + offset1) * np.pi / 180),
                    np.cos((X[n + 0] + 90 + offset0) * np.pi / 180) + np.cos(
                        (X[n + 1] + 90 + offset1) * np.pi / 180)))
            else:
                Theta = np.append(Theta,
                                  np.arctan2(np.sin((X[11] + 180) * np.pi / 180) + np.sin((X[0] + 90) * np.pi / 180),
                                             np.cos((X[11] + 180) * np.pi / 180) + np.cos((X[0] + 90) * np.pi / 180)))
        # print(Theta.shape)
        B = np.vstack((-np.cos(Theta), -np.sin(Theta))).T
        rel_positions = np.zeros([12, 2])
        l = 12
        for j in range(0, 11):
            rel_positions[j + 1, :] = rel_positions[j, :] + l * B[j, :]
        if np.abs(np.linalg.norm(rel_positions[0, :] - rel_positions[-1, :]) - l) > 3:
            rel_positions[-1, :] = 0.5 * (rel_positions[-2, :] + rel_positions[0, :])

        return rel_positions

    def update_aprilt_state_values(self, data):
        data_dict = eval(data.data)
        # self.base_link_pose_x = (data_dict['00'][0] - data_dict[self.origin_tag][0]) / 100
        # self.base_link_pose_y = -(data_dict['00'][1] - data_dict[self.origin_tag][1]) / 100
        # self.base_link_orientation = -(data_dict['00'][2] - data_dict[self.origin_tag][2]) * 3.14 / 180
        for i in range(self.num_of_bots):
            # self.heading_angles_rel[i] = (data_dict[self.key(i)][2] - data_dict['00'][2])
            self.heading_angles_rel[i] = (data_dict[self.key(i)][2])
        analyt_input = np.array(self.heading_angles_rel).reshape((1, 12))
        rel_poses = self.analyt_model(analyt_input)
        self.x_rel = rel_poses[:, 0] / 100
        self.y_rel = - rel_poses[:, 1] / 100
        for i in range(1, self.num_of_bots):
            self.transform_broadcaster(self.key(i))
        pose_offset = Pose()
        pose_offset.position.x = np.mean(self.x_rel) - self.x_rel[0]
        pose_offset.position.y = np.mean(self.y_rel) - self.y_rel[0]
        pose_offset.position.z = -(data_dict['00'][2] - data_dict[self.origin_tag][2])
        self.pose_offset_pub.publish(pose_offset)

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
    rospy.init_node('dynamic_tf2_broadcaster', anonymous=True)
    listener = tf.TransformListener()
    num_of_bots = sys.argv[1]
    frame_id = sys.argv[2]
    origin_tag = sys.argv[3]
    # child_frames = rospy.get_param('~child_frames')
    broadcaster = AnalyticModel(frame_id, num_of_bots, origin_tag)
    # for i in child_frames:
    #     rospy.Subscriber('%s/imu/data' % bot_id(i), Imu, broadcaster.update_imu_quaternions)
    rospy.Subscriber("state", String, broadcaster.update_aprilt_state_values)
    # rospy.Subscriber("odometry/filtered_map",
    #                  Odometry,
    #                  broadcaster.base_link_odom)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/odom', '/bot00_analyt', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            broadcaster.base_link_pose_x = trans[0]
            broadcaster.base_link_pose_y = trans[1]
            broadcaster.base_link_orientation = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
