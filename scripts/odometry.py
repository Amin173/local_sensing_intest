#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, PoseWithCovarianceStamped
import numpy as np
from std_msgs.msg import String
from numpy import array

class odomBroadcaster:

    def __init__(self):
        self.num_of_bots = 12
        """Initial state"""
        x0 = float(rospy.get_param('initial_pose/x'))
        y0 = float(rospy.get_param('initial_pose/y'))
        th0 = float(rospy.get_param('initial_pose/a'))

        self.x = x0
        self.y = y0
        self.th = th0

        # self.x_vel_model = x0
        # self.y_vel_model = y0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.heading = np.zeros(self.num_of_bots)
        self.prev_heading = self.heading
        self.range_data = np.ones(self.num_of_bots)
        self.des_dir = [1, 0]
        self.prev_dir = [1, 0]
        self.stacked_normal_distances = []
        self.prev_mean = 0
        self.obstacle_avoidance_thrld = 0.05

        self.last_time = rospy.Time.now()
        self.current_time = rospy.Time.now()
        self.r = rospy.Rate(10)

        # Publishers
        self.odom_pub = rospy.Publisher("odom/vel_model", Odometry, queue_size=50)
        self.pub_ground_truth = rospy.Publisher("pose/ground_truth", PoseWithCovarianceStamped, queue_size=50)
        # self.pub_vel_model = rospy.Publisher("pose/vel_model", PoseStamped, queue_size=50)

        self.odom_broadcaster = tf.TransformBroadcaster()
        self.t0 = rospy.Time.now()
        self.bot00_offset_x = 0
        self.bot00_offset_y = 0
        self.bot00_quat = None
        self.bot00_offset_x_array = np.array([0, 0])
        self.bot00_offset_y_array = np.array([0, 0])
        self.cmd = np.zeros(12)
        self.v_odom = [0.0]
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat,
            self.t0,
            "bot00_analyt",
            "odom"
        )

        self.origin_x = 0
        self.origin_y = 0
        self.origin_theta = 0

        # Subscribers
        rospy.Subscriber("state", String, self.publish_ground_truth)
        rospy.Subscriber("locomotion_stats", String, self.update_des_dir)
        rospy.Subscriber("bot00_pose_offset", Pose, self.update_bot00_offset)
        rospy.Subscriber("imu_heading_angles", String, self.update_heading_angle)
        rospy.Subscriber("control_cmd", String, self.update_cmd)
        # rospy.Subscriber("/bot00/imu_pose", PoseStamped, self.update_bot00_quat)

        # rospy.Subscriber("motion_model", Twist, self.update_from_vel_model)
        # rospy.Subscriber("point_cloud_pose_est", PoseWithCovarianceStamped, self.update_pos_reg)
        # for i in range(self.num_of_bots):
        #     rospy.Subscriber("bot%s/dist/data" % self.key(i), Range, self.update_range_sensors)
        # rospy.Subscriber("bot%s/imu/data" % self.key(i), Imu, self.update_range_sensors)

    def publish_ground_truth(self, msg):
        data_dict = eval(msg.data)
        # self.prev_heading = heading
        qx = np.zeros(self.num_of_bots + 1)
        qz = np.zeros(self.num_of_bots + 1)
        heading = np.zeros(self.num_of_bots + 1)
        # self.last_time = self.current_time
        for i in range(self.num_of_bots + 1):
            qx[i] = data_dict[self.key(i)][0]
            qz[i] = data_dict[self.key(i)][1]
            heading[i] = data_dict[self.key(i)][2]
        ground_truth = PoseWithCovarianceStamped()
        ground_truth.header.frame_id = 'map'
        ground_truth.header.stamp = rospy.Time.now()
        q = tf.transformations.quaternion_from_euler(0, 0, -(heading[0]-heading[12])*np.pi/180)
        ground_truth.pose.pose.position.x = (qx[0]-qx[12])/100
        ground_truth.pose.pose.position.y = -(qz[0]-qz[12])/100
        ground_truth.pose.pose.position.z = 0
        ground_truth.pose.pose.orientation.x = q[0]
        ground_truth.pose.pose.orientation.y = q[1]
        ground_truth.pose.pose.orientation.z = q[2]
        ground_truth.pose.pose.orientation.w = q[3]
        pose_covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0000, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0000, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0000, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]
        ground_truth.pose.covariance = pose_covariance
        self.origin_x = data_dict["12"][0]
        self.origin_y = data_dict["12"][1]
        self.origin_theta = data_dict["12"][2]
        self.pub_ground_truth.publish(ground_truth)

    def update_range_sensors(self, msg):
        bot_id = msg.header.frame_id
        bot_number = int(bot_id[3:])
        self.range_data[bot_number] = msg.range

    def update_heading_angle(self, msg):
        self.prev_heading = self.heading
        data_dict = eval(msg.data)
        for i, key in enumerate(data_dict.keys()):
            self.heading[i] = data_dict[key]

    def update_bot00_quat(self, msg):
        self.bot00_quat = [0, 0, msg.pose.orientation.z, msg.pose.orientation.w]

    def update_bot00_offset(self, msg):
        self.bot00_offset_x = msg.position.x * 100
        self.bot00_offset_y = msg.position.y * 100

    def update_des_dir(self, msg):
        data_dict = eval(msg.data)
        self.des_dir = data_dict['dir']
        V = 0.7
        self.vx = self.v_odom[0] * self.des_dir[0]
        self.vy = self.v_odom[0] * self.des_dir[1]
        # self.vx = 0.8 * self.des_dir[0]
        # self.vy = 0.8 * self.des_dir[1]

    def update_cmd(self, msg):
        df_cmd = msg.data
        dict = eval(df_cmd)
        cmd_i = []
        for j in range(12):
            cmd_i = np.append(cmd_i, dict[self.key(j)][0])
        self.cmd = cmd_i

    def key(self, i):
        if i < 10:
            key = str(0) + str(i)
        else:
            key = str(i)
        return key

    def update(self):
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()

            # compute odometry in a typical way given the velocities of the robot
            theta = - (self.heading - self.origin_theta + np.array([0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90])) * np.pi / 180
            # heading_dirs = np.vstack(([np.cos(theta)], [np.sin(theta)])).T
            dt = (self.current_time - self.last_time).to_sec()

            if self.bot00_offset_x_array[1] == 0:
                self.bot00_offset_x_array[0] = self.bot00_offset_x
                self.bot00_offset_y_array[0] = self.bot00_offset_y
                self.bot00_offset_x_array[1] = self.bot00_offset_x
                self.bot00_offset_y_array[1] = self.bot00_offset_y
            else:
                self.bot00_offset_x_array[0] = self.bot00_offset_x_array[1]
                self.bot00_offset_y_array[0] = self.bot00_offset_y_array[1]
                self.bot00_offset_x_array[1] = self.bot00_offset_x
                self.bot00_offset_y_array[1] = self.bot00_offset_y
            
            heading_dirs = np.vstack((np.cos(theta), np.sin(theta)))
            rel_headings = np.zeros([1, 12])
            for i in range(12):
                """
                TO DO: the minus sign in the equation below should be changed to + (it is a dot prodeuct).
                This is likely a probem with the heading anlge signs.
                """
                rel_headings[0, i] = heading_dirs[0, i] * self.des_dir[0] - heading_dirs[1, i] * self.des_dir[1]
            # rel_headings = np.dot(heading_dirs.reshape(12, 2), des_dir_array)
            if not sum(self.cmd) ==0:
                self.v_odom = -3.81*(np.mean(abs(rel_headings * self.cmd), axis=1)) + 3.25
            # rospy.loginfo(f"v_odom: {self.v_odom}")

            self.vx_or = -(self.bot00_offset_x_array[1] - self.bot00_offset_x_array[0]) / dt
            self.vy_or = -(self.bot00_offset_y_array[1] - self.bot00_offset_y_array[0]) / dt

            self.vth = ((-self.heading[0] + self.prev_heading[0])*np.pi/180) / dt
            delta_x = (self.vx + self.vx_or) * dt / 100
            delta_y = - (self.vy - self.vy_or) * dt / 100

            self.x += delta_x
            self.y += delta_y
            self.th = theta[0]

            # self.x_vel_model += delta_x
            # self.y_vel_model += delta_y

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
            # if self.bot00_quat:
            #     # rospy.logwarn(f"{odom_quat}")
            #     # rospy.loginfo(f"{self.bot00_quat}")
            #     odom_quat = self.bot00_quat

            pose_covariance = [0.0005, 0.0, 0.0, 0.0, 0.0, 0.0,
                                               0.0, 0.0005, 0.0, 0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0000, 0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0, 0.0000, 0.0, 0.0,
                                               0.0, 0.0, 0.0, 0.0, 0.0000, 0.0,
                                               0.0, 0.0, 0.0, 0.0, 0.0, 0.001]

            twist_covariance = [0.00001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                               0.0, 0.00001, 0.0, 0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0000, 0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0, 0.0000, 0.0, 0.0,
                                               0.0, 0.0, 0.0, 0.0, 0.0000, 0.0,
                                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]

            # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.0),
                odom_quat,
                self.current_time,
                "bot00_analyt",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
            odom.pose.covariance = pose_covariance

            # set the velocity
            odom.child_frame_id = "bot00_analyt"
            odom.twist.twist = Twist(
                Vector3((self.vx + self.vx_or)/100, -(self.vy - self.vy_or)/100, 0),
                Vector3(0, 0, self.vth))
            odom.twist.covariance = twist_covariance

            # publish the message
            self.odom_pub.publish(odom)

            # vel_model = PoseStamped()
            # vel_model.header.frame_id = 'map'
            # vel_model.header.stamp = rospy.Time.now()
            # vel_model.pose = Pose(Point(self.x, self.y, 0.),
            #                          Quaternion(*odom_quat))
            # self.pub_vel_model.publish(vel_model)

            self.last_time = self.current_time
            self.r.sleep()


if __name__ == '__main__':
    rospy.init_node('odometry_publisher')
    odom_pub = odomBroadcaster()
    odom_pub.update()
