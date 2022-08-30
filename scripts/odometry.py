#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, PoseWithCovarianceStamped
import numpy as np
from std_msgs.msg import String
from numpy import array
import sys
from time import sleep

class odomBroadcaster:

    def __init__(self, num_nodes=12):
        self.num_of_bots = num_nodes
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

        self.heading = np.zeros(self.num_of_bots + 1)
        self.prev_heading = self.heading
        self.qx = np.zeros(self.num_of_bots + 1)
        self.qz = np.zeros(self.num_of_bots + 1)
        self.range_data = np.ones(self.num_of_bots)
        self.des_dir = [1, 0]
        self.prev_dir = [1, 0]
        self.stacked_normal_distances = []
        self.prev_mean = 0
        self.obstacle_avoidance_thrld = 0.05

        self.last_time = rospy.Time.now()
        self.current_time = rospy.Time.now()
        self.r = rospy.Rate(2)

        # Publishers
        self.odom_pub = rospy.Publisher("odom/vel_model", Odometry, queue_size=50)
        self.pub_ground_truth = rospy.Publisher("pose/ground_truth", PoseWithCovarianceStamped, queue_size=50)
        # self.pub_vel_model = rospy.Publisher("pose/vel_model", PoseStamped, queue_size=50)

        self.odom_broadcaster = tf.TransformBroadcaster()
        self.t0 = rospy.Time.now()
        self.bott00_offset_x = 0
        self.bott00_offset_y = 0
        self.bott00_offset_x_array = np.array([0, 0])
        self.bott00_offset_y_array = np.array([0, 0])
        self.new_data = False

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat,
            self.t0,
            "bot00_analyt",
            "odom"
        )

        # Subscribers
        rospy.Subscriber("state", String, self.update_headings)
        rospy.Subscriber("locomotion_stats", String, self.update_des_dir)
        rospy.Subscriber("bot00_pose_offset", Pose, self.update_bot00_offset)


    # Gets the pose estimate from 
    def update_headings(self, msg):
        data_dict = eval(msg.data)
        self.prev_heading = self.heading
        # self.last_time = self.current_time
        for i in range(self.num_of_bots + 1):
            self.qx[i] = data_dict[self.key(i)][0]
            self.qz[i] = data_dict[self.key(i)][1]
            # heading of each sub-unit
            self.heading[i] = data_dict[self.key(i)][2]
        ground_truth = PoseWithCovarianceStamped()
        ground_truth.header.frame_id = 'map'
        ground_truth.header.stamp = rospy.Time.now()
        q = tf.transformations.quaternion_from_euler(0, 0, -(self.heading[0]-self.heading[-1])*np.pi/180)
        ground_truth.pose.pose.position.x = (self.qx[0]-self.qx[-1])/100
        ground_truth.pose.pose.position.y = -(self.qz[0]-self.qz[-1])/100
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
        self.pub_ground_truth.publish(ground_truth)

    # Position of base link calculated with slam
    def update_bot00_offset(self, msg):
        self.bott00_offset_x = msg.position.x * 100
        self.bott00_offset_y = msg.position.y * 100
        self.new_data = True

    # updates the desired direction according to the controller
    def update_des_dir(self, msg):
        data_dict = eval(msg.data)
        self.des_dir = data_dict['dir']
        V = 0.7
        self.vx = V * self.des_dir[0]
        self.vy = V * self.des_dir[1]

    def key(self, i):
        if i < 10:
            key = str(0) + str(i)
        else:
            key = str(i)
        return key

    # Calculates next step position of base link
    def update(self):
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()

            # Correct the heading angles of the sub-units with the pi/2 shift
            theta = (self.heading + (np.arange(self.num_of_bots + 1)%2) * 90) * np.pi / 180.


            # heading_dirs = np.vstack(([np.cos(theta)], [np.sin(theta)])).T
            dt = (self.current_time - self.last_time).to_sec()

            # Calculate the displacement of the base link from the previous step and the current step
            if self.bott00_offset_x_array[1] == 0:
                self.bott00_offset_x_array[0] = self.bott00_offset_x
                self.bott00_offset_y_array[0] = self.bott00_offset_y
                self.bott00_offset_x_array[1] = self.bott00_offset_x
                self.bott00_offset_y_array[1] = self.bott00_offset_y
            else:
                #if self.new_data:
                self.bott00_offset_x_array[0] = self.bott00_offset_x_array[1]
                self.bott00_offset_y_array[0] = self.bott00_offset_y_array[1]
                self.bott00_offset_x_array[1] = self.bott00_offset_x
                self.bott00_offset_y_array[1] = self.bott00_offset_y
                #    self.new_data = False

            # Calculate average velocity with the last two steps
            # TODO: add filter
            self.vx_or = -(self.bott00_offset_x_array[1] - self.bott00_offset_x_array[0]) / dt
            self.vy_or = -(self.bott00_offset_y_array[1] - self.bott00_offset_y_array[0]) / dt

           

            # vth is the angular velocity of the base link sub-unit in rad / s
            self.vth = ((-self.heading[0] + self.prev_heading[0])*np.pi/180) / dt


            delta_x = (self.vx + self.vx_or) * dt / 100
            delta_y = -(self.vy - self.vy_or) * dt / 100

            self.x += delta_x
            self.y += delta_y
            self.th = -(theta[0] - theta[-1])       # Last element in theta is the reference tag position
            
            #rospy.logerr("Average speed: %s", str(len(theta)))

            # self.x_vel_model += delta_x
            # self.y_vel_model += delta_y

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

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
    sleep(1.5)
    rospy.init_node('odometry_publisher')
    try:
        num_nodes = int(sys.argv[1])
        odom_pub = odomBroadcaster(num_nodes)
    except:
        odom_pub = odomBroadcaster()
    odom_pub.update()
