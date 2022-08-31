
#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu
import numpy as np
from std_msgs.msg import String
from numpy import array
import sys
from time import sleep
import tf_conversions

class odomBroadcaster:

    def __init__(self, num_nodes=12, estimator_model='linear'):

        self.num_of_bots = num_nodes

        # define initial pose variables, positions and velocities
        self.x = float(rospy.get_param('initial_pose/x'))
        self.y = float(rospy.get_param('initial_pose/y'))
        self.th = float(rospy.get_param('initial_pose/a'))
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        # Subscriber data variables: angles are stored as complex variables
        self.th_imu_data = np.ones(self.num_of_bots) + 0j
        self.vth_imu_data = np.zeros(self.num_of_bots)

        self.xy_april_data = np.zeros((self.num_of_bots, 2))
        self.th_april_data = np.ones(self.num_of_bots) + 0j
        self.vxvy_april_data = np.zeros((self.num_of_bots, 2))
        self.vth_april_data = np.zeros(self.num_of_bots)

        self.control_dir = np.array([0, -1])
        self.control_actions = np.array(self.num_of_bots)

        # Timing variables for subscribers
        tmp = rospy.Time.now()
        self.time_imu_list = [tmp] * self.num_of_bots
        self.time_april = tmp

        self.r = rospy.Rate(10)

        # TF variables:
        
        self.odom_broadcaster = tf.TransformBroadcaster()
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat,
            rospy.Time.now(),
            "bot00_analyt",
            "odom"
        )
        self.t = TransformStamped()
        self.t.header.frame_id = self.key(self.num_of_bots)
        
        # Variables to optimize model:
        # Max distance between bots
        self.lmax = .15/2 * np.tan(np.pi / self.num_of_bots) / np.sin(np.pi / 12)
        self.heading_angle_offsets = np.exp(1j * np.pi / 2 * (np.arange(self.num_of_bots) % 2)) #* np.exp(1j * np.pi / 2)

        if type(estimator_model) == None:
            self.estimator_model = 'linear'
        else:
            self.estimator_model = estimator_model

        # Subscribers
        # Subscriber for apriltags
        rospy.Subscriber("state", String, self.update_april_data)

        # Subscrber for imu
        for i in range(self.num_of_bots):
            idx = self.key(i)
            rospy.Subscriber("bot%s/imu/data" % idx, Imu, self.update_imu_data, idx)

        # Subscriber for controller
        rospy.Subscriber("locomotion_stats", String, self.update_des_dir)

        # Publishers
        self.odom_publisher = rospy.Publisher("odom/vel_model", Odometry, queue_size=50)
        self.pub_ground_truth = rospy.Publisher("pose/ground_truth", PoseWithCovarianceStamped, queue_size=50)
        

    # update imu data
    def update_imu_data(self, msg, idx):
        # Bot number
        i = int(idx)
        
        tmp = rospy.Time.now()
        dt = (tmp - self.time_imu_list[i]).to_sec()
        self.time_imu_list[i] = tmp

        # Convert quaternion to angle
        ang_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler = np.exp(1j * tf.transformations.euler_from_quaternion(ang_list)[-1])

        # Store result for the specified bot
        self.vth_imu_data[i] = np.angle(euler / self.th_imu_data[i]) / dt
        self.th_imu_data[i] = euler

    # Update apriltag data
    def update_april_data(self, msg):
        # First calculate increment in time:
        tmp = rospy.Time.now()
        dt = (tmp - self.time_april).to_sec()
        self.time_april = tmp

        # Convert data to usable type
        data_dict = eval(msg.data)
        tmp = data_dict[self.key(self.num_of_bots)]
        xy0 = np.array(tmp[:2])
        th0 = np.array(tmp[2])
        xy_l = np.empty((2, 0), float)
        th_l = np.empty((0), float)
        for i in range(self.num_of_bots):
            data_tmp = data_dict[self.key(i)]
            xyi = np.array(data_tmp[:2]) - xy0
            thi = np.exp(1j * (np.array(data_tmp[2]) - th0) * np.pi / 180)

            xy_l = np.hstack((xy_l, xyi.reshape((2, -1))))
            th_l = np.hstack((th_l, [thi]))

        # store data into lists and calculate velocity
        self.vxvy_april_data = (xy_l.T - self.xy_april_data) / dt
        self.xy_april_data = xy_l.T

        self.vth_april_data = np.angle(th_l / self.th_april_data) / dt
        self.th_april_data = th_l

    # Update controller info
    def update_des_dir(self, msg):
        data_dict = eval(msg.data)
        self.control_dir = data_dict['dir']
        self.control_actions = data_dict['actions']


    # Estimates the relative positions of bots given the heading directions
    def analyt_model(self, X, best_fit=True):
        # Bot normal direction angles
        X = np.angle(X.reshape(self.num_of_bots))
        
        ### Precalculate variables that will later be nidded
        # Calculate angles, offset 90 deg for odd numbered bots and constrain between - pi to pi=
        normals = X

        # Calculate difference in angle, constrain between - pi to pi
        diff = np.roll(normals, -1) - normals
        diff += (diff > np.pi) * (- 2 * np.pi) + (diff < - np.pi) * (2 * np.pi)

        # Calculate direction to next bot:
        theta = np.exp(1j * np.roll(normals, -1)) + np.exp(1j * normals)
        theta = np.angle(theta) + np.pi
        #theta += (theta > np.pi) * (- 2 * np.pi) + (theta < - np.pi) * (2 * np.pi)
        
        

        # Method to calculate distance between bots
        if self.estimator_model == 'rigid_joint':
            L = 2 * self.lmax * np.cos(.5 * diff)
        elif self.estimator_model == 'linear':         # ransac regressor
            L = (155.93 - 0.8547 * 180 / np.pi * np.abs(diff)) / 1000.
        else:
            L = 12**2 / self.num_of_bots/1000

        Ct = np.cos(theta)#np.cos(theta)
        St = np.sin(theta)#np.sin(theta)
        # Distance vector from one bot to the next
        B = np.vstack((Ct, St))
        
        # Apply best fit to the estimator model
        if best_fit:
            C = Ct.reshape((-1, self.num_of_bots))
            S = St.reshape((-1, self.num_of_bots))

            # Calculate the inverse of the covariances
            COVi = np.eye(self.num_of_bots)

            # Values to be used in operation
            CCT = np.sum(C**2)#(C @ COVi @ C.T)[0, 0]
            SST = np.sum(S**2)#(S @ COVi @ S.T)[0, 0]
            SCT = np.sum(C*S)#(S @ COVi @ C.T)[0, 0]
            CST = SCT#(C @ COVi @ S.T)[0, 0]

            P = (SCT / CCT * C - S) / (SCT * CST / CCT - SST)
            Q = (C - CST * P) / CCT

            # Apply adjustment
            #x = - COVi @ (C.T @ Q @ L + S.T @ P @ L)
            x = - ((C.T @ Q + S.T @ P) @ L)
            L += x
            
        
        # Update vectors that take you from one bot to the next
        B = (L * B).T

        # calculate the positions of each bot
        rel_positions = np.vstack((np.zeros((1, 2)), np.cumsum(B, axis=0)[:-1, :]))
        return rel_positions


    def key(self, i):
        if i < 10:
            key = str(0) + str(i)
        else:
            key = str(i)
        return key

    def publish_transform_rel_positions(self, positions, angles):
        # generate name
        for i in range(self.num_of_bots):
            idx = self.key(i)

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, np.angle(angles[i]) + np.pi / 2)
            self.odom_broadcaster.sendTransform(
                (positions[i, 0], positions[i, 1], 0.0),
                odom_quat,
                rospy.Time.now(),
                "bot" + idx + "_analyt2",
                "odom"
            )

    # Calculates next step position of base link
    def update(self):
        while not rospy.is_shutdown():
            # Select data to use to estimate shape

            angles = 1 / (self.th_april_data * self.heading_angle_offsets)
            p_rel = self.analyt_model(angles)

            self.publish_transform_rel_positions(p_rel, angles)


            self.r.sleep()


if __name__ == '__main__':
    sleep(1.5)
    rospy.init_node('odometry_publisher')
    try:
        #odom_pub = odomBroadcaster(int(sys.argv[1]), sys.argv[2])
        odom_pub = odomBroadcaster(24, "rigid_joint")
    except:
        odom_pub = odomBroadcaster()
    odom_pub.update()
