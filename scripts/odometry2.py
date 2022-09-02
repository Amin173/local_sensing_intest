
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
from scipy.signal import butter, lfilter, freqz, bessel


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, a, b, z=None):
    y = lfilter(b, a, np.array(data).reshape(-1), zi=z)
    return y, z

class odomBroadcaster:

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

    offsets_exp_imu = 1 / np.array([ 52694.50175353+57661.40458142j,  65848.6561348 +42018.62351792j,
                                     -2757.05888699-78063.31989653j,  36186.73233543-69222.5839325j,
                                     73506.09372357-26423.10766465j,  62052.50245221-47443.57507646j,
                                     31514.52143423+71469.62970958j,  64353.08234992-44275.04680041j,
                                     -31740.74226741+71371.41760756j,  43858.63270626+64634.40521362j,
                                     21682.67061298-75040.96016697j, -25168.84869007+73947.64673571j])

    def __init__(self, num_nodes=12, estimator_model='linear', data_source='simulation'):

        self.num_of_bots = num_nodes

        # define initial pose variables, positions and velocities
        self.xy = np.array([float(rospy.get_param('initial_pose/x')), float(rospy.get_param('initial_pose/y'))])
        self.th = 0.#float(rospy.get_param('initial_pose/a'))
        self.vxvy = np.array([0., 0.])
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

        # Filters
        fs = 5         #Copy from simulated_robot_tcp.py
        cutoff = 1.4    #Hz
        self.filter_b, self.filter_a = butter_lowpass(cutoff, fs, order=2)
        self.imu_zi = [None] * self.num_of_bots

        # Timing variables for subscribers
        tmp = rospy.Time.now()
        self.time_imu_list = [tmp] * self.num_of_bots
        self.time_april = tmp
        self.time_odom = tmp

        self.r = rospy.Rate(10)

        self.startup_sequence = True

        # TF variables:
        
        self.odom_broadcaster = tf.TransformBroadcaster()
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
            (self.xy[0], self.xy[1], 0.0),
            odom_quat,
            rospy.Time.now(),
            "bot_center",
            "odom"
        )
        self.t = TransformStamped()
        self.t.header.frame_id = self.key(self.num_of_bots)
        
        # Variables to optimize model:
        # Max distance between bots
        self.lmax = .15/2 * np.tan(np.pi / self.num_of_bots) / np.sin(np.pi / 12)
        self.heading_angle_offsets = np.exp(-1j * np.pi / 2 * (np.arange(self.num_of_bots) % 2))

        if type(estimator_model) == None:
            self.estimator_model = 'linear'
        else:
            self.estimator_model = estimator_model

        if type(data_source) == None:
            self.data_source = 'simulation'
        else:
            self.data_source = data_source

        # Subscribers
        # Subscriber for apriltags
        rospy.Subscriber("state", String, self.update_april_data)
        if self.data_source == 'experimental':
            rospy.Subscriber("control_cmd", String, self.update_control_command)

        # Subscrber for imu
        for i in range(self.num_of_bots):
            idx = self.key(i)
            rospy.Subscriber("bot%s/imu/data" % idx, Imu, self.update_imu_data, idx, queue_size=self.num_of_bots * 2)

        # Subscriber for controller
        rospy.Subscriber("locomotion_stats", String, self.update_des_dir)

        # Publishers
        self.odom_publisher = rospy.Publisher("odom/vel_model", Odometry, queue_size=50)
        self.pub_ground_truth = rospy.Publisher("pose/ground_truth", PoseWithCovarianceStamped)
        

    # update imu data
    def update_imu_data(self, msg, idx):
        # Bot number
        i = int(idx)
        
        tmp = rospy.Time.now()
        dt = (tmp - self.time_imu_list[i]).to_sec()
        self.time_imu_list[i] = tmp

        # Convert quaternion to angle
        ang_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        # Store result for the specified bot
        if self.data_source == 'experimental':
            euler = np.exp(1j * tf.transformations.euler_from_quaternion(ang_list)[-1]) #* np.exp(1j * np.pi / 2)
        else:
            euler = np.exp(-1j * tf.transformations.euler_from_quaternion(ang_list)[-1])
        # Apply filter:
        euler, self.imu_zi[i] =  butter_lowpass_filter(euler, self.filter_a, self.filter_b, self.imu_zi[i])

        # Store results
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
            if self.data_source == 'experimental':
                xyi /= 100
                xyi[1] = -xyi[1]
                thi = np.exp(-1j * (np.array(data_tmp[2]) - th0) * np.pi / 180)
            else:
                thi = np.exp(-1j * (np.array(data_tmp[2]) - th0) * np.pi / 180)

            xy_l = np.hstack((xy_l, xyi.reshape((2, -1))))
            th_l = np.hstack((th_l, [thi]))

        # store data into lists and calculate velocity
        self.vxvy_april_data = (xy_l.T - self.xy_april_data) / dt
        self.xy_april_data = xy_l.T

        self.vth_april_data = np.angle(th_l / self.th_april_data) / dt
        self.th_april_data = th_l

        # use apriltag data as initial position for the first step
        if self.startup_sequence:
            self.startup_sequence = False
            self.xy = np.average(self.xy_april_data, axis=0)

            

    # Update controller info
    def update_des_dir(self, msg):
        data_dict = eval(msg.data)
        self.control_dir = np.array(data_dict['dir'])
        if self.data_source == 'simulated':
            self.control_actions = np.array(data_dict['actions'])

    def update_control_command(self, msg):
        data_dict = eval(msg.data)

        tmp = []
        for i in range(self.num_of_bots):
            tmp.append(data_dict[self.key(i)][0])
        
        self.control_actions = np.array(tmp)


    # Estimates the relative positions of bots given the heading directions
    def analyt_model(self, X, best_fit=True):
        # Bot normal direction angles
        X = X.reshape(self.num_of_bots) * np.exp(-1j * np.pi / 2)
        
        ### Precalculate variables that will later be nidded
        # Calculate difference in angle, constrain between - pi to pi
        diff = np.angle(np.roll(X, -1) / X)

        # Calculate direction to next bot:
        theta =  - (np.roll(X, -1) + X)
        theta /= np.abs(theta)

        # Method to calculate distance between bots
        if self.estimator_model == 'rigid_joint':
            L = 2 * self.lmax * np.cos(.5 * diff)
        elif self.estimator_model == 'linear':         # ransac regressor
            L = (155.93 - 0.8547 * 180 / np.pi * np.abs(diff)) / 1000.
        else:
            L = 12**2 / self.num_of_bots/1000

        Ct = np.real(theta)#np.cos(theta)
        St = np.imag(theta)#np.sin(theta)
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

    # Publishes the position of all sub units relative to the odometry link, Slam will then change the position of the odom
    def publish_rel_poisitions(self, positions, angles):
        # generate name
        for i in range(self.num_of_bots):
            idx = self.key(i)

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, np.angle(angles[i]))
            self.odom_broadcaster.sendTransform(
                (positions[i, 0], positions[i, 1], 0.0),
                odom_quat,
                rospy.Time.now(),
                "bot" + idx + "_analyt",
                "bot_center"
            )

    def publish_ground_truth(self):
        # generate name
        angles = self.th_april_data * self.heading_angle_offsets
        positions = self.xy_april_data
        for i in range(self.num_of_bots):
            idx = self.key(i)

            odom_quat = tf.transformations.quaternion_from_euler(0, 0, np.angle(angles[i]))
            self.odom_broadcaster.sendTransform(
                (positions[i, 0], positions[i, 1], 0.0),
                odom_quat,
                rospy.Time.now(),
                "bot" + idx + "_true",
                "map"
            )

    def estimate_odometry(self, positions, angles):
        # Get time from prevous odometry
        current_time = rospy.Time.now()
        dt = (current_time - self.time_odom).to_sec()
        self.time_odom = current_time

        # Calculate desired direction velocity
        des_dir = self.control_dir * .7

        # Convert angles to wheel directions, then calculate the forces produced based on the control actions
        n = np.vstack((np.real(angles * self.heading_angle_offsets),
                       np.imag(angles * self.heading_angle_offsets))).T
        forces = - n.T * self.control_actions * .07#0.011

        # Convert actions into estimated results
        est_translation = np.average(forces, axis=1)

        r = positions - np.average(positions, axis=0)
        est_rotation = np.diagonal(r @ np.array([[0, 1], [-1, 0]]) @ forces)[np.arange(self.num_of_bots)%2 == 0]
        est_rotation = 0*np.average(est_rotation)

        # Calculate odometry position and rotation
        self.xy += est_translation * dt
        self.th += est_rotation *dt

        # Calculate odometry velocity and rotational velocity
        self.vxvy = est_translation
        self.vth = est_rotation


        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
                (self.xy[0], self.xy[1], 0.0),
                odom_quat,
                current_time,
                "bot_center",
                "odom"
            )

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
                (self.xy[0], self.xy[1], 0.0),
                odom_quat,
                current_time,
                "odom2",
                "map"
            )
            

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.xy[0], self.xy[1], 0.), Quaternion(*odom_quat))
        odom.pose.covariance = self.pose_covariance

        # set the velocity
        odom.child_frame_id = "bot_center"
        odom.twist.twist = Twist(
            Vector3(est_translation[0], est_translation[1], 0),
            Vector3(0, 0, est_rotation))
        odom.twist.covariance = self.twist_covariance

        # publish the message
        self.odom_publisher.publish(odom)

    # Calculates next step position of base link
    def update(self):

        # First wait for initial data to appear:
        while np.sum(np.abs(self.control_actions)) < self.num_of_bots:
            self.r.sleep()
        
        sleep(5)

        while not rospy.is_shutdown():
            # Get angles for robot
            if self.data_source == 'experimental':
                angles = self.th_imu_data / self.offsets_exp_imu * self.heading_angle_offsets 
                #angles = self.th_april_data / self.heading_angle_offsets
            else:
                angles = self.th_imu_data * self.heading_angle_offsets
                #angles = self.th_april_data * self.heading_angle_offsets

            # Calculate positions of subunits and set center as middle position
            p_rel = self.analyt_model(angles)
            p_rel -= np.average(p_rel, axis=0)

            # Publish relative positions
            self.publish_rel_poisitions(p_rel, angles)

            self.publish_ground_truth()

            # Now calculate odometry:
            #try:
            self.estimate_odometry(p_rel, angles)
            #except:
            #    rospy.logerr("Could not publish odometry")

            self.r.sleep()


if __name__ == '__main__':
    sleep(1.5)
    rospy.init_node('odometry_publisher')
    try:
        odom_pub = odomBroadcaster(int(sys.argv[1]), sys.argv[2], sys.argv[3])
        #odom_pub = odomBroadcaster(24, "rigid_joint", "simulated")
        #odom_pub = odomBroadcaster(12, "linear", "experimental")
    except:
        odom_pub = odomBroadcaster()
    odom_pub.update()
