"""
Python class used to set
up any applications for the photon
Amin
"""

###################################################
import rosparam
import rospy
from std_msgs.msg import String
from pyparticleio.ParticleCloud import ParticleCloud
import json
from numpy import zeros, sin, cos, ones, vdot, identity, pi, argmin, array, argsort, arange, tan, dot, multiply
from math import atan2, sqrt
from scipy.optimize import LinearConstraint, Bounds, minimize, linprog
from time import sleep, time
import numpy as np
import pandas as pd
import os
import datetime


####################################################

class UtilFunctions:

    def __init__(self, threshold_angle=60., vacuum=False, stop_dist=20., num=12, target_id='12'):
        self.cmd_pub = rospy.Publisher("control_cmd", String, queue_size=10)
        self.locomotion_stat_pub = rospy.Publisher("locomotion_stats", String, queue_size=10)
        parent_dir = '/home/amin/catkin_ws/src/local_sensing_intest/videos/'
        # all_subdirs = self.all_subdirs_of(parent_dir)
        # self.save_path = max(all_subdirs, key=os.path.getmtime)
        self.save_path = parent_dir
        num = int(num)
        self.num = num  # Set the number of robots in your swarm
        self.bot_ids = range(num)  # The ids of the legged robots
        self.vac_state = False
        self.t = zeros(num)
        self.qx = np.zeros(num)
        self.qz = np.zeros(num)
        self.heading = np.zeros(num)
        self.bounds = Bounds(list(ones(num) * 0.1), list(ones(num)))
        self.linear_constraint = LinearConstraint(identity(num), zeros(num), ones(num))
        self.x0 = zeros(num)
        self.last = time()
        self.opt = "simplex"
        self.angle_mode = "mean"
        self.stopped_robot = 0
        self.at_least_one_stopped = False
        self.access_token = {"Amin": "8269d62ca4178e500166245f1835a2e96f4a0de4"}

        # Set the override values
        is_override = zeros(num, dtype=bool)
        override_state = zeros(num)
        self.batt_level = ones(self.num) * -1
        self.t_a = threshold_angle
        self.vacuum = vacuum
        self.stop_dist_default = stop_dist
        self.stop_dist = ones(self.num) * stop_dist
        self.xo = 1e3
        self.yo = 1e3
        self.tar_x = 0
        self.tar_z = 0
        self.leg_closest_i = 0
        self.first_row = np.zeros(6)
        self.back_row = np.zeros(6)
        self.leg_closest_opposite = 0
        self.circle_initialization = False
        self.df = pd.DataFrame(columns=list('tKdxz'))
        self.K = 0
        self.k = self.K
        self.t0 = time()
        self.t_obstacle = time()
        self.just_aligned = True
        self.state = {"00": (0, 0)}
        for i in range(1, self.num):
            self.state[self.key(i)] = (0, 0)
        self.parastalic_dir = 1
        self.shape_err = []
        self.stage0 = True
        self.stage1, self.stage2, self.stage3, self.stage4, self.stage5, self.stage6, self.stage7, self.stage8, self.stage9 = False, False, False, False, False, False, False, False, False
        self.triangle_formed = False
        self.line_formed = False
        self.c_formed = False
        self.circle_formed = False
        self.first_iteration = True
        self.counter = 0
        self.result_local = np.zeros(self.num)
        self.target_id = target_id
        self.range_data = np.ones(self.num)
        # self.des_dir = [0, 1]
        self.des_dir = [-1, 0]
        self.stacked_normal_distances = {"-90": [], "-45": [], "0": [], "45": [], "90": []}
        # self.data_point_stack_size = 30 * 6
        self.prev_mean = 0
        self.direction_change_thrld = 0.2
        self.turn = 1
        self.rot_step = 0
        self.obstacle_avoidance_thrld = 0.15
        self.jammed_aligning = False

    def all_subdirs_of(self, b='.'):
        result = []
        for d in os.listdir(b):
            bd = os.path.join(b, d)
            if os.path.isdir(bd):
                result.append(bd)
        return result

    def state_callback(self, data):
        """ Method to send the desired signals to the photons """
        data_dict = eval(data.data)
        self.tar_x = data_dict[self.target_id][0]
        self.tar_z = data_dict[self.target_id][1]
        alpha = self.get_alpha(data_dict)
        angle = self.get_angle(data_dict)
        dist = self.get_dist(data_dict)
        closest, leg_closest = self.get_min(dist=dist)
        theta = zeros(self.num)
        self.stopped_robot = sum(1 * (self.stop_dist == self.stop_dist_default + 5))
        self.at_least_one_stopped = self.stopped_robot >= 1  # At least one stopped

        if self.at_least_one_stopped:  # If one robot is stopped
            # self.angle_mode = "abs_closest" #use if self.opt = engulfing method is used
            self.angle_mode = "abs_closest"
        else:
            self.angle_mode = "mean"
            
        # Get the angles values of the legged robot and put them in a array for the optimization
        for (i, ids) in enumerate(self.bot_ids):
            if self.angle_mode == "relative":
                theta[ids] = angle[ids] - data_dict[self.key(ids)][2]
            elif self.angle_mode == "closest":
                theta[ids] = angle[closest] - data_dict[self.key(ids)][2]
            elif self.angle_mode == "mean":
                theta[ids] = alpha - data_dict[self.key(ids)][2]
            elif self.angle_mode == "abs_closest":
                theta[ids] = data_dict[self.key(self.get_stopped_closest(robot_id=ids, data_dict=data_dict))][2] - \
                             data_dict[self.key(ids)][2]
            self.t[i] = pi * theta[ids] / 180.
            self.qx[i] = data_dict[self.key(ids)][0]
            self.qz[i] = data_dict[self.key(ids)][1]
            self.heading[i] = data_dict[self.key(ids)][2]
        self.cosTheta = np.cos(self.t)
        self.force = abs(
            self.cosTheta[(np.cos(self.t_a * np.pi / 180) <= self.cosTheta) * (self.cosTheta <= 1)]).sum() + \
                     abs(self.cosTheta[
                             (-1 <= self.cosTheta) * (self.cosTheta <= -np.cos(self.t_a * np.pi / 180))]).sum()
        self.jamming_included = False
        if self.jamming_included and not self.vac_state:
            for i in range(5):
                self.test("vac")
                rospy.sleep(1)
            self.vac_state = True
        ###################################################
        if (time() - self.last) < 5:
            if not rosparam.get_param("engulfed"):
                self.opt = "c-shape"
            else:
                if not self.vac_state:
                    for i in range(2):
                        self.test("stop")
                        rospy.sleep(1)
                    for i in range(5):
                        self.test("vac")
                        rospy.sleep(1)
                    self.vac_state = True
                self.opt = "move_in_jammed" # TODO: changed this line!
            # self.opt = "c-shape"
            print("moving - desired direction:", self.des_dir, "average dist:",
                  np.mean(self.stacked_normal_distances["0"]),
                  "right:", np.mean(self.stacked_normal_distances["90"] + self.stacked_normal_distances["45"]),
                  "left:", np.mean(self.stacked_normal_distances["-90"] + self.stacked_normal_distances["-45"]),
                  "turn dir:", self.turn)
        # elif 15 < (time() - self.last) < 20:
        #     self.opt = "scan"
        #     print("scanning - average dist:", np.mean(self.stacked_normal_distances["0"]))
        else:
            self.last = time()
            if len(self.stacked_normal_distances["0"]) > 0:
                if np.mean(self.stacked_normal_distances["0"]) < self.direction_change_thrld and abs(
                        self.prev_mean - np.mean(self.stacked_normal_distances["0"])) < 0.1:
                    right_distance = np.mean(self.stacked_normal_distances["90"] + self.stacked_normal_distances["45"])
                    left_distance = np.mean(self.stacked_normal_distances["-90"] + self.stacked_normal_distances["-45"])
                    if self.turn == 1:
                        if right_distance < 0.3 and right_distance < left_distance:
                            self.turn = -1
                    else:
                        if left_distance < 0.3 and left_distance < right_distance:
                            self.turn = 1
                    self.des_dir = np.dot(
                        np.array([[np.cos(self.turn * self.rot_step), -np.sin(self.turn * self.rot_step)],
                                  [np.sin(self.turn * self.rot_step), np.cos(self.turn * self.rot_step)]]),
                        np.array(self.des_dir))
                self.prev_mean = np.mean(self.stacked_normal_distances["0"])
                self.stacked_normal_distances = {"-90": [], "-45": [], "0": [], "45": [], "90": []}

        result, speeds = self.get_opt_result(theta=theta, angle=angle, dist=dist, alpha=alpha, leg_closest=leg_closest,
                                             data_dict=data_dict)
        ###################################################
        # print('heading angles:', self.heading)
        # print('x positions:', self.qx)
        # print('y positions:', self.qz)
        # print('result:', result)
        # result = np.zeros(self.num)
        ###################################################
        self.save_data(time() - self.t0, self.force / self.num, np.mean(dist), np.mean(self.qx), np.mean(self.qz))

        for (i, ids) in enumerate(self.bot_ids):
            if result[i] == 1:
                self.state[self.key(ids)] = (1, speeds[i])  # Move forward
            elif result[i] == -1:
                self.state[self.key(ids)] = (-1, speeds[i])  # Move backward
            elif result[i] == 0:
                self.state[self.key(ids)] = (np.random.randint(3, 9), 0)  # Vibrate

        # Send the values
        self.cmd_pub.publish(str(self.state))
        locomotion_stats = {"mode": self.opt, "dir": self.des_dir}
        self.locomotion_stat_pub.publish(str(locomotion_stats))

    def get_data(self):
        date = datetime.datetime.now()
        self.df.to_csv(self.save_path + date.strftime("/%Y-%m-%d %Hh%M") + '.csv', index=False, header=False,
                       columns=['t', 'K', 'd', 'x', 'z'])

    def save_data(self, t, k, d, x, z):
        data = np.array([t, k, d, x, z])
        df2 = pd.DataFrame([data], columns=list('tKdxz'))
        #self.df = self.df.append(df2)
        self.df = pd.concat([self.df, df2])

    def get_alpha(self, data_dict=None):
        """ Method to get the alpha angle of the whole swarm """
        xm = 0.0
        ym = 0.0
        for i in range(self.num):
            xm += data_dict[self.key(i)][0]
            ym += data_dict[self.key(i)][1]

        dx = self.tar_x - xm / self.num
        dy = self.tar_z - ym / self.num

        return 180. * atan2(dy, dx) / pi

    def get_angle(self, data_dict=None):
        """ Method to get the alpha angle of the whole swarm """
        angle = zeros(self.num)
        for i in range(self.num):
            dx = self.tar_x - data_dict[self.key(i)][0]
            dy = self.tar_z - data_dict[self.key(i)][1]
            angle[i] = 180. * atan2(dy, dx) / pi

        return angle

    def get_dist(self, data_dict=None):
        """ Method to get the alpha angle of the whole swarm """
        dist = zeros(self.num)
        for i in range(self.num):
            dx = self.tar_x - data_dict[self.key(i)][0]
            dy = self.tar_z - data_dict[self.key(i)][1]
            dist[i] = sqrt(dx ** 2 + dy ** 2)

        return dist

    def get_min(self, dist=ones(12)):
        """ Method to get the closest legged and vacuum robot """
        leg = zeros(len(self.bot_ids))
        for (i, ids) in enumerate(self.bot_ids):
            leg[i] = dist[ids]
        # print(argmin(dist), self.bot_ids[argmin(leg)])
        return argmin(dist), self.bot_ids[argmin(leg)]

    def get_stopped_closest(self, robot_id=0, data_dict=None):
        """ Method to get the id of the closest stopped robot"""
        closest = 10000000
        closest_id = 0
        for (ids, isStopped) in enumerate(self.stop_dist == self.stop_dist_default + 5):
            if isStopped:
                dx = data_dict[self.key(ids)][0] - data_dict[self.key(robot_id)][0]
                dy = data_dict[self.key(ids)][1] - data_dict[self.key(robot_id)][1]
                dist = sqrt(dx ** 2 + dy ** 2)
                if dist < closest:
                    closest = dist
                    closest_id = ids

        return closest_id

    def get_first_row(self, leg_closest, data_dict, method='closest'):

        if method == 'closest':
            i1 = leg_closest - 3
            i2 = leg_closest + 4
            a = np.multiply(1 * np.multiply((np.arange(i1, i2) >= 0), (np.arange(i1, i2) <= 11)),
                            np.arange(i1, i2)) + (
                        12 * (np.arange(i1, i2) < 0) + np.multiply((np.arange(i1, i2) < 0),
                                                                   np.arange(i1, i2))) + (
                        np.multiply(1 * (np.arange(i1, i2) > 11), np.arange(i1, i2)) - 12 * (
                        np.arange(i1, i2) > 11))
            dev = np.zeros(5)
            dev_sum = np.zeros(2)
            for i in range(0, 2):
                row = a[range(0 + i, 6 + i)]
                for j in range(0, 5):
                    dev[j] = (data_dict[self.key(row[j])][2] -
                              data_dict[self.key(row[j] + 1 * (row[j] != 11) - 11 * (row[j] == 11))][2]) ** 2
                dev_sum[i] = sum(dev) ** 0.5
            idd = np.nonzero(dev_sum == min(dev_sum))[0].astype(int).item()
            first_row = a[np.arange(0 + idd, 6 + idd)]

        elif method == 'heading_angle':
            cond1 = abs(self.cosTheta - np.mean(self.cosTheta[self.cosTheta > 0])) < 1
            first_row = np.arange(0, 12)[cond1]

        return first_row

    def form_circle(self, circle_init, data_dict):
        r = 23
        dx = np.array(self.qx) - np.mean(self.qx)
        dy = np.array(self.qz) - np.mean(self.qz)
        d = np.sqrt(dx ** 2 + dy ** 2)
        if np.sqrt((self.xo - self.tar_x) ** 2 + (
                self.yo - self.tar_z) ** 2) > 10 and circle_init and ((d < r).any()):
            # If the object is moved form a circle shape to initialize the alignment
            result = True
        else:  # Reset the object position
            self.xo = self.tar_x
            self.yo = self.tar_z
            result = False

        return result

    def target_changed(self, data_dict):
        if np.sqrt((self.xo - self.tar_x) ** 2 + (
                self.yo - self.tar_z) ** 2) > 50:
            # If the object is moved form a circle shape to initialize the alignment
            self.xo = self.tar_x
            self.yo = self.tar_z
            result = True
        else:  # Reset the object position
            result = False
        return result

    def get_first_row_middle_id(self, first_row):
        first_row_mean = np.array([np.mean(self.qx[first_row]), np.mean(self.qz[first_row])])
        dist_from_first_row_mean = np.linalg.norm(
            np.array([self.qx[first_row] - first_row_mean[0], self.qz[first_row] - first_row_mean[1]]).T,
            axis=-1)
        id = first_row[(dist_from_first_row_mean == np.min(dist_from_first_row_mean))]

        return id

    def get_opt_result(self, theta, angle, dist, alpha, leg_closest, data_dict):
        """ Method to get the result of the optimization """
        result = zeros(self.num)
        speeds = 255 * np.ones(self.num)
        theta %= 360

        if self.opt == "comparison":
            if self.form_circle(circle_init=self.circle_initialization, data_dict=data_dict):
                result = 1 * np.ones(self.num)
            else:
                # result = 1 * (np.cos(self.t_a * np.pi / 180) <= self.cosTheta) - 1 * (self.cosTheta < -np.cos(self.t_a * np.pi / 180))
                v1 = np.array([np.mean(self.qx), np.mean(self.qz)]) - np.array([self.tar_x, self.tar_z])
                for i in range(self.num):
                    v2 = np.array(
                        [cos(data_dict[self.key(i)][2] * pi / 180), sin(data_dict[self.key(i)][2] * pi / 180)])
                    cos_theta = np.dot(v1, v2)
                    cos_theta = cos_theta / np.linalg.norm(v1) * np.linalg.norm(v2)
                    result[i] = -int((np.cos(self.t_a * np.pi / 180) <= cos_theta)) + int(
                        (cos_theta < -np.cos(self.t_a * np.pi / 180)))

        if self.opt == "alignment_lifters":
            if self.form_circle(circle_init=self.circle_initialization, data_dict=data_dict):
                result = 1 * np.ones(self.num)
            else:
                # result = 1 * (np.cos(self.t_a * np.pi / 180) <= self.cosTheta) - 1 * (self.cosTheta < -np.cos(self.t_a * np.pi / 180))
                v1 = np.array([np.mean(self.qx), np.mean(self.qz)]) - np.array([self.tar_x, self.tar_z])
                for i in range(self.num):
                    v2 = np.array(
                        [cos(data_dict[self.key(i)][2] * pi / 180), sin(data_dict[self.key(i)][2] * pi / 180)])
                    cos_theta = np.dot(v1, v2)
                    cos_theta = cos_theta / np.linalg.norm(v1) * np.linalg.norm(v2)
                    result[i] = -int((np.cos(self.t_a * np.pi / 180) <= cos_theta)) + int(
                        (cos_theta < -np.cos(self.t_a * np.pi / 180)))
                result[result == 0] = 1

        if self.opt == 'stop':
            result = np.zeros(self.num)

        if self.opt == "engulfing":
            self.t_e = 89
            if self.first_iteration:
                v1 = np.array([np.mean(self.qx), np.mean(self.qz)]) - np.array([self.tar_x, self.tar_z])
                for i in range(self.num):
                    v2 = np.array(
                        [cos(data_dict[self.key(i)][2] * pi / 180), sin(data_dict[self.key(i)][2] * pi / 180)])
                    cos_theta = np.dot(v1, v2)
                    cos_theta = cos_theta / np.linalg.norm(v1) * np.linalg.norm(v2)
                    self.result_local[i] = -int((np.cos(self.t_e * np.pi / 180) <= cos_theta)) + int(
                        (cos_theta < -np.cos(self.t_e * np.pi / 180)))
                self.counter = self.counter + 1
                if self.counter > 5:
                    self.first_iteration = False
            if self.target_changed(data_dict):
                self.counter = 0
                self.first_iteration = True
            result = self.result_local

        elif self.opt == "simplex":
            if self.form_circle(circle_init=self.circle_initialization, data_dict=data_dict):
                result = 1 * np.ones(self.num)
            else:
                res = linprog(sin(self.t) ** 2 - cos(self.t) ** 2, method='simplex', bounds=[(0, 1)] * 12)
                for (i, ids) in enumerate(self.bot_ids):
                    if self.at_least_one_stopped:
                        if (90 + self.t_a) < theta[ids] < (270 - self.t_a):
                            result[i] = -1
                        elif 0 <= theta[ids] < (90 - self.t_a) or (270 + self.t_a) < theta[ids] <= 360:
                            result[i] = 1
                        else:
                            result[i] = 0
                    else:
                        result[i] = -res.x[i] * (abs(self.t[i]) > pi / 2) + res.x[i] * (abs(self.t[i]) < pi / 2)

        elif self.opt == 'alignment_distance':
            if self.form_circle(circle_init=self.circle_initialization, data_dict=data_dict):
                result = 1 * np.ones(self.num)
            else:
                self.first_row = self.get_first_row(leg_closest, data_dict, method='closest').astype(int)
                self.back_row = np.arange(0, 12)[np.isin(np.arange(0, 12), self.first_row, invert=True)].astype(int)
                id = self.get_first_row_middle_id(self.first_row)
                m = -1 / np.tan(np.pi * angle[id] / 180)
                yd = self.qz - self.qz[id]
                xd = self.qx - self.qx[id]
                cond1 = (abs(yd - m * xd) >= 0.05)
                v2robots = np.array([self.qx[id] - self.qx, self.qz[id] - self.qz])
                v2target = np.array([self.qx[id] - self.tar_x, self.qz[id] - self.tar_z])
                cond2 = -np.sign(np.dot(v2robots.T, v2target).flatten()) > 0
                result[self.first_row] = 1 * (cond2[self.first_row]) - 1 * (cond2[self.first_row] == 0)
                result[self.back_row] = -1 * (cond1[self.back_row] * cond2[self.back_row]) + 1 * (cond1[self.back_row] *
                                                                                                  cond2[
                                                                                                      self.back_row] == 0)
                result[id] = 2

        elif self.opt == 'peristaltic':
            result = (1 * (np.cos(self.t_a * np.pi / 180) <= self.cosTheta) - 1 * (
                    self.cosTheta <= -np.cos(self.t_a * np.pi / 180))) * (
                             abs(self.cosTheta) >= abs(np.cos(self.t_a * np.pi / 180))) + self.parastalic_dir * (
                             1 * (np.cos(self.t_a * np.pi / 180) > self.cosTheta) * (self.cosTheta > 0) - 1 * (
                             self.cosTheta > -np.cos(self.t_a * np.pi / 180)) * (self.cosTheta < 0)) * (
                             abs(self.cosTheta) < abs(np.cos(self.t_a * np.pi / 180)))


        elif self.opt == 'triangle':
            vertics = np.array([0, 4, 8])

            x1 = self.qx[vertics[0]]
            x2 = self.qx[vertics[1]]
            x3 = self.qx[vertics[2]]
            y1 = self.qz[vertics[0]]
            y2 = self.qz[vertics[1]]
            y3 = self.qz[vertics[2]]

            def area(x1, y1, x2, y2, x3, y3):

                return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                            + x3 * (y1 - y2)) / 2.0)

            # A function to check whether point P(x, y)
            # lies inside the triangle formed by
            # A(x1, y1), B(x2, y2) and C(x3, y3)

            def isInside(x1, y1, x2, y2, x3, y3, x, y):

                # Calculate area of triangle ABC
                A = area(x1, y1, x2, y2, x3, y3)

                # Calculate area of triangle PBC
                A1 = area(x, y, x2, y2, x3, y3)

                # Calculate area of triangle PAC
                A2 = area(x1, y1, x, y, x3, y3)

                # Calculate area of triangle PAB
                A3 = area(x1, y1, x2, y2, x, y)

                # Check if sum of A1, A2 and A3
                # is same as A
                if -1 <= ((A1 + A2 + A3) - A) <= 100:
                    print('err:', ((A1 + A2 + A3) - A))
                    return 0, ((A1 + A2 + A3) - A)
                elif ((A1 + A2 + A3) - A) >= 400:
                    print('err:', ((A1 + A2 + A3) - A))
                    return 1, ((A1 + A2 + A3) - A)
                else:
                    print('err:', ((A1 + A2 + A3) - A))
                    return 2, ((A1 + A2 + A3) - A)

            edges = np.setdiff1d(np.arange(12), vertics)
            result[vertics] = 1
            for i in edges:
                # print('qz', self.qz, 'qx', self.qx, 'i', i)
                print('bot', i)
                cond, er = isInside(x1, y1, x2, y2, x3, y3, self.qx[i], self.qz[i])
                if cond == 0:
                    result[i] = 1
                elif cond == 1:
                    result[i] = -1
                elif cond == 2:
                    result[i] = 2
                if np.sum((result[edges] - 2) ** 2) < 3:
                    result[vertics] = 2
                    self.triangle_formed = True
                # if err < self.shape_err[i] and err > 0.01:
                #     self.shape_err[i] = 10
                #     print('bot', i, ':', err)
                #     result[i] = 0
                # else:
                #     self.shape_err[i] = 5

        elif self.opt == 'line':
            vertics = np.array([0, 6])
            edges1 = np.array([1, 2, 3, 4, 5])
            edges2 = np.array([7, 8, 9, 10, 11])
            x1 = self.qx[vertics[0]]
            x2 = self.qx[vertics[1]]
            y1 = self.qz[vertics[0]]
            y2 = self.qz[vertics[1]]
            m = (y1 - y2) / (x1 - x2)

            result[vertics] = 1
            for i in edges1:
                # print('qz', self.qz, 'qx', self.qx, 'i', i)
                # print('bot', i)
                dist = abs((x2 - x1) * (y1 - self.qz[i]) - (y2 - y1) * (x1 - self.qx[i])) / sqrt(
                    (x2 - x1) ** 2 + (y2 - y1) ** 2)
                print(dist)
                if abs(dist) <= 9:
                    result[i] = 2
                else:
                    result[i] = -1

            for i in edges2:
                # print('qz', self.qz, 'qx', self.qx, 'i', i)
                # print('bot', i)
                dist = abs((x2 - x1) * (y1 - self.qz[i]) - (y2 - y1) * (x1 - self.qx[i])) / sqrt(
                    (x2 - x1) ** 2 + (y2 - y1) ** 2)
                print(dist)
                if abs(dist) <= 9:
                    result[i] = 2
                else:
                    result[i] = -1

            if np.sum(abs(abs(result[edges1]) - 2) + abs(abs(result[edges2]) - 2)) < 4:
                result[np.arange(0, 12)] = 2
                self.line_formed = True

        elif self.opt == 'c-shape':
            self.counter = self.counter + 1
            edges1 = np.array([0, 1, 11])
            edges2 = np.array([2, 10, 9, 7])
            edges3 = np.array([3, 4, 8, 5, 6])

            x1 = self.qx[3]
            x2 = self.qx[9]
            y1 = self.qz[3]
            y2 = self.qz[9]

            result[edges1] = 0
            result[edges2] = -1
            result[edges3] = 1
            result[np.array([0])] = 1
            if self.counter > 35:
                result[np.array([1])] = -1
                result[np.array([11])] = 1

            print('counter:', self.counter, 'dist:', np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2))

            if np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) < 21:
                result[np.arange(0, 12)] = 0
                self.c_formed = True
                self.counter = 0

        elif self.opt == 'circle':

            result = np.ones(self.num)

            if min(np.sqrt((self.qx - np.mean(self.qx)) ** 2 + (self.qz - np.mean(self.qz)) ** 2)) > 28:
                result = np.zeros(self.num)
                self.circle_formed = True

        elif self.opt == 'scan':
            tangential_bots = np.arange(self.num) % 2 != 0  # even bots are normal and odd bots are tangential
            normal_bots = np.arange(self.num) % 2 == 0  # even bots are normal and odd bots are tangential
            result[tangential_bots] = 1
            result[normal_bots] = 0
            theta = (self.heading + np.array([0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90])) * np.pi / 180
            heading_dirs = np.vstack(([np.cos(theta)], [np.sin(theta)])).T
            rel_headings = np.dot(heading_dirs, np.array(self.des_dir))
            if (rel_headings >= cos(np.pi / 6)).any():
                normal_distances = self.range_data * rel_headings
                normal_distances = normal_distances[(normal_distances > 0) * (rel_headings >= cos(np.pi / 6))]
            else:
                normal_distances = []
            self.stacked_normal_distances["0"].extend(np.array(normal_distances).reshape(-1).tolist())
            # overflow = self.stacked_normal_distances["0"].shape[0] - data_point_stack
            # if overflow > 0:
            #     self.stacked_normal_distances["0"] = self.stacked_normal_distances["0"][overflow:]

        elif self.opt == 'move':
            tangential_bots = np.arange(self.num) % 2 != 0
            normal_bots = np.arange(self.num) % 2 == 0
            result[normal_bots] = -1
            for i in range(self.num):
                v2 = np.array(
                    [cos(self.heading[i] * np.pi / 180), sin(self.heading[i] * np.pi / 180)])
                cos_theta = np.dot(np.array(self.des_dir), v2)
                cos_theta = cos_theta / np.linalg.norm(self.des_dir) * np.linalg.norm(v2)
                result[i] = -int((np.cos(self.t_a * np.pi / 180) <= cos_theta)) + int(
                    (cos_theta < -np.cos(self.t_a * np.pi / 180))) - int(
                    abs(cos_theta) < np.cos(self.t_a * np.pi / 180))
            result[tangential_bots] = self.turn
            obstacle_hit_bots = (self.range_data < self.obstacle_avoidance_thrld)
            result[normal_bots * obstacle_hit_bots] = 1

            theta = (self.heading + np.array([0, 90, 0, 90, 0, 90, 0, 90, 0, 90, 0, 90])) * np.pi / 180
            heading_dirs = np.vstack(([np.cos(theta)], [np.sin(theta)])).T
            for angle in np.arange(-90, 135, 45):
                direction = np.dot(
                    np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]),
                    np.array(self.des_dir))
                rel_headings = np.dot(heading_dirs, np.array(direction))
                if (rel_headings >= cos(np.pi / 6)).any():
                    normal_distances = self.range_data * rel_headings
                    normal_distances = normal_distances[(normal_distances > 0) * (rel_headings >= cos(np.pi / 6))]
                else:
                    normal_distances = []
                self.stacked_normal_distances[str(angle)].extend(np.array(normal_distances).reshape(-1).tolist())

        elif self.opt == 'move_in_jammed':
            tangential_bots = np.arange(self.num) % 2 != 0
            normal_bots = np.arange(self.num) % 2 == 0
            result[normal_bots] = -1
            self.t_a = 60
            for i in range(self.num):
                v2 = np.array(
                    [cos(self.heading[i] * np.pi / 180), sin(self.heading[i] * np.pi / 180)])
                cos_theta = np.dot(np.array(self.des_dir), v2)
                cos_theta = cos_theta / np.linalg.norm(self.des_dir) * np.linalg.norm(v2)
                result[i] = -int((np.cos(self.t_a * np.pi / 180) <= cos_theta)) + int(
                    (cos_theta < -np.cos(self.t_a * np.pi / 180)))
            # result[tangential_bots] = self.turn
            # obstacle_hit_bots = (self.range_data < self.obstacle_avoidance_thrld)
            # result[normal_bots * obstacle_hit_bots] = 1

            heading_dirs = np.vstack(([np.cos(self.heading)], [np.sin(self.heading)])).T
            rel_headings = np.dot(heading_dirs, self.des_dir)

            if self.jammed_aligning or np.mean(abs(rel_headings)) < np.cos(np.pi / 2):
                if abs(rel_headings[0]) > np.cos(5 * np.pi / 180):
                    self.jammed_aligning = False
                else:
                    self.jammed_aligning = True
                self.first_row = np.array([0, 1, 2, 11, 10])
                self.back_row = np.array([8, 7, 6, 5, 4])
                id = 0
                m = -1 / np.tan(np.pi * angle[id] / 180)
                yd = self.qz - self.qz[id]
                xd = self.qx - self.qx[id]
                cond1 = (abs(yd - m * xd) >= 0.05)
                v2robots = np.array([self.qx[id] - self.qx, self.qz[id] - self.qz])
                v2target = self.des_dir
                cond2 = -np.sign(np.dot(v2robots.T, v2target).flatten()) > 0
                result[self.first_row] = 1 * (cond2[self.first_row]) - 1 * (cond2[self.first_row] == 0)
                result[self.back_row] = -1 * (cond1[self.back_row] * cond2[self.back_row]) + 1 * (cond1[self.back_row] *
                                                                                                  cond2[
                                                                                                      self.back_row] == 0)
                result[id] = -1
            
            tmp = 90 * (np.arange(len(self.heading))%2)
            theta = (self.heading + tmp) * np.pi / 180
            heading_dirs = np.vstack(([np.cos(theta)], [np.sin(theta)])).T

            for angle in np.arange(-90, 135, 45):
                direction = np.dot(
                    np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]),
                    np.array(self.des_dir))
                rel_headings = np.dot(heading_dirs, np.array(direction))
                if (rel_headings >= cos(np.pi / 6)).any():
                    normal_distances = self.range_data * rel_headings
                    normal_distances = normal_distances[(normal_distances > 0) * (rel_headings >= cos(np.pi / 6))]
                else:
                    normal_distances = []
                self.stacked_normal_distances[str(angle)].extend(np.array(normal_distances).reshape(-1).tolist())

        return result, speeds

    def update_range_sensors(self, msg):
        bot_id = msg.header.frame_id
        bot_number = int(bot_id[3:])
        self.range_data[bot_number] = msg.range

    def test(self, test="vac"):
        """ Method to test different part of the vacuum robot """
        msg = {"00": (0, 0)}

        if test == "vac":
            for i in self.bot_ids:
                msg[self.key(i)] = (2, 100)

        elif test == "leg+":
            for i in self.bot_ids:
                msg[self.key(i)] = (-1, 255)

        elif test == "leg-":
            for i in self.bot_ids:
                msg[self.key(i)] = (1, 255)

        elif test == "stop":
            for i in range(self.num):
                msg[self.key(i)] = (np.random.randint(3, 9), 0)

        # elif test == "sleep":
        #     for i in self.bot_ids:
        #         msg[str(i)] = 7
        self.cmd_pub.publish(str(msg))

    def key(self, i):
        if i < 10:
            key = str(0) + str(i)
        else:
            key = str(i)
        return key

    # def start_ros_node(self):
    #     """ Method used to set the ROS node and start the looping """
    #
    #     rospy.Subscriber("state", String, self.state_callback)
    #     rospy.spin()
