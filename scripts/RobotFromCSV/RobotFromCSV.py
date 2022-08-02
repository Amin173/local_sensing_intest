'''
Python script used to
test the april tags
tracking system
from : https://github.com/duckietown/apriltags3-py
'''

####################################################
import os
import csv
import math
import cv2
import rospy
import datetime
import numpy as np
import sys

sys.path.append("/home/amin/catkin_ws/src/local_sensing_intest/scripts/apriltags3py")
import apriltags3 as at
from std_msgs.msg import String
from time import time
import screeninfo
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


####################################################
####################################################
class AprilTagTracker:
    """ Class for the April tag tracker """

    def __init__(self, num=8, cam_choice="brio", res_choice="1080p", tag_choice="new", save_data=False,
                 save_video=False, show_mean=False, show_robots=False, show_arrow_robots=False, crop_image=False,
                 plot=False, pub_freq=5, trail=20, testname=None, focus_frames=0):
        """ Initialisation of the class """
        # Set some variables
        self.number_of_robots = num  # The number of tags to track
        self.cam_choice = cam_choice  # Choice of the camera
        self.res_choice = res_choice  # Choice of the resolution
        self.tag_choice = tag_choice  # Tag choice
        self.save_data = save_data  # Save the data?
        self.save_video = save_video  # Save the video?
        self.show_mean = show_mean  # Show the mean trace?
        self.show_robots = show_robots  # Show the robot trace?
        self.show_arrow_robots = show_arrow_robots  # Show the robot arrows?
        self.crop_image = crop_image  # Crop the image?
        self.plot = plot  # Plot the data?
        self.publishFrequency = pub_freq  # [Hz]
        self.trail_length = trail  # Length of the trace
        self.focus_frames = focus_frames  # Number of frame not taken at the begining to let the camera focus
        self.camera_index = 2

        
        # Change the directory name if it already exist
        k = 1
        if os.path.exists(self.data_dir):
            self.data_dir = self.data_dir[:-1] + "_0/"

        while os.path.exists(self.data_dir):
            self.data_dir = self.data_dir.replace("_" + str(k - 1) + "/", "_" + str(k) + "/")
            k += 1

        # Set the different filenames
        self.filename = self.data_dir + "data.csv"
        self.video_name = self.data_dir + "video.avi"
        self.video_no_overlay_name = self.video_name.replace(".avi", "_no_overlay.avi")

        # If we save the video or the data, create the directory
        if self.save_data:
            os.mkdir(self.data_dir[:-1])

        # Set some values
        self.origin = (0, 0)
        self.frame_index = 0
        self.state = {'00': (0, 0, 0)}  # The array are (x [cm], y [cm], theta [deg])
        for i in range(self.number_of_robots):
            if i < 10:
                key = str(0) + str(i)
            else:
                key = str(i)
            self.state[key] = (0, 0, 0)

        self.pt = np.ones((self.trail_length, 2, self.number_of_robots))
        self.ptmean = np.ones((self.trail_length, 2))
        self.arrow_vec = np.ones((2, self.number_of_robots))

        # Setup the tracker
        self._tracker_setup()

        # Setup ROS
        self.pub = rospy.Publisher('state', String, queue_size=10)
        rospy.init_node('AprilTags', anonymous=False)

        self.last = time()

    def _tracker_setup(self):
        """ Method to setup the tracker """

        # Initialize the apriltags detector
        self.detector = at.Detector(
            searchpath=['/home/amin/catkin_ws/src/local_sensing_intest/scripts/apriltags3py/apriltags'],
            families="tag36h11", nthreads=2,
            quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)

        # Initialize the csv writer if you decided to save the data
        if self.save_data:
            self.writer = csv.writer(open(self.filename, 'w'), quotechar='|', quoting=csv.QUOTE_MINIMAL)
            header_row = [''] * self.number_of_robots * 3
            for i in range(self.number_of_robots):
                key = str(i)
                header_row[3 * i] = 'X' + key
                header_row[3 * i + 1] = 'Y' + key
                header_row[3 * i + 2] = 'Theta' + key
            self.writer.writerow(header_row)

    def __del__(self):
        pass

    @staticmethod
    def _is_rotation_matrix(R):
        """ Method to check if the matrix is a rotation matrix """
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def _rotation_matrix_to_euler_angles(self, R):
        assert (self._is_rotation_matrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def start_tracking(self):
        """ Method to start the tracker loop """

        running = True  # While this is true, the loop will run

        while running:
            # read data from csv file
            
            # To do: asd
            # Detect tags , set a list of detection items
            # detections = 

            if self.frame_index == 0:  # First iteration of the loop
                ox = oy = 0
                for i, detection in enumerate(detections):
                    self.pt[:, 0, detection.tag_id] *= detection.center[0] + self.dx
                    self.pt[:, 1, detection.tag_id] *= detection.center[1] + self.dy
                    ox += 100 * detection.pose_t[0][0]
                    oy += 100 * detection.pose_t[1][0]

                self.origin = (ox / self.number_of_robots, oy / self.number_of_robots)

                self.ptmean[:, 0] *= sum(self.pt[0, 0, :])
                self.ptmean[:, 1] *= sum(self.pt[0, 1, :])

            else:
                for i, detection in enumerate(detections):
                    if 0 <= detection.tag_id < self.number_of_robots:  # Make sure the detection is within the tags we want
                        if detection.tag_id < 10:
                            key = str(0) + str(detection.tag_id)
                        else:
                            key = str(detection.tag_id)
                        # key = str(detection.tag_id)
                        self.state[key] = (100 * detection.pose_t[0][0] - self.origin[0],
                                           100 * detection.pose_t[1][0] - self.origin[1],
                                           180 * self._rotation_matrix_to_euler_angles(detection.pose_R)[2] / np.pi)
                        self.pt[self.trail_length - 1, 0, detection.tag_id] = detection.center[0] + self.dx
                        self.pt[self.trail_length - 1, 1, detection.tag_id] = detection.center[1] + self.dy
                        vecx = self.pt[self.trail_length - 1, 0, detection.tag_id] - self.pt[
                            self.trail_length - 2, 0, detection.tag_id]
                        vecy = self.pt[self.trail_length - 1, 1, detection.tag_id] - self.pt[
                            self.trail_length - 2, 1, detection.tag_id]
                        veclen = np.sqrt(vecx ** 2 + vecy ** 2)
                        if -1 < vecx < 1 and -1 < vecy < 1:
                            vecx = vecy = 0
                            veclen = 1
                        self.arrow_vec[0, detection.tag_id] = 20.0 * float(vecx) / veclen
                        self.arrow_vec[1, detection.tag_id] = 20.0 * float(vecy) / veclen

                self.ptmean[self.trail_length - 1, 0] = sum(self.pt[self.trail_length - 1, 0, :]) / float(
                    self.number_of_robots)
                self.ptmean[self.trail_length - 1, 1] = sum(self.pt[self.trail_length - 1, 1, :]) / float(
                    self.number_of_robots)

            freq = 1.0 / (time() - self.last)  # Cap the publish frequency to let the Photons Controller read the data
            if freq <= self.publishFrequency:
                self.last = time()
                self.pub.publish(str(self.state))
                if self.save_data:
                    data = [0] * self.number_of_robots * 3
                    for i in range(self.number_of_robots):
                        if i < 10:
                            key = str(0) + str(i)
                        else:
                            key = str(i)
                        data[3 * i] = self.state[key][0]
                        data[3 * i + 1] = self.state[key][1]
                        data[3 * i + 2] = self.state[key][2]
                    self.writer.writerow(data)

            self.ptmean = np.roll(self.ptmean, -1, axis=0)
            self.ptmean[-1] = self.ptmean[-2]
            for i in range(self.number_of_robots):
                self.pt[:, :, i] = np.roll(self.pt[:, :, i], -1, axis=0)
                self.pt[-1, :, i] = self.pt[-2, :, i]

            if k == 27:
                running = False








####################################################
class DataPlotter:
    """ A class to plot the data of a previous test """

    def __init__(self, filename, freq=5., standard_notation=True):
        """ Initialisation of the class """
        self.freq = freq
        self.dt = 1. / self.freq

        # Getting the data from the file
        self.data = np.genfromtxt(filename, delimiter=',')
        self.num_robot = int(len(self.data[0, :]) / 3)
        self.data_length = len(self.data[:, 0])
        self.data = self.data[3:self.data_length, :]
        self.data_length = len(self.data[:, 0])
        self.t = np.arange(self.data_length) * self.dt
        self.speed = 0
        self.meanx = np.zeros(self.data_length)
        self.meany = np.zeros(self.data_length)
        if standard_notation:
            self.save_dir = filename.replace("data.csv", "anim.mp4")
        else:
            self.save_dir = filename.replace(".csv", ".mp4")

    def _compute_mean(self, style="vib18"):
        """ Method to compute the mean of the robots over time """
        if style == "vib18":
            num = self.num_robot - 1
        elif style == "string":
            num = self.num_robot
        else:
            num = self.num_robot
        for i in range(num):
            self.meanx += self.data[:, 3 * i] / self.num_robot
            self.meany += self.data[:, 3 * i + 1] / self.num_robot

    def _animate_vib18(self, speed=1., save=False, show=True):
        """ Method to animate the data for the 18 vibration robot"""
        self._compute_mean(style="vib18")
        self.speed = speed
        fig = plt.figure()
        ax = plt.axes(xlim=(-100, 100), ylim=(-100, 100))
        ax.set_aspect("equal")
        ax.set_xlabel("x position (cm)")
        ax.set_ylabel("y position (cm)")
        self.line, = ax.plot([], [], c="pink", lw=8)
        self.point, = ax.plot([], [], c="red", lw=0, marker="+")
        self.center, = ax.plot([], [], c="blue", lw=2)
        ax.legend((self.center, self.point), ('Center of Robot', 'Target'), loc='upper right', shadow=True)
        self.time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
        anim = FuncAnimation(fig, self._frame_vib18, init_func=self._init_vib18, frames=len(self.t),
                             interval=self.dt * 1000. / self.speed, blit=True)
        if save:
            anim.save(self.save_dir, writer='ffmpeg')
        if show:
            plt.show()

    def _init_vib18(self):
        self.line.set_data([], [])
        self.point.set_data([], [])
        self.center.set_data([], [])
        self.time_text.set_text('')
        return self.line, self.point, self.center, self.time_text

    def _frame_vib18(self, i):
        x = np.zeros(self.num_robot)
        y = np.zeros(self.num_robot)
        for j in range(self.num_robot - 1):
            x[j] = self.data[i, 3 * j]
            y[j] = -self.data[i, 3 * j + 1]
        x[-1] = x[0]
        y[-1] = y[0]
        self.line.set_data(x, y)
        x = self.meanx[0:i]
        y = -self.meany[0:i]
        self.center.set_data(x, y)
        x = self.data[i, 3 * (self.num_robot - 1)]
        y = -self.data[i, 3 * (self.num_robot - 1) + 1]
        self.point.set_data(x, y)
        self.time_text.set_text('Time = %.1fs (%.0fX)' % (self.t[i], self.speed))
        return self.line, self.point, self.center, self.time_text

    def _animate_string(self, speed=1., save=False, show=True):
        """ Method to animate the data for the 18 vibration robot"""
        self._compute_mean(style="string")
        self.speed = speed
        fig = plt.figure()
        ax = plt.axes(xlim=(-100, 100), ylim=(-100, 100))
        ax.set_aspect("equal")
        ax.set_xlabel("x position (cm)")
        ax.set_ylabel("y position (cm)")
        self.line, = ax.plot([], [], c="red", lw=0, marker='o')
        self.center, = ax.plot([], [], c="blue", lw=2)
        ax.legend((self.line, self.center), ('Robot node', 'Center of Robot'), loc='upper right', shadow=True)
        self.time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
        anim = FuncAnimation(fig, self._frame_string, init_func=self._init_string, frames=len(self.t),
                             interval=self.dt * 1000. / self.speed, blit=True)
        if save:
            anim.save(self.save_dir, writer='ffmpeg')
        if show:
            plt.show()

    def _init_string(self):
        self.line.set_data([], [])
        self.center.set_data([], [])
        self.time_text.set_text('')
        return self.line, self.center, self.time_text

    def _frame_string(self, i):
        x = np.zeros(self.num_robot)
        y = np.zeros(self.num_robot)
        for j in range(self.num_robot):
            x[j] = self.data[i, 3 * j]
            y[j] = -self.data[i, 3 * j + 1]
        self.line.set_data(x, y)
        x = self.meanx[0:i]
        y = -self.meany[0:i]
        self.center.set_data(x, y)
        self.time_text.set_text('Time = %.1fs (%.0fX)' % (self.t[i], self.speed))
        return self.line, self.center, self.time_text

    def animate(self, style="vib18", speed=1., save=False, show=True):
        """ Method to animate the data """
        if style == "vib18":
            self._animate_vib18(speed=speed, save=save, show=show)
        elif style == "string":
            self._animate_string(speed=speed, save=save, show=show)

    def plot(self):
        """ Method to plot the data """
        for i in range(self.num_robot):
            plt.figure()
            plt.title("Robot #" + str(i))
            plt.plot(self.t, self.data[:, 3 * i], label='X')
            plt.plot(self.t, self.data[:, 3 * i + 1], label='Y')
            plt.plot(self.t, self.data[:, 3 * i + 2], label='Angle')

            plt.legend()

        plt.figure()
        plt.title("Robots in 2D")
        for i in range(self.num_robot):
            plt.plot(self.data[:, 3 * i], self.data[:, 3 * i + 1], label="Robot #" + str(i))

            plt.legend()

        plt.show()
####################################################
