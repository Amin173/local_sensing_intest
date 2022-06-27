#!/usr/bin/env python

"""
Python script used to use the april tags
tracking system from : https://github.com/duckietown/apriltags3-py
"""

####################################################
import sys
import rospy

sys.path.append('./JameobaApriltags/')
sys.path.append('./apriltags3py/')
from JameobaApriltags import AprilTagTracker
import datetime

####################################################


def main(num_of_tags):
    # Say if you want to lot the data after the test
    plot_data = False
    animate_data = True
    date = datetime.datetime.now()
    # Initialise the tracker
    tracker = AprilTagTracker(num=int(num_of_tags), save_data=True, save_video=True, show_mean=False, show_robots=False, trail=150,
                              show_arrow_robots=True, crop_image=False, cam_choice="brio", res_choice="1080p",
                              focus_frames=100, pub_freq=5)

    # Start the loop
    tracker.start_tracking()

    # Get the filename
    file = tracker.filename
    data_saved = tracker.save_data

    # Delete the tracker
    del tracker

if __name__ == '__main__':
    try:
        main(sys.argv[1])
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
        pass
