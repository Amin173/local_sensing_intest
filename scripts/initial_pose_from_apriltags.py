#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from ruamel.yaml import YAML
from pathlib import Path
import tf2_ros
<<<<<<< HEAD

=======
>>>>>>> ea3eafef6618ae75a87dee5f40d5f6e1d8598c89

class InitialPose:
    def __init__(self):
        self.isSet = False
        self.x0, self.y0, self.th0 = [0, 0, 0]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.yaml = YAML()
        self.path = Path('/opt/ros/overlay_ws/src/local_sensing_intest/config/config.yml')

    def update(self, data):
        data_dict = eval(data.data)
        self.x0 = (data_dict['00'][0] - data_dict['12'][0]) / 100
        self.y0 = -(data_dict['00'][1] - data_dict['12'][1]) / 100
        self.th0 = -(data_dict['00'][2] - data_dict['12'][2]) * 3.14 / 180
        if not (self.x0 == 0 or self.y0 == 0 or self.th0 == 0):
            rospy.set_param('/config/initial_pose/x', self.x0)
            rospy.set_param('/config/initial_pose/y', self.y0)
            rospy.set_param('/config/initial_pose/a', self.th0)
            self.isSet = True


if __name__ == '__main__':
    rospy.init_node('dynamic_tf2_broadcaster')
    initial_pose = InitialPose()
    rospy.Subscriber("state", String, initial_pose.update)
    rate = rospy.Rate(0.1)
    while not initial_pose.isSet:
        rospy.logwarn(f"Waiting for rosbag data...")
        rate.sleep()
    initial_pose.yaml.dump(rospy.get_param("/config"), initial_pose.path)
    rospy.logwarn(f"Initial pose set: {initial_pose.isSet}")
    rospy.logwarn("Configuration complete!")

