#!/usr/bin/env python2.7
# license removed for brevity

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Pose2D, PoseStamped
import tf_conversions


class InitialPose:
    def __init__(self):
        self.isSet = False
        self.x0, self.y0, self.th0 = [0, 0, 0]

    def update(self, data):
        data_dict = eval(data.data)
        self.x0 = (data_dict['00'][0] - data_dict['12'][0]) / 100
        self.y0 = -(data_dict['00'][1] - data_dict['12'][1]) / 100
        self.th0 = -(data_dict['00'][2] - data_dict['12'][2]) * 3.14 / 180
        if not (self.x0 == 0 or self.y0 == 0 or self.th0 == 0):
            rospy.set_param('initial_pose_x', self.x0)
            rospy.set_param('initial_pose_y', self.y0)
            rospy.set_param('initial_pose_a', self.th0)
            self.isSet = True


if __name__ == '__main__':
    rospy.init_node('dynamic_tf2_broadcaster')
    initial_pose = InitialPose()
    rospy.Subscriber("state", String, initial_pose.update)
    rate = rospy.Rate(10)
    while not initial_pose.isSet:
        print(initial_pose.isSet)
        rate.sleep()
