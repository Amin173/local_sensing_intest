#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Pose2D, PoseStamped
import tf_conversions
import tf2_ros
from tf.transformations import euler_from_quaternion


class InitialPose:
    def __init__(self):
        self.isSet = False
        self.x0, self.y0, self.th0 = [0, 0, 0]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

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
    # rospy.Subscriber("state", String, initial_pose.update)
    rate = rospy.Rate(10)
    while not initial_pose.isSet:
        rospy.logwarn(f"Initial pose set: {initial_pose.isSet}")
        try:
            trans = initial_pose.tfBuffer.lookup_transform('bot00', 'map', rospy.Time(0))
            rospy.set_param('initial_pose_x', trans.transform.translation.x)
            rospy.set_param('initial_pose_y', trans.transform.translation.y)
            e = euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, \
                trans.transform.rotation.z, trans.transform.rotation.w])
            rospy.set_param('initial_pose_a', e[2])
            initial_pose.isSet = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        rate.sleep()
    rospy.logwarn(f"Initial pose set: {initial_pose.isSet}")