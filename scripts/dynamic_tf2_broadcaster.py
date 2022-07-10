#!/usr/bin/env python
import rospy
import tf_conversions
from tf.transformations import *
import sys
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class Broadcaster:
    def __init__(self, bot_id, frame_id, origin_tag_id):
        x0, y0, th0 = [0, 0, 0]
        while x0 == 0 or y0 == 0 or th0 == 0:
            x0 = float(rospy.get_param('initial_pose_x'))
            y0 = float(rospy.get_param('initial_pose_y'))
            th0 = float(rospy.get_param('initial_pose_a'))
        self.base_link_pose_x = x0
        self.base_link_pose_y = y0
        self.base_link_orientation = th0
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.t.header.frame_id = frame_id
        self.t.child_frame_id = bot_id
        self.bot_number = bot_id[3:]
        self.t.header.stamp = rospy.Time.now()
        self.t.transform.translation.x = self.base_link_pose_x
        self.t.transform.translation.y = self.base_link_pose_y
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.base_link_orientation)
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]
        self.imu_pub = rospy.Publisher("imu_pose_publisher_" + str(bot_id), PoseStamped, queue_size=2)
        self.rel_rotation_set = False
        self.qr = [0, 0, 0, 1]
        self.cam_trnsfrm_recieved = False
        self.origin_tag = origin_tag_id

    def handle_pose(self, msg):
        self.t.header.stamp = rospy.Time.now()
        self.t.transform.translation.x = msg.pose.position.x + self.base_link_pose_x
        self.t.transform.translation.y = msg.pose.position.y + self.base_link_pose_y
        q = tf_conversions.transformations.quaternion_from_euler(0, 0,
                                                                 msg.pose.orientation.z + self.base_link_orientation)
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]
        self.br.sendTransform(self.t)

    def base_link_pose(self, msg):
        data_dict = eval(msg.data)
        self.base_link_pose_x = (data_dict['00'][0] - data_dict[self.origin_tag][0]) / 100
        self.base_link_pose_y = -(data_dict['00'][1] - data_dict[self.origin_tag][1]) / 100
        self.base_link_orientation = -(data_dict['00'][2] - data_dict[self.origin_tag][2]) * 3.14 / 180

    def base_link_odom(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.base_link_pose_x = msg.pose.pose.position.x / 100
        self.base_link_pose_y = msg.pose.pose.position.y / 100
        self.base_link_orientation = yaw

    def imu_pose_publisher(self, msg):
        pose = PoseStamped()
        pose.header.frame_id = self.t.header.frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.t.transform.translation.x
        pose.pose.position.y = self.t.transform.translation.y
        pose.pose.position.z = 0
        if (not self.rel_rotation_set) and self.cam_trnsfrm_recieved:
            q1_inv = [msg.orientation.x, msg.orientation.y, msg.orientation.z, - msg.orientation.w]
            q2 = [self.t.transform.rotation.x, self.t.transform.rotation.y, self.t.transform.rotation.z,
                  self.t.transform.rotation.w]
            self.qr = tf.transformations.quaternion_multiply(q2, q1_inv)
            self.rel_rotation_set = True
        modified_quat = tf.transformations.quaternion_multiply(self.qr, [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        pose.pose.orientation.x = modified_quat[0]
        pose.pose.orientation.y = modified_quat[1]
        pose.pose.orientation.z = modified_quat[2]
        pose.pose.orientation.w = modified_quat[3]
        # print(self.t.child_frame_id, str(msg.orientation.x - self.t.transform.rotation.x) + " " + str(
        #     pose.pose.orientation.y - self.t.transform.rotation.y) + " " +
        #       str(pose.pose.orientation.z - self.t.transform.rotation.z) + " " + str(
        #     pose.pose.orientation.w - self.t.transform.rotation.w))
        self.imu_pub.publish(pose)

    def update(self, data):
        data_dict = eval(data.data)
        self.base_link_pose_x = (data_dict['00'][0] - data_dict[self.origin_tag][0]) / 100
        self.base_link_pose_y = -(data_dict['00'][1] - data_dict[self.origin_tag][1]) / 100
        self.base_link_orientation = -(data_dict['00'][2] - data_dict[self.origin_tag][2]) * 3.14 / 180
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.t.child_frame_id
        pose_msg.pose.position.x = (data_dict[self.bot_number][0] - data_dict['00'][0]) / 100
        pose_msg.pose.position.y = -(data_dict[self.bot_number][1] - data_dict['00'][1]) / 100
        if (int(self.bot_number) % 2) == 0:
            theta = -(data_dict[self.bot_number][2] - data_dict['00'][2]) * 3.14 / 180
        else:
            theta = -(data_dict[self.bot_number][2] - data_dict['00'][2] + 90) * 3.14 / 180
        pose_msg.pose.orientation.z = theta
        self.handle_pose(pose_msg)


def update_all(msg):
    for i in child_frames:
        broadcasters[str(i)].update(msg)
        broadcasters[str(i)].cam_trnsfrm_recieved = True


def bot_id(i):
    if i < 10:
        id1 = "bot" + str(0) + str(i)
    else:
        id1 = "bot" + str(i)
    return id1


if __name__ == '__main__':
    rospy.init_node('dynamic_tf2_broadcaster', anonymous=True)
    fixed_frame_id = sys.argv[1]
    fixed_frame_origin_tag = sys.argv[2]
    child_frames = rospy.get_param('~child_frames')
    broadcasters = {}
    for i in child_frames:
        broadcasters[str(i)] = Broadcaster(bot_id(i), fixed_frame_id, fixed_frame_origin_tag)
        rospy.Subscriber('%s/imu/data' % bot_id(i),
                         Imu,
                         broadcasters[str(i)].imu_pose_publisher)
    rospy.Subscriber("state", String, update_all)
    # rospy.Subscriber("odom/vel_model",
    #                  Odometry,
    #                  broadcaster.base_link_odom)
    rospy.spin()
