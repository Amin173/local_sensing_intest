#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, TransformStamped, Pose2D, Transform, PoseStamped, PoseWithCovarianceStamped
import open3d as o3d
import numpy as np
import copy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
import tf.transformations as tr
import tf_conversions
# from open3d.open3d.geometry import voxel_down_sample, estimate_normals, statistical_outlier_removal, \
#     radius_outlier_removal, select_down_sample
import matplotlib.pyplot as plt

import time

PI = 3.14159265


class PointCloudPoseEst:

    def __init__(self):
        self.source_points = np.array([])
        self.target_points = np.array([])
        self.trans_init = np.array([])
        self.pose = PoseWithCovarianceStamped()
        self.counter = 0
        self.target_is_set = False
        self.trans_init_set = False

    def msg_to_se3(self, msg):
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w])
        norm = np.linalg.norm(q)
        if np.abs(norm - 1.0) > 1e-3:
            try:
                rospy.logwarn(ValueError(
                    "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                        str(q), np.linalg.norm(q))))
                q = q / norm
            except:
                raise ValueError(
                    "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                        str(q), np.linalg.norm(q)))
        elif np.abs(norm - 1.0) > 1e-6:
            q = q / norm
        g = tr.quaternion_matrix(q)
        g[0:3, -1] = p
        return g

    def get_source(self, cloud):
        self.source_points = np.array([])
        for p in pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z")):
            self.source_points = np.append(self.source_points, [p])
        self.source_points = self.source_points.reshape((-1, 3))

    def set_target(self, msg):
        if not self.target_is_set:
            self.target_points = np.array([])
            for p in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
                self.target_points = np.append(self.target_points, p)
            self.target_points = self.target_points.reshape((-1, 3))
            # print(self.target_points[0:10, :])
            self.target_is_set = True

    def get_init_trans(self, msg):
        if not self.trans_init_set:
            self.trans_init = self.msg_to_se3(msg.pose)
        else:
            pass

    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        # target_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp])

    def preprocess_point_cloud(self, pcd, voxel_size):
        print(":: Downsample with a voxel size %.3f." % voxel_size)
        pcd_down = pcd.voxel_down_sample(voxel_size)

        radius_normal = voxel_size * 2
        print(":: Estimate normal with search radius %.3f." % radius_normal)
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

        radius_feature = voxel_size * 5
        print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        pcd_fpfh = o3d.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh

    def display_inlier_outlier(self, cloud, ind):
        inlier_cloud = cloud.select_down_sample(ind)
        outlier_cloud = cloud.select_down_sample(ind, invert=True)
        print("Showing outliers (red) and inliers (gray): ")
        o3d.geometry.PointCloud.paint_uniform_color(outlier_cloud, [1, 0, 0])
        o3d.geometry.PointCloud.paint_uniform_color(inlier_cloud, [0.8, 0.8, 0.8])
        o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    def prepare_dataset(self, voxel_size, source, target):
        print(":: Load two point clouds and disturb initial pose.")

        trans_init = np.asarray([[1.0, 0.0, 0.0, 0.1], [0.0, 1.0, 0.0, 0.1],
                                 [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        # target_down.transform(trans_init)
        source_down = source.voxel_down_sample(voxel_size=0.02)
        # cl, ind = statistical_outlier_removal(source_down, nb_neighbors=20, std_ratio=2.0)
        # cl, ind = source_down.remove_radius_outlier(nb_points=5, radius=0.05)

        # plane_model, ind = source_down.segment_plane(distance_threshold=0.05, ransac_n=3, num_iterations=1000)
        # cl, ind = source.remove_statistical_outlier(nb_neighbors=10,
        #                                                     std_ratio=2.0)
        cl, ind = source_down.remove_radius_outlier(nb_points=5, radius=0.05)

        # labels = np.array(
        #     source_down.cluster_dbscan(eps=0.02, min_points=3, print_progress=True))
        # max_label = labels.max()
        # # print(f"point cloud has {max_label + 1} clusters")
        # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        # colors[labels < 0] = 0
        # source_down.colors = o3d.utility.Vector3dVector(colors[:, :3])
        # o3d.visualization.draw_geometries([source_down])

        # keypoints.paint_uniform_color([1.0, 0.75, 0.0])
        # o3d.visualization.draw_geometries([keypoints, mesh], front=[0, 0, -1.0])
        self.display_inlier_outlier(source_down, ind)

        # self.draw_registration_result(source_down, target_down, trans_init)

        return source, target

    def execute_fast_global_registration(self, source_down, target_down, source_fpfh,
                                         target_fpfh, voxel_size):
        distance_threshold = voxel_size * 100
        print(":: Apply fast global registration with distance threshold %.3f" \
              % distance_threshold)
        result = o3d.registration.registration_fast_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            o3d.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=distance_threshold))
        return result

    def localize(self, msg):
        self.get_source(msg)
        # if self.target_points.size > 0 and self.source_points.size > 0 and self.trans_init.size > 0:
        try:
            # Pass xyz to Open3D.o3d.geometry.PointCloud and visualize
            source = o3d.geometry.PointCloud()
            source.points = o3d.utility.Vector3dVector(self.source_points)

            target = o3d.geometry.PointCloud()
            target.points = o3d.utility.Vector3dVector(self.target_points)

            voxel_size = 0.05  # means 5cm for the dataset
            source, target = self.prepare_dataset(voxel_size, source, target)
            # result_fast = self.execute_fast_global_registration(source, target,
            #                                                source_fpfh, target_fpfh,
            #                                                voxel_size)
            # reg_p2p = result_fast

            threshold = 0.1
            start_time = time.time()

            reg_p2p = o3d.registration.registration_icp(
                source, target, threshold, self.trans_init,
                o3d.registration.TransformationEstimationPointToPoint(),
                o3d.registration.ICPConvergenceCriteria(max_iteration=30000))

            print("--- %s seconds ---" % (time.time() - start_time))

            # self.draw_registration_result(source, target, reg_p2p)
            self.pose.header.frame_id = 'map'
            self.pose.header.stamp = rospy.Time.now()
            self.pose.pose.pose.position.x = reg_p2p.transformation[0, 3]
            self.pose.pose.pose.position.y = reg_p2p.transformation[1, 3]
            self.pose.pose.pose.position.z = 0
            pose_covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0000, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0000, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0000, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

            self.pose.pose.covariance = pose_covariance
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, np.arctan2(reg_p2p.transformation[1, 0],
                                                                                      reg_p2p.transformation[0, 0]))
            self.pose.pose.pose.orientation.x = q[0]
            self.pose.pose.pose.orientation.y = q[1]
            self.pose.pose.pose.orientation.z = q[2]
            self.pose.pose.pose.orientation.w = q[3]
            self.trans_init = self.msg_to_se3(self.pose.pose)
            # self.trans_init_set = True

        except:

            rospy.logwarn("Point cloud pose estimation not available")
            rospy.logwarn("map cloud size: %s, source cloud size: %s initial transform size: %s" % (
                self.target_points.size, self.source_points.size, self.trans_init.size))
            # rospy.logwarn(msg)
            raise


if __name__ == "__main__":
    rospy.init_node('pcl_register', anonymous=True)
    pcpe = PointCloudPoseEst()
    rospy.Subscriber("cloud_in", PointCloud2, pcpe.set_target)
    rospy.Subscriber("cloud_in", PointCloud2, pcpe.localize)
    rospy.Subscriber("odom/vel_model", Odometry, pcpe.get_init_trans)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
