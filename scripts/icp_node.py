#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
# Visualization libraries ###################
from std_msgs.msg import ColorRGBA          #
from visualization_msgs.msg import Marker   #
from geometry_msgs.msg import Point         #
##############################################
import sensor_msgs.point_cloud2 as pc2


from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d
import tf
import tf2_ros

import time


class rgbd_odom(object):
    def __init__(self, points_topic, frame_id):


        self.past_points = None
        self.new_points = None
        self.odom = np.identity(4)
        self.trans_init = self.odom
        self.voxel_size = 0.06

        self.threshold = 0.02


        #self.intrinsic_params = np.array([[554.254691191187, 0.0, 320.5],[0.0, 554.254691191187, 240.5], [0.0, 0.0, 1.0]])


        # ROS methods
        self.points_sub = rospy.Subscriber(points_topic, PointCloud2, self.point_storage)

        self.pub_timer = rospy.Timer(rospy.Duration(0.1), self.retrieve_transform)

    def retrieve_transform(self, event):
        
        if self.past_points is not None and self.new_points is not None:
            start_time = time.time()

            reg_p2p = o3d.pipelines.registration.registration_icp(self.new_points, self.past_points, self.threshold, self.trans_init, o3d.pipelines.registration.TransformationEstimationPointToPoint())


            self.odom = self.odom @ reg_p2p.transformation

            print(np.round(self.odom,2))



    def point_storage(self, msg):

        print(msg.data)


        # if self.new_points is None:
        #     self.new_points = convertCloudFromRosToOpen3d(msg)
        #     #self.new_points = self.new_points.voxel_down_sample(voxel_size = 0.06)

        # else:
        #     self.past_points = self.new_points
        #     self.new_points = convertCloudFromRosToOpen3d(msg)
        #     #self.new_points = self.new_points.voxel_down_sample(voxel_size = 0.06)

            


if __name__ == "__main__":

    print("sometimes there might be wrong times to rock!")

    rospy.init_node('rgbd_odometry')   

    node = rgbd_odom('/converted_pc', 'odom')
    
    # Run forever
    rospy.spin()