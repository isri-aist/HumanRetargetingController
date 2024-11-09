#! /usr/bin/env python

import numpy as np
import rospy
from tf import transformations
from geometry_msgs.msg import PoseStamped
import openvr

from PublishManager import PublishManager

class PublishPoseViveTracker(object):
    def __init__(self):
        self.vr_system = openvr.init(openvr.VRApplication_Other)

        self.root_mat = transformations.euler_matrix(0.5 * np.pi, 0.0, 0.0)
        self.waist_offset_mat = transformations.euler_matrix(-0.5 * np.pi, 0.0, 0.5 * np.pi, "rxyz")

        self.device_sn_to_body_part_map = {
            "LHR-8C30BD01": "waist",
            "LHR-1FB29FC6": "left_elbow",
            "LHR-301CBF17": "left_hand",
        }

        self.pub_managers = {}
        for body_part in self.device_sn_to_body_part_map.values():
            self.pub_managers[body_part] = PublishManager(body_part)

    def __del__(self):
        openvr.shutdown()

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.current_stamp = rospy.Time.now()

            self.device_data_list = self.vr_system.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding,
                0,
                openvr.k_unMaxTrackedDeviceCount)

            for device_idx in range(openvr.k_unMaxTrackedDeviceCount):
                self.processSingleDeviceData(device_idx)

            rate.sleep()

    def processSingleDeviceData(self, device_idx):
        if not self.device_data_list[device_idx].bDeviceIsConnected:
            return

        device_type = self.vr_system.getTrackedDeviceClass(device_idx)
        device_sn = self.vr_system.getStringTrackedDeviceProperty(device_idx, openvr.Prop_SerialNumber_String)
        is_device_pose_valid = self.device_data_list[device_idx].bPoseIsValid
        device_pose_matrix = self.device_data_list[device_idx].mDeviceToAbsoluteTracking

        if device_sn not in self.device_sn_to_body_part_map:
            return

        if not is_device_pose_valid:
            return

        body_part = self.device_sn_to_body_part_map[device_sn]

        pose_mat = np.zeros((4, 4))
        pose_mat[0:3, 0:4] = device_pose_matrix.m
        pose_mat[-1, -1] = 1.0
        pose_mat = self.root_mat @ pose_mat
        if body_part == "waist":
            pose_mat = pose_mat @ self.waist_offset_mat

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.current_stamp
        pose_msg.header.frame_id = "robot_map"
        pos = transformations.translation_from_matrix(pose_mat)
        pose_msg.pose.position.x = pos[0]
        pose_msg.pose.position.y = pos[1]
        pose_msg.pose.position.z = pos[2]
        quat = transformations.quaternion_from_matrix(pose_mat)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.pub_managers[body_part].setPoseMsg(pose_msg)
        self.pub_managers[body_part].publishPoseMsg()

if __name__ == "__main__":
    rospy.init_node("publish_pose_vive_tracker", anonymous=True)
    publish_pose = PublishPoseViveTracker()
    publish_pose.run()
