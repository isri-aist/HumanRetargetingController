#! /usr/bin/env python

import sys
import numpy as np
import json
import rospy
from tf import transformations
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
import openvr

from PublishManager import PublishManager

class PublishPoseViveTracker(object):
    def __init__(self):
        self.vr_system = openvr.init(openvr.VRApplication_Other)

        self.root_mat = transformations.euler_matrix(0.5 * np.pi, 0.0, 0.0)
        self.offset_mat_map = {
            "waist": transformations.euler_matrix(-0.5 * np.pi, 0.0, 0.5 * np.pi, "rxyz"),
            "left_elbow": transformations.translation_matrix([0.0, 0.0, 0.04]),
            "left_wrist": transformations.translation_matrix([0.0, 0.0, 0.05]),
            "right_elbow": transformations.translation_matrix([0.0, 0.0, 0.04]),
            "right_wrist": transformations.translation_matrix([0.0, 0.0, 0.05]),
        }

        self.device_sn_to_body_part_map = {
            "LHR-8C30BD01": "waist",
            "LHR-1FB29FC6": "left_elbow",
            "LHR-301CBF17": "left_wrist",
        }
        if len(sys.argv) >= 2:
            import yaml
            from pathlib import Path
            self.device_sn_to_body_part_map = yaml.safe_load(Path(sys.argv[1]).read_text())

        rospy.loginfo("Map from device SN to body part:\n{}".format(json.dumps(self.device_sn_to_body_part_map, indent=4)))

        self.pose_pub_managers = {}
        self.joy_pub_managers = {}
        for body_part in self.device_sn_to_body_part_map.values():
            self.pose_pub_managers[body_part] = PublishManager(body_part)

    def __del__(self):
        openvr.shutdown()

    def run(self):
        rate = rospy.Rate(30)
        print_info = True

        while not rospy.is_shutdown():
            self.current_stamp = rospy.Time.now()

            self.device_data_list = self.vr_system.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding,
                0,
                openvr.k_unMaxTrackedDeviceCount)

            if print_info:
                rospy.loginfo("Device info:")
            for device_idx in range(openvr.k_unMaxTrackedDeviceCount):
                self.processSingleDeviceData(device_idx, print_info)

            if print_info:
                print_info = False

            rate.sleep()

    def processSingleDeviceData(self, device_idx, print_info=False):
        if not self.device_data_list[device_idx].bDeviceIsConnected:
            return

        device_type = self.vr_system.getTrackedDeviceClass(device_idx)
        device_sn = self.vr_system.getStringTrackedDeviceProperty(device_idx, openvr.Prop_SerialNumber_String)
        is_device_pose_valid = self.device_data_list[device_idx].bPoseIsValid
        device_pose_matrix = self.device_data_list[device_idx].mDeviceToAbsoluteTracking

        if print_info:
            if device_type == openvr.TrackedDeviceClass_GenericTracker:
                device_type_str = "Tracker"
            elif device_type == openvr.TrackedDeviceClass_Controller:
                device_type_str = "Controller"
            elif device_type == openvr.TrackedDeviceClass_TrackingReference:
                device_type_str = "BaseStation"
            elif device_type == openvr.TrackedDeviceClass_HMD:
                device_type_str = "HMD"
            rospy.loginfo("  - {}: {}".format(device_type_str, device_sn))

        if device_sn not in self.device_sn_to_body_part_map:
            return

        if not is_device_pose_valid:
            return

        body_part = self.device_sn_to_body_part_map[device_sn]

        pose_mat = np.zeros((4, 4))
        pose_mat[0:3, 0:4] = device_pose_matrix.m
        pose_mat[-1, -1] = 1.0
        pose_mat = self.root_mat @ pose_mat
        if body_part in self.offset_mat_map:
            pose_mat = pose_mat @ self.offset_mat_map[body_part]

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

        self.pose_pub_managers[body_part].setMsg(pose_msg)
        self.pose_pub_managers[body_part].publishMsg()

        if device_type == openvr.TrackedDeviceClass_Controller:
            if body_part not in self.joy_pub_managers:
                self.joy_pub_managers[body_part] = PublishManager(body_part, msg_type=Joy, topic_prefix="hrc/joys")

            device_state = self.vr_system.getControllerState(device_idx)[1]
            trackpad_x = device_state.rAxis[0].x # -1.0 to 1.0
            trackpad_y = device_state.rAxis[0].y # -1.0 to 1.0
            trigger = device_state.rAxis[1].x # 0.0 to 1.0, where 0.0 is fully released
            menu_button = bool(device_state.ulButtonPressed >> openvr.k_EButton_ApplicationMenu & 1)
            grip_button = bool(device_state.ulButtonPressed >> openvr.k_EButton_Grip & 1)
            trackpad_pressed = bool(device_state.ulButtonPressed >> openvr.k_EButton_SteamVR_Touchpad & 1)
            trackpad_touched = bool(device_state.ulButtonTouched >> openvr.k_EButton_SteamVR_Touchpad & 1)

            joy_msg = Joy()
            joy_msg.header = pose_msg.header
            joy_msg.axes = [trackpad_x, trackpad_y, trigger]
            joy_msg.buttons = [menu_button, grip_button, trackpad_pressed, trackpad_touched]

            self.joy_pub_managers[body_part].setMsg(joy_msg)
            self.joy_pub_managers[body_part].publishMsg()

if __name__ == "__main__":
    rospy.init_node("publish_pose_vive_tracker", anonymous=True)
    publish_pose = PublishPoseViveTracker()
    publish_pose.run()
