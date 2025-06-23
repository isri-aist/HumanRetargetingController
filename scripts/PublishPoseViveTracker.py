#!/usr/bin/env python3

import sys
import json
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from tf_transformations import euler_matrix, translation_matrix, translation_from_matrix, quaternion_from_matrix
import openvr
from PublishManager import PublishManager  # Make sure this is ROS 2 compatible!

class PublishPoseViveTracker(Node):
    def __init__(self):
        super().__init__('publish_pose_vive_tracker')
        self.vr_system = openvr.init(openvr.VRApplication_Other)

        self.root_mat = euler_matrix(0.5 * np.pi, 0.0, 0.0)
        self.offset_mat_map = {
            "waist": euler_matrix(-0.5 * np.pi, 0.0, 0.5 * np.pi, "rxyz"),
            "left_elbow": translation_matrix([0.0, 0.0, 0.04]),
            "left_wrist": translation_matrix([0.0, 0.0, 0.05]),
            "right_elbow": translation_matrix([0.0, 0.0, 0.04]),
            "right_wrist": translation_matrix([0.0, 0.0, 0.05]),
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

        self.get_logger().info("Map from device SN to body part:\n{}".format(json.dumps(self.device_sn_to_body_part_map, indent=4)))

        self.pose_pub_managers = {}
        self.joy_pub_managers = {}

        for body_part in self.device_sn_to_body_part_map.values():
            self.pose_pub_managers[body_part] = PublishManager(self, body_part)

    def __del__(self):
        openvr.shutdown()

    def run(self):
        rate = self.create_rate(30)
        print_info = True

        while rclpy.ok():
            self.current_stamp = self.get_clock().now().to_msg()
            self.device_data_list = self.vr_system.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
            )

            if print_info:
                self.get_logger().info("Device info:")
            for device_idx in range(openvr.k_unMaxTrackedDeviceCount):
                self.process_single_device_data(device_idx, print_info)

            if print_info:
                print_info = False

            rate.sleep()

    def process_single_device_data(self, device_idx, print_info=False):
        if not self.device_data_list[device_idx].bDeviceIsConnected:
            return

        device_type = self.vr_system.getTrackedDeviceClass(device_idx)
        device_sn = self.vr_system.getStringTrackedDeviceProperty(device_idx, openvr.Prop_SerialNumber_String)
        is_pose_valid = self.device_data_list[device_idx].bPoseIsValid
        pose_matrix_data = self.device_data_list[device_idx].mDeviceToAbsoluteTracking

        if print_info:
            type_map = {
                openvr.TrackedDeviceClass_GenericTracker: "Tracker",
                openvr.TrackedDeviceClass_Controller: "Controller",
                openvr.TrackedDeviceClass_TrackingReference: "BaseStation",
                openvr.TrackedDeviceClass_HMD: "HMD"
            }
            device_type_str = type_map.get(device_type, "Unknown")
            self.get_logger().info(f"  - {device_type_str}: {device_sn}")

        if device_sn not in self.device_sn_to_body_part_map or not is_pose_valid:
            return

        body_part = self.device_sn_to_body_part_map[device_sn]

        pose_mat = np.zeros((4, 4))
        pose_mat[0:3, 0:4] = pose_matrix_data.m
        pose_mat[3, 3] = 1.0
        pose_mat = self.root_mat @ pose_mat
        if body_part in self.offset_mat_map:
            pose_mat = pose_mat @ self.offset_mat_map[body_part]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.current_stamp
        pose_msg.header.frame_id = "robot_map"
        pos = translation_from_matrix(pose_mat)
        quat = quaternion_from_matrix(pose_mat)

        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = pos
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = quat

        self.pose_pub_managers[body_part].setMsg(pose_msg)
        self.pose_pub_managers[body_part].publishMsg()

        if device_type == openvr.TrackedDeviceClass_Controller:
            if body_part not in self.joy_pub_managers:
                self.joy_pub_managers[body_part] = PublishManager(self, body_part, msg_type=Joy, topic_prefix="hrc/joys")

            device_state = self.vr_system.getControllerState(device_idx)[1]
            joy_msg = Joy()
            joy_msg.header = pose_msg.header
            joy_msg.axes = [device_state.rAxis[0].x, device_state.rAxis[0].y, device_state.rAxis[1].x]
            joy_msg.buttons = [
                bool(device_state.ulButtonPressed >> openvr.k_EButton_ApplicationMenu & 1),
                bool(device_state.ulButtonPressed >> openvr.k_EButton_Grip & 1),
                bool(device_state.ulButtonPressed >> openvr.k_EButton_SteamVR_Touchpad & 1),
                bool(device_state.ulButtonTouched >> openvr.k_EButton_SteamVR_Touchpad & 1),
            ]

            self.joy_pub_managers[body_part].setMsg(joy_msg)
            self.joy_pub_managers[body_part].publishMsg()

def main(args=None):
    rclpy.init(args=args)
    node = PublishPoseViveTracker()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
