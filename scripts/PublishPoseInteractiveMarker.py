#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from tf_transformations import quaternion_from_euler

from PublishManager import PublishManager  # You must adapt this to ROS 2 too.

class PublishPoseInteractiveMarker(Node):
    def __init__(self):
        super().__init__('publish_pose_interactive_marker')

        self.body_part_init_pos_map = {
            "waist": [2.0, 0.0, 0.86],
            "left_elbow": [2.2, 0.4, 1.0],
            "left_wrist": [2.6, 0.4, 0.95],
            "right_elbow": [2.2, -0.4, 1.0],
            "right_wrist": [2.6, -0.4, 0.95],
        }

        self.im_server = InteractiveMarkerServer(self, "im_server")
        self.pub_managers = {}

        for body_part, init_pos in self.body_part_init_pos_map.items():
            self.add_interactive_marker(body_part, init_pos)
            self.pub_managers[body_part] = PublishManager(self, body_part)

        self.im_server.applyChanges()

    def add_interactive_marker(self, body_part, init_pos):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "robot_map"
        int_marker.name = f"hrc/{body_part}"
        int_marker.pose.position.x, int_marker.pose.position.y, int_marker.pose.position.z = init_pos
        int_marker.pose.orientation.w = 1.0
        int_marker.scale = 0.2

        self.add_control(int_marker, "rotate_x", InteractiveMarkerControl.ROTATE_AXIS, 0, 0, 0)
        self.add_control(int_marker, "rotate_y", InteractiveMarkerControl.ROTATE_AXIS, 0, 0, np.pi / 2)
        self.add_control(int_marker, "rotate_z", InteractiveMarkerControl.ROTATE_AXIS, 0, np.pi / 2, 0)

        self.add_control(int_marker, "move_x", InteractiveMarkerControl.MOVE_AXIS, 0, 0, 0)
        self.add_control(int_marker, "move_y", InteractiveMarkerControl.MOVE_AXIS, 0, 0, np.pi / 2)
        self.add_control(int_marker, "move_z", InteractiveMarkerControl.MOVE_AXIS, 0, np.pi / 2, 0)

        self.im_server.insert(int_marker, self.interactive_marker_feedback)

    def add_control(self, marker, name, interaction_mode, roll, pitch, yaw):
        control = InteractiveMarkerControl()
        control.name = name
        control.interaction_mode = interaction_mode
        control.orientation_mode = InteractiveMarkerControl.FIXED
        quat = quaternion_from_euler(roll, pitch, yaw)
        control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w = quat
        marker.controls.append(control)

    def interactive_marker_feedback(self, feedback):
        pose_msg = PoseStamped()
        pose_msg.header = feedback.header
        pose_msg.pose = feedback.pose

        body_part = feedback.marker_name[len("hrc/"):]
        self.pub_managers[body_part].setMsg(pose_msg)

    def run(self):
        rate = self.create_rate(30)
        while rclpy.ok():
            for pub in self.pub_managers.values():
                pub.publishMsg()
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = PublishPoseInteractiveMarker()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
