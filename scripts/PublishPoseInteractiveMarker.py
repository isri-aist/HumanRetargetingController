#! /usr/bin/env python

import numpy as np
import rospy
from tf import transformations
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from PublishManager import PublishManager

class PublishPoseInteractiveMarker(object):
    def __init__(self):
        body_part_init_pos_map = {
            "waist": [2.0, 0.0, 0.86],
            "left_elbow": [2.2, 0.4, 1.0],
            "left_wrist": [2.6, 0.4, 0.95],
            "right_elbow": [2.2, -0.4, 1.0],
            "right_wrist": [2.6, -0.4, 0.95],
        }

        self.im_server = InteractiveMarkerServer("im_server")

        self.pub_managers = {}

        for body_part in body_part_init_pos_map.keys():
            self.addInteractiveMarker(body_part, body_part_init_pos_map[body_part])
            self.pub_managers[body_part] = PublishManager(body_part)

        self.im_server.applyChanges()

    def addInteractiveMarker(self, body_part, init_pos=None):
        if init_pos is None:
            init_pos = [0.0, 0.0, 0.0]

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "robot_map"
        int_marker.name = "hrc/{}".format(body_part)
        int_marker.pose.position.x = init_pos[0]
        int_marker.pose.position.y = init_pos[1]
        int_marker.pose.position.z = init_pos[2]
        int_marker.pose.orientation.x = 0.0
        int_marker.pose.orientation.y = 0.0
        int_marker.pose.orientation.z = 0.0
        int_marker.pose.orientation.w = 1.0
        int_marker.scale = 0.2

        # rotate_x control
        control = InteractiveMarkerControl()
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w = transformations.quaternion_from_euler(0, 0, 0)
        int_marker.controls.append(control)

        # rotate_y control
        control = InteractiveMarkerControl()
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w = transformations.quaternion_from_euler(0, 0, np.pi/2)
        int_marker.controls.append(control)

        # rotate_z control
        control = InteractiveMarkerControl()
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w = transformations.quaternion_from_euler(0, np.pi/2, 0)
        int_marker.controls.append(control)

        # move_x control
        control = InteractiveMarkerControl()
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w = transformations.quaternion_from_euler(0, 0, 0)
        int_marker.controls.append(control)

        # move_y control
        control = InteractiveMarkerControl()
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w = transformations.quaternion_from_euler(0, 0, np.pi/2)
        int_marker.controls.append(control)

        # move_z control
        control = InteractiveMarkerControl()
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        ori = control.orientation
        ori.x, ori.y, ori.z, ori.w = transformations.quaternion_from_euler(0, np.pi/2, 0)
        int_marker.controls.append(control)

        self.im_server.insert(int_marker, self.interactivemarkerFeedback)

    def interactivemarkerFeedback(self, feedback):
        pose_msg = PoseStamped()
        pose_msg.header = feedback.header
        pose_msg.pose = feedback.pose

        body_part = feedback.marker_name[len("hrc/"):]
        self.pub_managers[body_part].setMsg(pose_msg)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            for body_part in self.pub_managers.keys():
                self.pub_managers[body_part].publishMsg()

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("publish_pose_interactive_marker", anonymous=True)
    publish_pose = PublishPoseInteractiveMarker()
    publish_pose.run()
