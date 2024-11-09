import rospy
from geometry_msgs.msg import PoseStamped

class PublishManager(object):
    def __init__(self, body_part):
        self.body_part = body_part
        self.pub = rospy.Publisher("hrc/poses/{}".format(body_part), PoseStamped, queue_size=1)
        self.pose_msg = None

    def setPoseMsg(self, _pose_msg):
        self.pose_msg = _pose_msg

    def publishPoseMsg(self):
        if self.pose_msg is None:
            return

        self.pub.publish(self.pose_msg)
