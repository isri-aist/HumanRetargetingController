import rospy
from geometry_msgs.msg import PoseStamped

class PublishManager(object):
    def __init__(self, body_part, msg_type=PoseStamped, topic_prefix="hrc/poses"):
        self.body_part = body_part
        self.pub = rospy.Publisher("{}/{}".format(topic_prefix, body_part), msg_type, queue_size=1)
        self.msg = None

    def setMsg(self, _msg):
        self.msg = _msg

    def publishMsg(self):
        if self.msg is None:
            return

        self.pub.publish(self.msg)
