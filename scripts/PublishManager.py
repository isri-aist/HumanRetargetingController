from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

class PublishManager:
    def __init__(self, node: Node, body_part: str, msg_type=PoseStamped, topic_prefix="hrc/poses"):
        self.node = node
        self.body_part = body_part
        topic_name = f"{topic_prefix}/{body_part}"
        self.pub = self.node.create_publisher(msg_type, topic_name, 10)
        self.msg = None

    def setMsg(self, _msg):
        self.msg = _msg

    def publishMsg(self):
        if self.msg is not None:
            self.pub.publish(self.msg)
