#!/usr/bin/env python3
# Copyright 2026 Paulo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Forward only selected transforms from a replayed bag to /tf.

A bag recorded during a SLAM session contains, in /tf, transforms from three
different sources:

  * odom → base_footprint  (Gazebo diff drive)        → wanted on replay
  * wheel transforms       (robot_state_publisher)    → republished live
  * map → odom             (the old SLAM run)         → would fight the new one

slam_replay.launch.py plays the bag with /tf remapped to /tf_bag. This node
subscribes to /tf_bag and republishes on /tf only the transforms whose child
frame is listed in the `child_frames` parameter (default: base_footprint).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class BagTfFilter(Node):

    def __init__(self):
        super().__init__('bag_tf_filter')
        self.declare_parameter('child_frames', ['base_footprint'])
        self.child_frames = set(
            self.get_parameter('child_frames').value)

        # Same QoS as the standard /tf topic
        qos = QoSProfile(depth=100,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(TFMessage, '/tf', qos)
        self.sub = self.create_subscription(
            TFMessage, '/tf_bag', self.on_tf, qos)

        self.get_logger().info(
            f'Forwarding /tf_bag → /tf for child frames: '
            f'{sorted(self.child_frames)}')

    def on_tf(self, msg: TFMessage):
        kept = [t for t in msg.transforms
                if t.child_frame_id in self.child_frames]
        if kept:
            self.pub.publish(TFMessage(transforms=kept))


def main():
    rclpy.init()
    node = BagTfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
