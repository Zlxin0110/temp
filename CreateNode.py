创建一个ROS 2工作区（如果尚未创建）：
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create my_test_pkg

进入包目录并创建节点文件
cd ~/ros2_ws/src/my_test_pkg
touch my_test_node.py
chmod +x my_test_node.py

编辑 my_test_node.py 文件并添加以下内容：
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MyTestNode(Node):
    def __init__(self):
        super().__init__('my_test_node')
        self.publisher_a = self.create_publisher(String, 'topic_a', 10)
        self.publisher_b = self.create_publisher(String, 'topic_b', 10)
        self.timer_a = self.create_timer(1.0, self.publish_a_callback)
        self.timer_b = self.create_timer(2.0, self.publish_b_callback)

    def publish_a_callback(self):
        msg = String()
        msg.data = 'Hello from Topic A!'
        self.publisher_a.publish(msg)

    def publish_b_callback(self):
        msg = String()
        msg.data = 'Hello from Topic B!'
        self.publisher_b.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    my_test_node = MyTestNode()
    rclpy.spin(my_test_node)
    my_test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


在终端中运行：
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run my_test_pkg my_test_node.py
