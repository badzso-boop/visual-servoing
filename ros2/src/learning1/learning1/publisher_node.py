import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')          # node neve
        self.publisher_ = self.create_publisher(
            String,        # üzenet típus
            'sajat_topic', # topic neve
            10             # queue méret
        )
        # Timer: minden 1 másodpercben hív egy függvényt
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Szia ROS2! Szam: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Kuldtem: {msg.data}')
        self.counter += 1

def main():
    rclpy.init()
    node = PublisherNode()
    rclpy.spin(node)   # "pörög" amíg Ctrl+C-t nem nyomsz
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()