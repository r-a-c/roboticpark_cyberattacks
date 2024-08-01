import rclpy
from rclpy.node import Node

class dosnode(Node):
    def __init__(self):
        super().__init__("dosnode")


def main():
    rclpy.init()
    mydosnode = dosnode()
    rclpy.spin(mydosnode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
