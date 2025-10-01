import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, String


class Sub(Node):

    def __init__(self):
        super().__init__('sub')

        # Store the latest number from topic_generator/msg
        self.latest_number = None

        # Subscribe to both topics
        self.number_subscription = self.create_subscription(
            UInt8,
            'topic_generator/msg',
            self.number_callback,
            10)

        self.info_subscription = self.create_subscription(
            String,
            'data_reciever/msg',
            self.info_callback,
            10)

        self.get_logger().info('Topic listener started, waiting for messages...')

    def number_callback(self, msg):
        self.latest_number = msg.data

    def info_callback(self, msg):
        student_info = msg.data
        if self.latest_number is not None:
            output = f"{student_info} {self.latest_number}"
            self.get_logger().info(f'Output: {output}')
        else:
            self.get_logger().warn('No number received yet, cannot output combined message')


def main(args=None):
    rclpy.init(args=args)

    sub = Sub()

    try:
        rclpy.spin(sub)
    except KeyboardInterrupt:
        pass

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
