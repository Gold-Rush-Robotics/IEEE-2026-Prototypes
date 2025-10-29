import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'isaac_joint_states',  # Topic name to subscribe to
            self.joint_state_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("Subscribed to isaac_joint_states topic")

    def joint_state_callback(self, msg):
        self.get_logger().info(f"Received JointState message: {msg}")
        # Process the JointState message as needed

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    rclpy.spin(joint_state_subscriber)
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    