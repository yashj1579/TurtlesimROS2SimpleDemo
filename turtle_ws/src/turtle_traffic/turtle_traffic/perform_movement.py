import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class Movement(Node):

    def __init__(self):
        super().__init__('movement')


        #create parameters
        self.declare_parameter('constant_v', 1.0)
        self.v = self.get_parameter('constant_v').value
        self.declare_parameter('max_w', 7.0)
        self.declare_parameter('max_alpha', 1.0)
        self.max_w = self.get_parameter('max_w').value
        self.max_alpha = self.get_parameter('max_alpha').value
        self.w = 0.0
        self.alpha = 0.0
        self.angle = 0.0

        #subscribe to get message
        self.pose_subscriber = self.create_subscription(
        String,                  # Message type
        'car_heading',       # Topic name
        self.pose_callback,    # Callback function
        10                     # queue size
        )


        #publish to command velocity
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer = 0.1
        self.dt = timer
        self.timer = self.create_timer(timer, self.movement)

    def pose_callback(self, msg):
        self.angle = msg.data
        #self.get_logger().info(f"Received pose message: {msg.data}")
        try:
            self.angle = float(msg.data.split(" ")[1])
        except Exception as e:
            self.get_logger().error(f"Failed to parse angle: {e}")
            self.angle = None
    def calculate_w(self):
        desired_theta = self.angle
        if desired_theta is None:
            return None
        if abs(desired_theta) < 1e-2:
            return 0.0  # close enough

        #alpha = self.max_alpha * desired_theta / math.pi
        k = 4.0  # Try tuning this gain value
        alpha = k * math.tanh(desired_theta)
        w = alpha

        # Cap to max_w
        if abs(w) > self.max_w:
            w = self.max_w * (-1 if w < 0 else 1)

        return w


    def movement(self):
        twist = Twist()
        res = self.calculate_w()

        if res is None:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().error(f'Node call failed')
            return

        heading_error = abs(self.angle)
        twist.linear.x = self.v * (1 - min(1, heading_error / math.pi))
        #twist.linear.x = self.v * math.cos(heading_error)
        #twist.linear.x = self.v
        twist.angular.z = res

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    mover = Movement()

    rclpy.spin(mover)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
                                
