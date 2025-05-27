from turtle_interfaces.srv import GeneratePOI
import rclpy
from rclpy.node import Node
import random
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, TeleportAbsolute
from rclpy.task import Future
from turtlesim.srv import SetPen


class RandPOI(Node):

    def __init__(self):
        super().__init__('POI_generator')
        self.srv = self.create_service(GeneratePOI, 'generate_random_points', self.gen_random_points)
        self.existing_turtles = set()

    def gen_random_points(self, request, response):
        num = request.num_points
        response.poses = []
        self.get_logger().info(f'Publishing {num} random POIs.')

        for i in range(num):
            pose = Pose()
            pose.x = random.uniform(1,10)
            pose.y = random.uniform(1,10)
            pose.theta = 0.0
            pose.linear_velocity = 0.0
            pose.angular_velocity = 0.0

            self.get_logger().info(f'{i} point: {pose.x}, {pose.y}')
            response.poses.append(pose)


            name = f'point{i}'
            try:
                #self.disable_pen(name)
                self.move_turtle(name, pose.x, pose.y)
                self.get_logger().info(f'Moved turtle {name}')
            except Exception as e:
                self.get_logger().info(f"Spawned turtle {name}")
                self.spawn_turtle(name, pose.x, pose.y)
                #self.existing_turtles.add(name)

        return response

    def spawn_turtle(self, name, x, y):
        cli = self.create_client(Spawn, '/spawn')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available...')
        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = 0.0
        req.name = name
        future = cli.call_async(req)

    def move_turtle(self, name, x, y):
        cli = self.create_client(TeleportAbsolute, f'/{name}/teleport_absolute')
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Teleport service for {name} not available...')
            raise RuntimeError()
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = 0.0
        future = cli.call_async(req)

def main(args=None):
    rclpy.init()
    service = RandPOI()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
