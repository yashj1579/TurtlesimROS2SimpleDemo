import rclpy
from rclpy.node import Node
from turtle_interfaces.srv import GeneratePOI
from std_msgs.msg import String
import math
from turtlesim.msg import Pose
from turtlesim.srv import Kill


class Anglificator(Node):

    def __init__(self):
        super().__init__('anglificator')

        #create paramters
        self.declare_parameter('num_points', 10)
        self.num = self.get_parameter('num_points').value
        self.POI = []
        self.visited = []

        self.car_info = None
        self.future = None

        self.requesting_poi = False

        #subscribe to get position of the car
        self.pose_subscriber = self.create_subscription(
        Pose,                  # Message type
        '/turtle1/pose',       # Topic name
        self.pose_callback,    # Callback function
        10                     # queue size
        )


        #get the generate random points service
        self.cli = self.create_client(GeneratePOI, 'generate_random_points')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')


        #get kill command to remove points

        self.kill_cli = self.create_client(Kill, '/kill')
        while not self.kill_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /kill service...')

        #call the directionificate method TiDo to publish the angle the car needs to be moving at
        self.publisher_ = self.create_publisher(String, 'car_heading', 10) #name of publish is car_heading
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def dist(self, pos1, pos2):
        """
        pos1, pos2: Pose
        returns: distance between both points
        """
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)
    def closest_point(self, pos_car, pos_POI):
        """
        returns: the index associated with the point closest 
        """
        pos = -1
        for i in range(len(pos_POI)):
            if pos == -1 and not self.visited[i]:
                pos = i
                continue
            if not self.visited[i] and self.dist(pos_car, pos_POI[pos]) > self.dist(pos_car, pos_POI[i]):
                pos = i
        return pos

    def pose_callback(self, msg):
        self.car_info = msg

    def kill_turtle(self, name):
        req = Kill.Request()
        req.name = name
        future = self.kill_cli.call_async(req)

    def allTrue(self, lis):
        for i in lis:
            if i == False:
                return False
        return True

    def directionificate(self):
        """
        return: angle the car needs to be driving at
        """


        #get car position
        if self.car_info is None:
            self.get_logger().info("Failed to get car information")
            return None
        car_info = self.car_info

        POI = self.POI

        if (not len(POI) or self.allTrue(self.visited)) and not self.requesting_poi:
            #self.visited = [False for _ in range(len(self.POI))]
            self.get_logger().info("Calling POI service")
            self.req = GeneratePOI.Request()
            self.req.num_points = self.num
            self.future = self.cli.call_async(self.req)
            self.requesting_poi = True  # Block further calls
            return None


        #perform calculation for closest point
        point = self.closest_point(car_info, POI)
        if point == -1:
            self.get_logger().info("No unvisited POIs available.")
            return None
        self.get_logger().info(f"Point to reach is ({POI[point].x}, {POI[point].y})")

        #if the distance between the point and the car is relatively small, we have reached the point
        collision_dist = 0.3
        if self.dist(car_info, POI[point]) <= collision_dist:
            #remove it from the list
            self.get_logger().info("\n==========\nReach a point\n==========\n")
            #self.POI.pop(point)
            self.visited[point] = True
            self.kill_turtle(f'point{point}')
            return None


        angle_to_point = math.atan2(POI[point].y - car_info.y, POI[point].x - car_info.x)
        angle_diff = angle_to_point - car_info.theta

        # Normalize to [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        return angle_diff

    def timer_callback(self):
        if self.future and self.future.done():
            try:
                response = self.future.result()
                self.POI = response.poses
                self.visited = [False for _ in range(len(self.POI))]
                self.get_logger().info("POIs received and set.")
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
            self.future = None  # reset so we can request again later
            self.requesting_poi = False

        angle = self.directionificate()
        if angle is not None:
            msg = String()
            self.get_logger().info(f'Publishing angle: {angle:.2f}')
            msg.data = f'Angle: {angle:.2f}'
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    angler = Anglificator()

    rclpy.spin(angler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    angler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
