import rclpy
from random import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen, Spawn

class TurtleHub(Node):
    def __init__(self):
        super().__init__('turtle_hub')
        self.client_tp = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.client_pen = self.create_client(SetPen, '/turtle1/set_pen')
        self.client_spawn = self.create_client(Spawn, '/spawn')
        self.publisher_draw = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)
        self.draw()

    def togglePen(self):
        req = SetPen.Request()
        req.r = 16
        req.g = 20
        req.b = 10
        req.width = 1
        req.off = True

        return req
        

    def teleport(self):
        req = TeleportAbsolute.Request()
        req.x = 2.0
        req.y = 8.0
        req.theta = 0.0

        return req
    
    def spawn(self, x, y, theta):
        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = theta
        req.name = 'turtle2'

        return req
    
    def draw(self):
        twist_msg_ = Twist()

        twist_msg_.linear.x = 1.0
        twist_msg_.angular.z = 0.0
        self.publisher_draw.publish(twist_msg_)


def main(args=None):
    rclpy.init(args=args)
    turtle_hub = TurtleHub()
    future = turtle_hub.client_spawn.call_async(turtle_hub.spawn(4.0, 9.0, 4.71))
    rclpy.spin_until_future_complete(turtle_hub, future)
    #rclpy.spin_once(turtle_hub)
    rclpy.spin(turtle_hub)
    result = future.result()
    turtle_hub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
