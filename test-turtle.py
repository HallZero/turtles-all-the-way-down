import rclpy
from random import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen, Spawn
from time import sleep

stroke = [  (2.0, 8.5, 0.0, 3.0, 0.0),          # x, y, theta, linear, angular
            (3.0, 9.0, 4.71, 1.0, 0.0),
            (4.0, 9.0, 4.71, 1.0, 0.0),
            (2.5, 8.0, 4.45, 1.0, -0.3),
            (2.5, 8.0, 0.0, 2.0, 0.0),
            (4.5, 8.0, 4.45, 3.0, -0.3),
            (2.8, 7.9, 4.97, 0.5, 0.3),
            (4.0, 7.9, 4.45, 0.5, -0.3),
            (2.8, 7.2, 0.0, 1.0, 0.0),
            (3.4, 7.9, 4.71, 1.5, 0.0),
            (3.2, 7.2, 4.45, 1.0, -0.3),
            (3.6, 7.2, 4.71, 1.0, 0.3),
            # ----------------------------
            (6.0, 8.5, 4.97, 0.5, 0.3),
            (6.0, 8.0, 4.97, 0.5, 0.3),
            (6.0, 6.0, 0.90, 0.5, 0.3),
            (7.0, 7.5, 0.0, 3.0, 0.1),
            (10.0, 7.6, 4.45, 0.75, -0.3),
            (8.7, 8.5, 4.71, 2.5, -0.1),
            (7.8, 8.5, 4.71, 2.8, 0.0),
            (7.8, 5.7, 4.41, 0.5, 1.0),
            (7.8, 5.2, 0.0, 2.0, 0.1)
            ]

class TurtleHub(Node):
    def __init__(self):
        super().__init__('turtle_hub')
        self.tp = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen = self.create_client(SetPen, '/turtle1/set_pen')
        self.spawn_ = self.create_client(Spawn, '/spawn')
        self.draw_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.status = False

    def togglePen(self, state):
        req = SetPen.Request()
        req.r = 16
        req.g = 20
        req.b = 10
        req.width = 1
        req.off = state

        return req
        

    def teleport(self, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)

        return req
    
    def spawn(self, x, y, theta, index):
        req = Spawn.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        req.name = 'turtle{index}'

        return req
    
    def draw(self, linear, angular):
        twist_msg_ = Twist()

        twist_msg_.linear.x = linear
        twist_msg_.angular.z = angular
        self.draw_.publish(twist_msg_)


def main(args=None):
    rclpy.init(args=args)
    turtle_hub = TurtleHub()
    for i in range (0, 21):
        pen = turtle_hub.pen.call_async(turtle_hub.togglePen(True))
        rclpy.spin_until_future_complete(turtle_hub, pen)
        future = turtle_hub.tp.call_async(turtle_hub.teleport(stroke[i][0], stroke[i][1], stroke[i][2]))
        rclpy.spin_until_future_complete(turtle_hub, future)
        pen = turtle_hub.pen.call_async(turtle_hub.togglePen(False))
        rclpy.spin_until_future_complete(turtle_hub, pen)
        turtle_hub.draw(stroke[i][3], stroke[i][4])
        sleep(1)
    pen = turtle_hub.pen.call_async(turtle_hub.togglePen(True))
    rclpy.spin_until_future_complete(turtle_hub, pen)
    home = turtle_hub.tp.call_async(turtle_hub.teleport(0.0, 0.0, 0.0))
    rclpy.spin_until_future_complete(turtle_hub, home)
    turtle_hub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
