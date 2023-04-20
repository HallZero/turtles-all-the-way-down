import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Turtle(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.timer = self.create_timer(1, self.pub)
        self.publisher_ = self.create_publisher(String, 'turtle1/cmd_vel', 10)
        self.msg_ = String()
    
    def sub(self):
        for i in self.get_subscriptions_info_by_topic('turtle1/cmd_vel'):
            print(i)
        #self.get_subscriptions_info_by_topic('turtle1/cmd_vel')
    
    def pub(self):
        self.msg_.data = "Publishing to you!"
        self.publisher_.publish(self.msg_)


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = Turtle()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()