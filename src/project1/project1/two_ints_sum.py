#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import TwoInts
from example_interfaces.msg import Int64
    
class TwoIntsSum(Node):
    def __init__(self):
        super().__init__("two_ints_sum")
        self.subscriber_ = self.create_subscription(TwoInts,"two_ints",self.callback_two_ints,10)
        self.publisher_ = self.create_publisher(Int64,"sum",10)
        self.get_logger().info("Two Ints sum node has been started.")
    
    def callback_two_ints(self,msg:TwoInts):
        sum = Int64()
        sum.data = msg.a + msg.b
        self.publisher_.publish(sum)
        
def main(args=None):
    rclpy.init(args=args)
    node = TwoIntsSum()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()