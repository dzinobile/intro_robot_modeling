#! usr/bin/env python3

#import libraries
import rclpy
import rclpy.clock
from rclpy.node import Node
from geometry_msgs.msg import Twist

#define variables
d = float(input("Enter travel distance [m]: \n"))
t = float(input("Enter travel time [s]: \n"))
v = d/t #constant velocity for scenario 1
max_v = (4*d)/(3*t) #constant velocity for scenario 2
a = (16*d)/(3*(t**2)) #acceleration for scenario 2
rate = 100.0 #publisher rate [hz]

#deal with invalid scenario input
while True:
    scenario = int(input("Enter scenario (1 or 2)\n"))
    if scenario in [1, 2]:
        break
    else:
        print("Invalid input. Enter 1 or 2.\n")
           
#define initial linear and angular velocity 0
vel = Twist()
vel.linear.x = 0.0
vel.angular.z = 0.0
    

#create MoveRobot class and move_robot node
class MoveRobot(Node): 
    def __init__(self): 
        super().__init__("move_robot") 
        self.get_logger().info("starting node")
        
        #create publisher
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        if scenario == 1:
            self.create_timer((1/rate), self.scenario_1)
        elif scenario == 2:
            self.create_timer((1/rate), self.scenario_2)
        
        #create start time variable
        self.start_time = self.get_clock().now()
        
    #create scenario 1 function
    def scenario_1(self):
        self.elapsed_time = ((self.get_clock().now()) - self.start_time).nanoseconds / 1e9
        
        if self.elapsed_time < t:
            vel.linear.x = v
            self.publisher.publish(vel)
            self.get_logger().info("moving robot")
        else:
            vel.linear.x = 0.0
            self.publisher.publish(vel)
            self.get_logger().info("stopping robot")
            rclpy.shutdown()

    #create scenario 2  function    
    def scenario_2(self):
               
        #create elapsed time variable
        self.elapsed_time = ((self.get_clock().now()) - self.start_time).nanoseconds / 1e9
        
        #create if block for triangular velocity profile
        if self.elapsed_time < (t/4):
            vel.linear.x = vel.linear.x + (a/rate)
            self.publisher.publish(vel)
            self.get_logger().info("accelerating robot")
        elif (t/4) <= self.elapsed_time < ((3*t)/4):
            vel.linear.x = max_v
            self.publisher.publish(vel)
            self.get_logger().info("moving robot")
            
        elif ((3*t)/4) <= self.elapsed_time < (t):
            vel.linear.x = vel.linear.x - (a/rate)
            self.publisher.publish(vel)
            self.get_logger().info("decelerating robot")
        else:
            vel.linear.x = 0.0
            self.publisher.publish(vel)
            self.get_logger().info("robot stopped")
            rclpy.shutdown()
            

#use main function to call node
def main(args = None):
    rclpy.init(args=args)
    move_robot = MoveRobot()

    rclpy.spin(move_robot)
    

            
            

        
        
        
        
        
        
        
        
        
    




