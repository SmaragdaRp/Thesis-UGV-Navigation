# Import libraries
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import rospkg
from math import sqrt
import time

counter = 0
oldX = 0.0
oldY = 0.0
PI = 3.14
rotation = None

# Class responsible for moving the Turtlebot
class RobotMovement:

    def __init__(self):
        rospy.init_node("move_robot")
        self.vel_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=1)
        self.move = Twist()


    def stop(self):
        self.move.linear.x = 0.0
        self.move.angular.z = 0.0
        while True:
            if self.vel_publisher.get_num_connections() > 0:
                self.vel_publisher.publish(self.move)
                break


    def move_forward(self, dist):
        self.move.linear.x = dist
        self.move.angular.z = 0.0
        # Wait to connect to topic
        while True:
            if self.vel_publisher.get_num_connections() > 0:
                self.vel_publisher.publish(self.move)
                break


    def rotate(self, clockwise=True):

        speed = 90
        angle = 130

        #Converting from degrees to radians
        angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360
    
        #We won't use linear components
        self.move.linear.x = 0
        self.move.linear.y = 0
        self.move.linear.z = 0
        self.move.angular.x = 0
        self.move.angular.y = 0
 
        # Checking if our movement is CW or CCW
        if clockwise:
            self.move.angular.z = -abs(angular_speed)
            print("Rotating clockwise")
        else:
            self.move.angular.z = abs(angular_speed)
            print("Rotating counter clockwise")

        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            while True:
                if self.vel_publisher.get_num_connections() > 0:
                    self.vel_publisher.publish(self.move)
                    break
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        #Forcing our robot to stop
        self.move.angular.z = 0
        while True:
            if self.vel_publisher.get_num_connections() > 0:
                self.vel_publisher.publish(self.move)
                break


    def move_backwards(self, dist):

        speed = 90
        angle = 200
        #Converting from angles to radians
        angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360
    
        #We wont use linear components
        self.move.linear.x = 0
        self.move.linear.y = 0
        self.move.linear.z = 0
        self.move.angular.x = 0
        self.move.angular.y = 0

        self.move.angular.z = -abs(angular_speed)
        print("Rotating 180 degrees")

        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            while True:
                if self.vel_publisher.get_num_connections() > 0:
                    self.vel_publisher.publish(self.move)
                    break
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)
        time.sleep(5)

        #Forcing our robot to stop
        self.move.angular.z = 0
        self.vel_publisher.publish(self.move)

        self.move.linear.x = dist
        self.move.angular.z = 0.0
        while True:
            if self.vel_publisher.get_num_connections() > 0:
                self.vel_publisher.publish(self.move)
                break       
