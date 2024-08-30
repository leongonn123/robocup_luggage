#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class TurtleBotApproach:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('turtlebot_proportional_control_node')

        # Publisher to control TurtleBot's velocity
        self.vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

        # Subscriber to the bag distance and angle topics
        rospy.Subscriber('/bag_distance', Float32, self.distance_callback)
        rospy.Subscriber('/bag_angle', Float32, self.angle_callback)

        # Desired distance from the bag (in meters)
        self.target_distance = 0.55

        # Proportional control gains
        self.kp_dist = 0.5  # Proportional gain for distance
        self.kp_ang = 2.0   # Proportional gain for angle

        # Maximum velocities
        self.max_linear_vel = 0.1  # Maximum linear velocity
        self.max_angular_vel = 0.3  # Maximum angular velocity

        # Initialize velocity command
        self.cmd_vel = Twist()

        # Initialize bag distance and angle
        self.distance_to_bag = None
        self.angle_to_bag = None

        # Flag to determine if the robot should stop moving
        self.stop_moving = False

    def distance_callback(self, msg):
        self.distance_to_bag = msg.data
        self.update_velocity()

    def angle_callback(self, msg):
        self.angle_to_bag = msg.data
        self.update_velocity()

    def update_velocity(self):
        # Ensure that both distance and angle data are available
        if self.distance_to_bag is None or self.angle_to_bag is None:
            return

        rospy.loginfo(f'Distance to bag: {self.distance_to_bag:.2f} meters, Angle to bag: {self.angle_to_bag:.2f} radians')

        # Calculate the error between current distance and target distance
        error_dist = self.distance_to_bag - self.target_distance

        # Calculate the error for angle (assuming the target angle is 0 radians, meaning centered)
        error_ang = self.angle_to_bag

        # Compute the proportional control outputs
        control_output_dist = self.kp_dist * error_dist
        control_output_ang = self.kp_ang * error_ang

        # Set the angular velocity based on the proportional control output for angle
        self.cmd_vel.angular.z = max(min(-control_output_ang, self.max_angular_vel), -self.max_angular_vel)

        # Set the linear velocity only if the angular error is small
        if abs(error_ang) < 0.2:  # Allow small angular errors before moving forward/backward
            self.cmd_vel.linear.x = max(min(control_output_dist, self.max_linear_vel), -self.max_linear_vel)
        else:
            self.cmd_vel.linear.x = 0.0  # Stop forward/backward movement if the angular error is too large

        # Publish the velocity command
        self.vel_pub.publish(self.cmd_vel)

        # Print the velocity commands
        rospy.loginfo(f'Published velocity - Linear: {self.cmd_vel.linear.x:.2f}, Angular: {self.cmd_vel.angular.z:.2f}')
        rospy.loginfo(f'Control Output - Linear: {control_output_dist:.2f}, Angular: {control_output_ang:.2f}')

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        turtlebot_approach = TurtleBotApproach()
        turtlebot_approach.run()
    except rospy.ROSInterruptException:
        pass
