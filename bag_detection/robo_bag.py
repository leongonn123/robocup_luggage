#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class TurtleBotApproach:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('turtlebot_pid_centering_node')

        # Publisher to control TurtleBot's velocity
        self.vel_pub = rospy.Publisher('/mobile_base/command/velocity', Twist, queue_size=10)

        # Subscriber to the bag distance and angle topics
        rospy.Subscriber('/bag_distance', Float32, self.distance_callback)
        rospy.Subscriber('/bag_angle', Float32, self.angle_callback)  # Updated to Float32

        # Desired distance from the bag (in meters)
        self.target_distance = 0.55

        # PID control gains for distance
        self.kp_dist = 0.5  # Proportional gain
        self.ki_dist = 0.1  # Integral gain
        self.kd_dist = 0.05  # Derivative gain

        # PID control gains for angle (centering)
        self.kp_ang = 0.7   # Proportional gain
        self.ki_ang = 0.0   # Integral gain (can be tuned if needed)
        self.kd_ang = 0.1   # Derivative gain

        # Initialize PID control variables for distance
        self.prev_error_dist = 0.0
        self.integral_dist = 0.0
        self.prev_time_dist = rospy.Time.now()

        # Initialize PID control variables for angle
        self.prev_error_ang = 0.0
        self.integral_ang = 0.0
        self.prev_time_ang = rospy.Time.now()

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

        # Get current time and calculate time difference for distance PID
        current_time_dist = rospy.Time.now()
        dt_dist = (current_time_dist - self.prev_time_dist).to_sec()

        # Proportional term for distance
        p_dist = self.kp_dist * error_dist

        # Integral term for distance
        self.integral_dist += error_dist * dt_dist
        i_dist = self.ki_dist * self.integral_dist

        # Derivative term for distance
        derivative_dist = (error_dist - self.prev_error_dist) / dt_dist if dt_dist > 0 else 0.0
        d_dist = self.kd_dist * derivative_dist

        # Compute the PID output for distance
        control_output_dist = p_dist + i_dist + d_dist

        # Update previous error and time for the next distance iteration
        self.prev_error_dist = error_dist
        self.prev_time_dist = current_time_dist

        # Get current time and calculate time difference for angle PID
        current_time_ang = rospy.Time.now()
        dt_ang = (current_time_ang - self.prev_time_ang).to_sec()

        # Proportional term for angle
        p_ang = self.kp_ang * error_ang

        # Integral term for angle
        self.integral_ang += error_ang * dt_ang
        i_ang = self.ki_ang * self.integral_ang

        # Derivative term for angle
        derivative_ang = (error_ang - self.prev_error_ang) / dt_ang if dt_ang > 0 else 0.0
        d_ang = self.kd_ang * derivative_ang

        # Compute the PID output for angle
        control_output_ang = p_ang + i_ang + d_ang

        # Update previous error and time for the next angle iteration
        self.prev_error_ang = error_ang
        self.prev_time_ang = current_time_ang

        # Set the linear velocity based on the PID output for distance
        # Ensure that control_output_dist is positive when moving forward
        self.cmd_vel.linear.x = control_output_dist

        # Set the angular velocity based on the PID output for angle
        self.cmd_vel.angular.z = -control_output_ang  # Negative to correct direction

        # Check if the robot is within the target distance range
        if abs(error_dist) < 0.05:
            self.cmd_vel.linear.x = 0.0
            self.stop_moving = True
        else:
            self.stop_moving = False

        # Publish the velocity command if the robot should not stop
        if not self.stop_moving:
            self.vel_pub.publish(self.cmd_vel)

        # Print the velocity commands
        rospy.loginfo(f'Published velocity - Linear: {self.cmd_vel.linear.x:.2f}, Angular: {self.cmd_vel.angular.z:.2f}')

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        turtlebot_approach = TurtleBotApproach()
        turtlebot_approach.run()
    except rospy.ROSInterruptException:
        pass

