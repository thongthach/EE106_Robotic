#!/usr/bin/env python3

from math import pi, sqrt, atan2, cos, sin

import numpy as np


import rospy

import tf

from std_msgs.msg import Empty

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist, Pose2D


class Controller:

    def __init__(self, P=0.0, D=0.0, set_point=0):

        self.Kp = P

        self.Kd = D

        self.set_point = set_point # reference (desired value)

        self.previous_error = 0


    def update(self, current_value):

        # calculate P_term and D_term

        error = self.set_point - current_value

        P_term = self.Kp * error

        D_term = self.Kd * (error - self.previous_error)

        self.previous_error = error

        return P_term + D_term


    def setPoint(self, set_point):

        self.set_point = set_point

        self.previous_error = 0

    

    def setPD(self, P=0.0, D=0.0):

        self.Kp = P

        self.Kd = D

        print("Kp=",self.Kp)

        print("Kd=",self.Kd)


class Turtlebot:

    def __init__(self):

        rospy.init_node("turtlebot_move")

        rospy.loginfo("Press Ctrl + C to terminate")

        self.vel_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)

        self.rate = rospy.Rate(10)


        self.pose = Pose2D()

        self.logging_counter = 0

        self.trajectory = list()

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)


        try:

            self.run()

        except rospy.ROSInterruptException:

            rospy.loginfo("Action terminated.")

        finally:

            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


    def run(self):

        vel = Twist()

        pd_control = Controller()

        pd_control.setPD(0.005, 0.005)

        

        pd_control.setPoint(4.0)

        

        pd_control_turn = Controller()

        pd_control_turn.setPD(0.1, 0.1)

        

        final_x = 4.0

        final_y = 0.0

        

        # For going straight path 5 meters

        while abs(final_x - self.pose.x) > 0.01:

            adjust_pos = pd_control.update(self.pose.x)

            velocity = max(abs(adjust_pos * 10), 0.05)

            vel.linear.x = velocity

            self.vel_pub.publish(vel)

            self.rate.sleep()

        

        # For the rotation 90 degrees

        pd_control_turn.setPoint(pi/2)

        final_rotation = pi/2

        

        while abs(final_rotation - self.pose.theta) > 0.01:

            adjust_angle = pd_control_turn.update(self.pose.theta)

            velocity_theta = max(abs(adjust_angle * 10), 0.05)

            vel.linear.x = 0

            vel.angular.z = velocity_theta

            self.vel_pub.publish(vel)

            self.rate.sleep()

        

        # Update target values for the next movement

        final_x = 0.0

        final_y = 4.0

        print("working?")

        # For going straight path 5 meters

        while abs(final_y - self.pose.y) > 0.01:

            adjust_pos = pd_control.update(self.pose.y)

            velocity = max(abs(adjust_pos * 10), 0.05)

            vel.linear.x = velocity

            vel.angular.z = 0

            self.vel_pub.publish(vel)

            self.rate.sleep()

        

        # For the rotation 90 degrees

        pd_control_turn.setPoint(pi)

        final_rotation = pi

        

        while abs(final_rotation - self.pose.theta) > 0.01:

            adjust_angle = pd_control_turn.update(self.pose.theta)

            velocity_theta = max(abs(adjust_angle * 10), 0.05)

            vel.linear.y = 0

            vel.angular.z = velocity_theta

            self.vel_pub.publish(vel)

            self.rate.sleep()

            

        # for going straight 5 more meters:

        final_x = 0.0

        final_y = 0.0

        

        # For going straight path 5 meters

        while abs(final_x - self.pose.x) > 0.01:

            adjust_pos = pd_control.update(self.pose.x)

            velocity = max(abs(adjust_pos * 10), 0.05)

            vel.linear.x = velocity

            vel.angular.z =0.0

            self.vel_pub.publish(vel)

            self.rate.sleep()

        

      

        # for turning 3pi/2

        pd_control_turn.setPoint(-pi/2)

        final_rotation = -pi/2

        

        while abs(final_rotation - self.pose.theta) > 0.01:

            adjust_angle = pd_control_turn.update(self.pose.theta)

            velocity_theta = max(abs(adjust_angle * 10), 0.05)

            vel.angular.z = velocity_theta

            vel.linear.x = 0

            self.vel_pub.publish(vel)

            self.rate.sleep()

         #going straight for 5 more meters:

        final_x = 0.0

        final_y = 0.0

        

        # For going straight path 5 meters

        while abs(final_y - self.pose.y) > 0.01:

            adjust_pos = pd_control.update(self.pose.y)

            velocity = max(abs(adjust_pos * 10), 0.05)

            vel.linear.x = velocity

            vel.angular.z =0

            self.vel_pub.publish(vel)

            self.rate.sleep()
         
        vel.linear.x = 0
        vel.angular.z =0


    def odom_callback(self, msg):

        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,

                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

        self.pose.theta = yaw

        self.pose.x = msg.pose.pose.position.x

        self.pose.y = msg.pose.pose.position.y


        self.logging_counter += 1

        if self.logging_counter == 100:

            self.logging_counter = 0

            self.trajectory.append([self.pose.x, self.pose.y])

            rospy.loginfo("odom: x=" + str(self.pose.x) +

                          ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':

    whatever = Turtlebot()




