#!/usr/bin/env python

# Author: Elliot Potter, CS 81, Dartmouth College
# Date: 9/17/2021

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for velocity command.
from nav_msgs.msg import Odometry

import math
import tf # library for transformations.
import numpy

# Constants.
FREQUENCY = 10 #Hz.
VELOCITY = 1 #m/s
ANGULAR_VELOCITY = 1 #radians / sec
DURATION = 5 #s how long the message should be published.


class DriveLine:
    """Class example for a ROS node."""
    def __init__(self, robot_name):
        """Initialization function."""
        self.robot_name = robot_name

        # 2nd. setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # this is an empty field, so we don't need the laser_scan
        # self.subscriber = rospy.Subscriber("base_scan", LaserScan, self.laserscan_callback)
        self.odom_subscriber = rospy.Subscriber("odom", Odometry, self.odometry_callback )

        # Setting up transformation listener.
        self.listener = tf.TransformListener()

        # We set the angular velocity to just be the robot wheels driving in opposition
        self.angular_velocity = ANGULAR_VELOCITY
        self.velocity = VELOCITY

        # We set the initial position of the robot; this will be updated by odom
        self.position = {"x": 0, "y": 0, "z": 0}
        # this is a quaternion representing the current orientation
        self.orientation = [ 0, 0, 0, 1 ]

        # Sleep important for allowing time for registration to the ROS master.
        rospy.sleep(2)

    def odometry_callback(self, odom_message):
        """Updates class variables with information about the robot state"""
        pose = odom_message.pose.pose
        self.position = pose.position
        self.orientation = [pose.orientation.x, pose.orientation.y,
                            pose.orientation.z, pose.orientation.w]

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def run_movement_loop(self, movement_time, linear_vel, angular_vel):
        """Moves the robot in the specified manner for the specified time"""
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        start_time = rospy.get_rostime()
        while not rospy.is_shutdown():
            if rospy.get_rostime() - start_time >= rospy.Duration(movement_time):
                break

            self.move(linear_vel, angular_vel)
            rate.sleep()

    def rotate(self, angle_to_rotate):
        """Rotates the robot by a certain angle"""
        time_to_rotate = angle_to_rotate / self.angular_velocity
        angular_velocity = self.angular_velocity

        # if the angle is negative, the rotation time will be negative.
        # we will flip the sign of rotation time and change angular velocity as well
        reverse_traverse_direction = time_to_rotate < 0
        if reverse_traverse_direction:
            time_to_rotate = -time_to_rotate
            angular_velocity = -angular_velocity
        self.run_movement_loop(time_to_rotate, 0, angular_velocity)

    def move_forward(self, distance):
        """Moves the robot forward by a certain distance"""
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.

        time_to_travel = distance / self.velocity
        self.run_movement_loop(time_to_travel, self.velocity, 0)

    def rotate_to_point(self, initial_orientation, current_point, desired_point):
        """
        Rotates in the direction of the specified point
        """
        difference = [desired_point[0] - current_point[0],
                      desired_point[1] - current_point[1]]
        # find the angle that we need to rotate to
        # this is from the reference frame of the previous point, where the +x direction
        # of that point is in the +x direction of the robot when it was at 0,0.
        # the angle is just opp / adj; in this case y / x

        # We use math.atan2 because Python 2 has a stupid issue where
        # it uses integer division for integers.
        # This would be fine in a typed language, but is not when
        # we are dealing with interpreted variables!
        # Fortunately, this is fixed in Python 3.
        angle = math.atan2(difference[1], difference[0])

        # we find the current orientation w.r.t the initial orientiation of the robot
        # (the initial orientation is when the robot begins the polygon)
        current_orientation = tf.transformations.euler_from_quaternion(self.orientation)[2] - initial_orientation

        # We find the angle to rotate and calculate the modulo so we don't spin in a circle
        angle_to_rotate = (angle - current_orientation) % ( 2 * math.pi )
        # we do the faster rotation if the angle is more than 180 degrees
        if angle_to_rotate > math.pi:
            angle_to_rotate = angle_to_rotate - ( 2 * math.pi )

        # rotate to the angle
        self.rotate(angle_to_rotate)

    def drive_to_point(self, current_point, desired_point):
        """
        Drives to the point
        """
        # find the difference between the current point and the next one
        difference = [desired_point[0] - current_point[0],
                      desired_point[1] - current_point[1]]

        # find the length that we need to travel
        length = math.sqrt(difference[0] ** 2 + difference[1] ** 2)
        self.move_forward(length)


    def draw_arbitrary_polygon_in_local_frame(self, vertices, initial_orientation, initial_T_odom):
        """
        For each vertex, we will rotate in the direction we need to travel, calculate the
        distance we need to travel, and then travel the distance
        """
        for i in range(len(vertices) - 1):
            # We get the current point from odometry and modify it with the initial
            # transformation matrix at the beginning of the polygon
            current_point = [self.position.x, self.position.y]
            current_point_transformed_quad = initial_T_odom.dot(
                numpy.array([current_point[0], current_point[1], 0, 1])
            )
            # transform the position of the robot into the initial orientation of the polygon
            current_point_transformed = [
                current_point_transformed_quad[0], current_point_transformed_quad[1]
            ]

            # rotate to point at the next vertex
            self.rotate_to_point(
                initial_orientation, current_point_transformed, vertices[i+1]
            )
            # drive to the next vertex
            self.drive_to_point(current_point_transformed, vertices[i+1])


    def draw_arbitrary_polygon(self, vertices):
        """
        Takes a polyline in the global reference frame, sets up the robot and produces
        a polyline in the local reference frame.
        """
        # Drives the robot to the correct initial position
        # gets the yaw (z) rotation
        current_position = [self.position.x, self.position.y, self.position.z]
        self.rotate_to_point(
            0,
            current_position,
            vertices[0]
        )
        self.drive_to_point(current_position, vertices[0])

        # Rotates the robot to the correct initial orientation
        self.rotate_to_point(0, vertices[0], vertices[1])

        # sleeps to make sure we get the most up-to-date odom information
        rospy.sleep(1)

        # Finds the transformation matrices in each direction
        # since the robot is at the origin of the polyline, we can use the
        # transforms between odom and base_link
        (trans, rot) = self.listener.lookupTransform(self.robot_name + '/odom', self.robot_name + '/base_link', rospy.Time(0))
        t = tf.transformations.translation_matrix(trans)
        R = tf.transformations.quaternion_matrix(rot)

        odom_T_baseLink = t.dot(R)
        baseLink_T_odom = numpy.linalg.inv(odom_T_baseLink)

        # Converts the polyline to the local reference frame and prints it out
        new_vertices = []
        for vertex in vertices:
            new_vertex_np = baseLink_T_odom.dot(
                numpy.array([vertex[0], vertex[1], 0, 1])
            )
            new_vertices.append([new_vertex_np[0], new_vertex_np[1]])

        print("New vertices: " + str(new_vertices))

        # prints the local-to-global transformation
        print("odom_T_baseLink: ")
        print(odom_T_baseLink)

        # runs the robot through the shape
        initial_orientation = tf.transformations.euler_from_quaternion(self.orientation)[2]
        self.draw_arbitrary_polygon_in_local_frame(
            new_vertices, initial_orientation, baseLink_T_odom
        )

