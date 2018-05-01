#!/usr/bin/env python
import rospy
import sys

import numpy as np
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from math import cos, sin, asin, tan, atan2, radians, atan
# msgs and srv for working with the set_model_service
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
from math import pow,atan2,sqrt
# a handy tool to convert orientations
from tf.transformations import euler_from_quaternion, quaternion_from_euler


distance_tolerance = 0.5
tolerance= 0.001
angle_tolerance = 0.01
max_linear_speed= 0.2
max_angular_speed= 3
vel_P = 1
vel_I = 0
vel_D = 0

ang_P = 2
ang_I = 0
ang_D = 0

point_P = 10
point_I = 0
point_D = 0

back_P = 10
back_I = 0
back_D = 2

"""
Aux variables for the three states of the angry turtle
"""
WALKING = W = 0
TURNING = T = 1
POINTED = P = 2
WALKING2M = W2 = 3


"""
Useful class to implement a PID controller (as seen in class)
"""
class PID:
    
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_e = None
        self.sum_e = 0
        
    def step(self, e, dt):
        """ dt should be the time interval from the last method call """
        if(self.last_e is not None):
            derivative = (e - self.last_e) / dt
        else:
            derivative = 0
        self.last_e = e
        self.sum_e += e * dt
        return self.Kp * e + self.Kd * derivative + self.Ki * self.sum_e

"""
Useful method to implement angle difference (as seen in class)
"""
def angle_difference(angle1, angle2):
    return np.arctan2(np.sin(angle1-angle2), np.cos(angle1-angle2))
    assert(np.isclose(angle_difference(0.5*np.pi, 0),  0.5*np.pi))
    assert(np.isclose(angle_difference(1.5*np.pi, 0), -0.5*np.pi))
    assert(np.isclose(angle_difference(np.pi*-2/3, np.pi*2/3), np.pi*2/3))
    assert(np.isclose(angle_difference(0, 2*np.pi), 0))

class BasicThymio:

    prox_sensors_measure = {'thymio14/proximity_rear_left_link':0, 'thymio14/proximity_rear_right_link':0, 'thymio14/proximity_right_link': 0, 'thymio14/proximity_center_right_link': 0, 'thymio14/proximity_center_link': 0, 'thymio14/proximity_left_link': 0, 'thymio14/proximity_center_left_link': 0}

    def __init__(self, thymio_name):
        """init"""
        self.thymio_name = thymio_name
        rospy.init_node('basic_thymio_controller', anonymous=True)

        # Publish to the topic '/thymioX/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(self.thymio_name + '/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber(self.thymio_name + '/odom',
                                                Odometry, self.update_state)

        #Subscriber to frontal proximity sensors
        self.prox_c = rospy.Subscriber(self.thymio_name + '/proximity/center', Range, self.check_transition)
        self.prox_cl = rospy.Subscriber(self.thymio_name + '/proximity/center_left', Range, self.check_transition)
        self.prox_l = rospy.Subscriber(self.thymio_name + '/proximity/left', Range, self.check_transition)
        self.prox_cr = rospy.Subscriber(self.thymio_name + '/proximity/center_right', Range, self.check_transition)
        self.prox_r = rospy.Subscriber(self.thymio_name + '/proximity/right', Range, self.check_transition)
        self.prox_rear_r = rospy.Subscriber(self.thymio_name + '/proximity/rear_left', Range, self.check_transition)
        self.prox_rear_l = rospy.Subscriber(self.thymio_name + '/proximity/rear_right', Range, self.check_transition)

        self.current_pose = Pose()
        self.yaw = 0
        self.current_twist = Twist()
        # publish at this rate
        self.hz =10.0
        self.rate = rospy.Rate(self.hz)

        self.vel_controller= PID(vel_P, vel_I, vel_D)

        #High P for the angular vel makes the turtle turning faster and writing USI simpler
        self.angle_controller = PID(ang_P, ang_I, ang_D)
        self.pointer_controller = PID(point_P, point_I, point_D)
        self.back_controller = PID(back_P, back_I, back_D)
        self.dt = 1.0/self.hz
        self.state = WALKING
        self.from_wall = 0

    def check_transition(self, data):
        self.prox_sensors_measure[data.header.frame_id]= data.range
        if data.range < 0.04 and self.state == WALKING:            
            self.state = TURNING

    def thymio_state_service_request(self, position, orientation):
        """Request the service (set thymio state values) exposed by
        the simulated thymio. A teleportation tool, by default in gazebo world frame.
        Be aware, this does not mean a reset (e.g. odometry values)."""
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            model_state = ModelState()
            model_state.model_name = self.thymio_name
            model_state.reference_frame = '' # the frame for the pose information
            model_state.pose.position.x = position[0]
            model_state.pose.position.y = position[1]
            model_state.pose.position.z = position[2]
            qto = quaternion_from_euler(orientation[0], orientation[0], orientation[0], axes='sxyz')
            model_state.pose.orientation.x = qto[0]
            model_state.pose.orientation.y = qto[1]
            model_state.pose.orientation.z = qto[2]
            model_state.pose.orientation.w = qto[3]
            # a Twist can also be set but not recomended to do it in a service
            gms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            response = gms(model_state)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def update_state(self, data):
        """A new Odometry message has arrived. See Odometry msg definition."""
        # Note: Odmetry message also provides covariance
        self.current_pose = data.pose.pose
        self.current_twist = data.twist.twist
        quat = (
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion (quat)
        self.yaw = yaw
        #rospy.loginfo("State from Odom: (%.5f, %.5f, %.5f) " % (self.current_pose.position.x, self.current_pose.position.y, yaw))

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.current_pose.position.x), 2) + pow((goal_y - self.current_pose.position.y), 2))
        return distance

    def move2goal(self, moves):
        goal_pose = Pose()
        goal_pose.position.x = moves[0]
        goal_pose.position.y = moves[1]
        
        #distance_tolerance = 0.2
        vel_msg = Twist()

        while self.get_distance(goal_pose.position.x, goal_pose.position.y) >= distance_tolerance:

            #Porportional Controller
            #linear velocity in the x-axis:
            dist_error = self.get_distance(goal_pose.position.x, goal_pose.position.y) 
            vel_msg.linear.x = np.clip(self.vel_controller.step(dist_error, self.dt), 0.0, max_linear_speed)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angle_controller.step(angle_difference(atan2(goal_pose.position.y - self.current_pose.position.y, goal_pose.position.x - self.current_pose.position.x), self.yaw), self.dt)

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

    def point_wall(self):
        vel_msg = Twist()
        while (np.abs(self.prox_sensors_measure['thymio14/proximity_right_link'] - self.prox_sensors_measure['thymio14/proximity_left_link']) > tolerance):
            error = self.prox_sensors_measure['thymio14/proximity_right_link'] - self.prox_sensors_measure['thymio14/proximity_left_link']
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = np.clip(self.pointer_controller.step(error, self.dt), -max_angular_speed, max_angular_speed)
            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

    def adjust(self):
        vel_msg = Twist()
        while (np.abs(self.prox_sensors_measure['thymio14/proximity_rear_left_link'] -  self.prox_sensors_measure['thymio14/proximity_rear_right_link']) > tolerance):
            error = self.prox_sensors_measure['thymio14/proximity_rear_left_link'] -  self.prox_sensors_measure['thymio14/proximity_rear_right_link'] 
            rospy.loginfo(error)
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = np.clip(self.back_controller.step(error, self.dt), -max_angular_speed, max_angular_speed)
            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

    def move2angle(self, goal_angle):
        vel_msg = Twist()

        while angle_difference(goal_angle, self.yaw) > angle_tolerance:
            #Porportional Controller
            #linear velocity in the x-axis: 
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = np.clip(self.angle_controller.step(angle_difference(goal_angle, self.yaw), self.dt), -max_angular_speed, max_angular_speed)

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

    def run_thymio(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.1 # m/s
        vel_msg.angular.z = 0. # rad/s
        while self.state == WALKING and  not rospy.is_shutdown():
            self.velocity_publisher.publish(vel_msg)
            # .. at the desired rate.
            self.rate.sleep()
        if self.state == TURNING:
            vel_msg.linear.x = 0
            vel_msg.angular.z =0
            self.velocity_publisher.publish(vel_msg)
            self.point_wall()
            self.state = POINTED
        if self.state == POINTED:
            self.from_wall = self.prox_sensors_measure['thymio14/proximity_center_link']
            self.move2angle(np.pi+ self.yaw)
            self.adjust()
            self.state = WALKING2M
        if self.state == WALKING2M:
            print("computing new destination")
            # compute destination position
            x_n = self.current_pose.position.x + 2*cos(self.yaw)
            y_n = self.current_pose.position.y + 2*sin(self.yaw)
            print("start to walk")
            self.move2goal((x_n, y_n))
# move to goal
        '''
        while self.usi_moves_idx<len(self.usi_moves):
                self.move2goal(self.usi_moves[self.usi_moves_idx])
                self.usi_moves_idx +=1'''

def usage():
    return "Wrong number of parameters. easy_point_wall.py [thymio_name]"

if __name__ == '__main__':
    if len(sys.argv) == 2:
        thymio_name = sys.argv[1]
        print "Now working with robot: %s" % thymio_name
    else:
        print usage()
        sys.exit(1)
    thymio = BasicThymio(thymio_name)

    # Teleport the robot to a certain pose. If pose is different to the
    # origin of the world, you must account for a transformation between
    # odom and gazebo world frames.
    # NOTE: The goal of this step is *only* to show the available
    # tools. The launch file process should take care of initializing
    # the simulation and spawning the respective models
    
    #thymio.thymio_state_service_request([0.,0.,0.], [0.,0.,0.])
    #rospy.sleep(1.)

    #thymio.basic_move()
    while not rospy.is_shutdown():
        thymio.run_thymio()
        sys.exit(1)
        

