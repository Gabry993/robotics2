#!/usr/bin/env python
import rospy
import sys

import numpy as np
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from math import cos, sin, asin, tan, atan2, radians, atan, isnan
# msgs and srv for working with the set_model_service
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
from math import pow,atan2,sqrt
# a handy tool to convert orientations
from tf.transformations import euler_from_quaternion, quaternion_from_euler


distance_tolerance = 0.5    #distance tolerance from goal point for move2goal
tolerance= 0.001            #tolerance of the difference between proximity sensors to point the wall
angle_tolerance = 0.01      #angle tolerance from goal angle for move2angle
wall_threshold = 0.07       #threshold to stop the Thymio when a wall is detected
max_linear_speed= 0.14      #max thymio linear speed
max_angular_speed= 3        #max Thymio angular speed
max_sensor_dist=0.13        #max distance detected by sensor (used to raplace inf in real world)
min_sensor_dist=0.01        #min distance detected by sensor (used to raplace -inf in real world)

#Parameter for linear velocity PID Controller
vel_P = 1
vel_I = 0
vel_D = 0

#Parameter for angular velocity PID Controller
ang_P = 10
ang_I = 0
ang_D = 0

#Parameter for the PID Controller that controlls angular speed with respect to the difference between 2 proximity sensors
point_P = 10
point_I = 0
point_D = 0

#Parameter for the PID Controller that controlls angular speed with respect to the difference between 2 rear proximity sensors
back_P = 10
back_I = 0
back_D = 2

#Used to move 2m away from the wall
wall_distance=2.5 #1.0 = circa 0.5cm

"""
Aux variables for the states of the Thymio
"""
WALKING = W = 0     #Is walking towards the wall
TURNING = T = 1     #Is turning to be orthogonal to the wall
POINTED = P = 2     #Is pointed, so now is turning 180 degrees
WALKING2M = W2 = 3  #is walking 2 meters away from the wall


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
    #Dict to keep the last value measured from each proximity sensor
    prox_sensors_measure = {'thymio14/proximity_rear_left_link':0, 'thymio14/proximity_rear_right_link':0, 'thymio14/proximity_right_link': 0, 'thymio14/proximity_center_right_link': 0, 'thymio14/proximity_center_link': 0, 'thymio14/proximity_left_link': 0, 'thymio14/proximity_center_left_link': 0}

    def __init__(self, thymio_name):
        """init"""
        self.thymio_name = thymio_name
        rospy.init_node('basic_thymio_controller', anonymous=True)

        # Publish to the topic '/thymioX/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(self.thymio_name + '/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/odom'. self.update_state is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber(self.thymio_name + '/odom',
                                                Odometry, self.update_state)

        #Subscriber to frontal and rear proximity sensors
        self.prox_c = rospy.Subscriber(self.thymio_name + '/proximity/center', Range, self.check_transition)
        self.prox_cl = rospy.Subscriber(self.thymio_name + '/proximity/center_left', Range, self.check_transition)
        self.prox_l = rospy.Subscriber(self.thymio_name + '/proximity/left', Range, self.check_transition)
        self.prox_cr = rospy.Subscriber(self.thymio_name + '/proximity/center_right', Range, self.check_transition)
        self.prox_r = rospy.Subscriber(self.thymio_name + '/proximity/right', Range, self.check_transition)
        self.prox_rear_r = rospy.Subscriber(self.thymio_name + '/proximity/rear_left', Range, self.check_transition)
        self.prox_rear_l = rospy.Subscriber(self.thymio_name + '/proximity/rear_right', Range, self.check_transition)

        self.current_pose = Pose()
        
        #we always keep the yaw as it's useful
        self.yaw = 0
        self.current_twist = Twist()
        # publish at this rate
        self.hz =10.0
        self.rate = rospy.Rate(self.hz)

        #All the needed PID
        self.vel_controller= PID(vel_P, vel_I, vel_D)               #based on distance from goal point
        self.angle_controller = PID(ang_P, ang_I, ang_D)            #based on difference between self.yaw and goal angle
        self.pointer_controller = PID(point_P, point_I, point_D)    #based on difference between left and right front proximity sensors
        self.back_controller = PID(back_P, back_I, back_D)          #based on difference between left and right back proximity sensors

        self.dt = 1.0/self.hz
        #set the initial state
        self.state = WALKING
        self.from_wall = 0

    #Callback for all proximity sensors subscribers
    #The range is saved and if the thymio is closer than wall_threshold to the wall, it stops
    def check_transition(self, data):

        #This part is needed on the real Thymio, where over its max_range and under its min_range a sensor gives back inf/-inf
        if(data.range==float("inf")):
            self.prox_sensors_measure[data.header.frame_id]= max_sensor_dist
            #print("reading inf sensor: "+str(data.header.frame_id)+" value: "+str(data.range))
        elif(data.range==float("-inf")):
            self.prox_sensors_measure[data.header.frame_id]= min_sensor_dist
            #print("reading -inf sensor: "+str(data.header.frame_id)+" value: "+str(data.range))
        else:
            self.prox_sensors_measure[data.header.frame_id]= data.range

        if data.range < wall_threshold and self.state == WALKING:            
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
        self.yaw = yaw #we save the yaw
        #rospy.loginfo("State from Odom: (%.5f, %.5f, %.5f) " % (self.current_pose.position.x, self.current_pose.position.y, yaw))

    #return the distance between the current position of the Thymio (obtained with odometry) and a goal point (all in the world reference frame)
    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.current_pose.position.x), 2) + pow((goal_y - self.current_pose.position.y), 2))
        return distance

    #Use 2 PID controller to move the Thymio to a goal point (one for linear and one for angular velocity)
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

    #Use a PID to rotate the Thymio and make it orthogonal to the wall
    #It uses the difference between left and right proximity sensors: if it's 0 it means that the Thymio is orthogonal
    def point_wall(self):
        vel_msg = Twist()
        while (np.abs(self.prox_sensors_measure['thymio14/proximity_right_link'] - self.prox_sensors_measure['thymio14/proximity_left_link']) > tolerance):
            error = self.prox_sensors_measure['thymio14/proximity_right_link'] - self.prox_sensors_measure['thymio14/proximity_left_link']
            error_rear = self.prox_sensors_measure['thymio14/proximity_rear_left_link'] -  self.prox_sensors_measure['thymio14/proximity_rear_right_link'] 
            #print("VALORECHEPIACE ALDARIO: "+str(error_rear))
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

    #Used to adjust the rotation of the Thymio after it has rotated to have the wall on his back
    #It works just like the point_wall, but uses ther rear sensors
    #This part has been removed with the real Thymio as those sensors were not reliable 
    def adjust(self):
        vel_msg = Twist()
        while (np.abs(self.prox_sensors_measure['thymio14/proximity_rear_left_link'] -  self.prox_sensors_measure['thymio14/proximity_rear_right_link']) > tolerance):
            error = self.prox_sensors_measure['thymio14/proximity_rear_left_link'] -  self.prox_sensors_measure['thymio14/proximity_rear_right_link'] 
            print("diff sensors: "+str(error))
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

    #Controls the angular velocity with a PID that works using the difference between the yaw of the Thymio and a goal angle
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

    #Open Loop Controller to rotate the Thymio of 180 degrees
    def turn180_magic(self):
        i=0
        vel_msg = Twist()
        while i<30:
            vel_msg.linear.x = 0 # m/s
            vel_msg.angular.z = 1 # rad/s
            self.velocity_publisher.publish(vel_msg)
            # .. at the desired rate.
            i +=1
            self.rate.sleep()
            print("gira gira")

    #Open Loop Controller to move the Thymio 2 meters aways from the wall
    def open_dritto(self):
        i=0
        vel_msg = Twist()
        while i<200:
            vel_msg.linear.x = 0.10 # m/s
            vel_msg.angular.z = 0 # rad/s
            self.velocity_publisher.publish(vel_msg)
            # .. at the desired rate.
            i +=1
            self.rate.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

    #Run the thymio with the desired parameters
    #real: True if we are working with the real Thymio, Falsefor the  simulator
    #open_loop: True if we want to use open_loop rotation or closed_loop + adjust. True is better for real Thymio, False for the simulator
    
    def run_thymio(self, real=False, open_loop=False):
        vel_msg = Twist()
        vel_msg.linear.x = 0.1 # m/s
        vel_msg.angular.z = 0. # rad/s
        while self.state == WALKING and  not rospy.is_shutdown():
            self.velocity_publisher.publish(vel_msg)
            # .. at the desired rate.
            self.rate.sleep()
        if self.state == TURNING:
            print("-------turning towards the wall")
            vel_msg.linear.x = 0
            vel_msg.angular.z =0
            self.velocity_publisher.publish(vel_msg)
            self.point_wall()
            self.state = POINTED
        if self.state == POINTED:
            print("-------turning the other way")
            self.from_wall = self.prox_sensors_measure['thymio14/proximity_center_link']
            if not real and not open_loop:
                self.move2angle(np.pi+ self.yaw)
                self.adjust()
            elif not real and open_loop:
                self.turn180_magic()
                self.adjust()
            elif real and not open_loop:
                self.move2angle(np.pi+ self.yaw)
            else:
                self.turn180_magic()
            self.state = WALKING2M
        if self.state == WALKING2M:
            print("-------computing new destination")
            # compute destination position
            '''x_n = self.current_pose.position.x + (wall_distance-self.from_wall)*cos(self.yaw)
            y_n = self.current_pose.position.y + (wall_distance-self.from_wall)*sin(self.yaw)
            print("-------start to walk")
            self.move2goal((x_n, y_n))'''
            self.open_dritto()
            print("done")

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

    real = raw_input("Are you working with the real thymio? y or n\n")
    if real == "y":
        real = True
    else:
        real = False
    open_loop = raw_input("Do you want open loop rotation? y or n\n")
    if open_loop == "y":
        open_loop = True
    else:
        open_loop = False
    while not rospy.is_shutdown():
        thymio.run_thymio(real, open_loop)
        sys.exit(0)