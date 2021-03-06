#!/usr/bin/env python
import rospy
import sys

import numpy as np
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from math import cos, sin, asin, tan, atan2, radians
# msgs and srv for working with the set_model_service
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty
from math import pow,atan2,sqrt
# a handy tool to convert orientations
from tf.transformations import euler_from_quaternion, quaternion_from_euler


distance_tolerance = 0.5        #distance tolerance from goal point for move2goal
max_linear_speed= 0.14          #max thymio linear speed
max_angular_speed = 3           #max Thymio angular speed

#Parameter for linear velocity PID Controller
vel_P = 1
vel_I = 0
vel_D = 0

#Parameter for angular velocity PID Controller

ang_P = 2
ang_I = 0
ang_D = 0

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

        self.current_pose = Pose()

        #we always keep the yaw as it's useful
        self.yaw = 0
        self.current_twist = Twist()
        # publish at this rate
        self.hz =10.0
        self.rate = rospy.Rate(self.hz)

        #All the needed PID
        self.vel_controller= PID(vel_P, vel_I, vel_D)
        self.angle_controller = PID(ang_P, ang_I, ang_D)
        self.dt = 1.0/self.hz

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
    
    #return the distance between the current position of the Thymio (obtained with odometry) and a goal point (all in the world reference frame)
    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.current_pose.position.x), 2) + pow((goal_y - self.current_pose.position.y), 2))
        return distance

    #Use 2 PID controller to move the Thymio to a goal point (one for linear and one for angular velocity)    
    def move2goal(self, moves):
        goal_pose = Pose()
        goal_pose.position.x = moves[0]
        goal_pose.position.y = moves[1]

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
            vel_msg.angular.z = np.clip(self.angle_controller.step(angle_difference(atan2(goal_pose.position.y - self.current_pose.position.y, goal_pose.position.x - self.current_pose.position.x), self.yaw), self.dt), -max_angular_speed, max_angular_speed)

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

    #Open loop controller to move the Thymio to do an 8 trajectory made up of two circle with a radius of 0.5m
    def open_loop8(self):
        i=0
        vel_msg = Twist()
        while i<63:
            vel_msg.linear.x = 0.11 # m/s
            vel_msg.angular.z = 1 # rad/s
            self.velocity_publisher.publish(vel_msg)
            # .. at the desired rate.
            i +=1
            self.rate.sleep()
            print("gira gira")
        i=0
        vel_msg.linear.x = 0 # m/s
        vel_msg.angular.z = 0 # rad/s
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()
        print("stop1")
        while i<63:
            vel_msg.linear.x = 0.11 # m/s
            vel_msg.angular.z = -1 # rad/s
            self.velocity_publisher.publish(vel_msg)
            # .. at the desired rate.
            i +=1
            self.rate.sleep()
            print("arig arig")
        i=0
        vel_msg.linear.x = 0 # m/s
        vel_msg.angular.z = 0 # rad/s
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()
        print("stop 2")

    #Uses move2goal (so, closed loop controller) to move the Thymio on a 8 trajectory
    #The trayectory is actually computed as an "infinite sign" with a parametric function
    def closed_loop8(self):
        t=-90
        while t<1000:
            rad = radians(t)
            scale = 2 / (3 - cos(2*rad));
            x = 1.5*scale * cos(rad);
            y = 1.5*scale * sin(2*rad) / 2;
            rospy.loginfo("(x, y): (%.5f, %.5f) " % (x, y))
            self.move2goal((x, y))
            t+=10

def usage():
    return "Wrong number of parameters. basic_move.py [thymio_name]"

if __name__ == '__main__':
    if len(sys.argv) == 2:
        thymio_name = sys.argv[1]
        print "Now working with robot: %s" % thymio_name
    else:
        print usage()
        sys.exit(1)
    thymio = BasicThymio(thymio_name)

    open_loop = raw_input("Do you want open loop or closed loop 8? type o or c (open or closed)\n")
    if open_loop == "o":
        thymio.open_loop8()
    else:
        thymio.closed_loop8()
        
    
        

