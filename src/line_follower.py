#!/usr/bin/env python

import collections
import sys

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

import utils

# The topic to publish control commands to
PUB_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation'

'''
Follows a given plan using constant velocity and PID control of the steering angle
'''


class LineFollower:
    '''
    Initializes the line follower
      plan: A list of length T that represents the path that the robot should follow
            Each element of the list is a 3-element numpy array of the form [x,y,theta]
      pose_topic: The topic that provides the current pose of the robot as a PoseStamped msg
      plan_lookahead: If the robot is currently closest to the i-th pose in the plan,
                      then it should navigate towards the (i+plan_lookahead)-th pose in the plan
      translation_weight: How much the error in translation should be weighted in relation
                          to the error in rotation
      rotation_weight: How much the error in rotation should be weighted in relation
                       to the error in translation
      kp: The proportional PID parameter
      ki: The integral PID parameter
      kd: The derivative PID parameter
      error_buff_length: The length of the buffer that is storing past error values
      speed: The speed at which the robot should travel
    '''

    def __init__(self, plan, pose_topic, plan_lookahead, translation_weight,
                 rotation_weight, kp, ki, kd, error_buff_length, speed):
        # Store the passed parameters
        self.plan = plan
        self.plan_lookahead = plan_lookahead
        # Normalize translation and rotation weights
        self.translation_weight = translation_weight / (translation_weight + rotation_weight)
        self.rotation_weight = rotation_weight / (translation_weight + rotation_weight)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # The error buff stores the error_buff_length most recent errors and the
        # times at which they were received. That is, each element is of the form
        # [time_stamp (seconds), error]. For more info about the data struct itself, visit
        # https://docs.python.org/2/library/collections.html#collections.deque
        self.error_buff = collections.deque(maxlen=error_buff_length)
        self.speed = speed

        # for error plt
        self.plt_finished = False
        self.err_hist = [] #store history of error

        # YOUR CODE HERE
        self.cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped,
                                       queue_size=10)  # Create a publisher to PUB_TOPIC
        self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb,
                                         queue_size=10)  # Create a subscriber to pose_topic, with callback 'self.pose_cb'

    '''
    Computes the error based on the current pose of the car
      cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
    Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
             (True, E) - where E is the computed error
    '''

    def compute_error(self, cur_pose):
        # Find the first element of the plan that is in front of the robot, and remove
        # any elements that are behind the robot. To do this:
        # Loop over the plan (starting at the beginning) For each configuration in the plan
        # If the configuration is behind the robot, remove it from the plan
        #   Will want to perform a coordinate transformation to determine if
        #   the configuration is in front or behind the robot
        # If the configuration is in front of the robot, break out of the loop
        cur_pose_rot = utils.rotation_matrix(-1 * cur_pose[2])
        while len(self.plan) > 0:
            offset = (self.plan[0][0:2] - cur_pose[0:2]).reshape(2, 1)
            pose = cur_pose_rot * offset
            pose.flatten()
            # If the configuration is in front of the robot, break out of the loop
            if pose[0] > 0:
                break

            # remove configuration that is behind the robot from the plan
            self.plan.pop(0)

        # Check if the plan is empty. If so, return (False, 0.0)
        # YOUR CODE HERE
        if len(self.plan) == 0:
            return False, 0.0

        # At this point, we have removed configurations from the plan that are behind
        # the robot. Therefore, element 0 is the first configuration in the plan that is in
        # front of the robot. To allow the robot to have some amount of 'look ahead',
        # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
        # We call this index the goal_index
        goal_idx = min(0 + self.plan_lookahead, len(self.plan) - 1)

        # Compute the translation error between the robot and the configuration at goal_idx in the plan
        # YOUR CODE HERE
        goal_offset = (self.plan[goal_idx][0:2] - cur_pose[0:2]).reshape(2, 1)
        translation_error = cur_pose_rot * goal_offset
        translation_error.flatten()
        translation_error = translation_error[1]
        # Compute the total error
        # Translation error was computed above
        # Rotation error is the difference in yaw between the robot and goal configuration
        rotation_error = self.plan[goal_idx][2] - cur_pose[2]
        #   Be carefult about the sign of the rotation error
        # YOUR CODE HERE
        error = self.translation_weight * translation_error + self.rotation_weight * rotation_error

        return True, error

    '''
    Uses a PID control policy to generate a steering angle from the passed error
      error: The current error
    Returns: The steering angle that should be executed
    '''

    def compute_steering_angle(self, error):
        now = rospy.Time.now().to_sec()  # Get the current time

        # Compute the derivative error using the passed error, the current time,
        # the most recent error stored in self.error_buff, and the most recent time
        # stored in self.error_buff
        # YOUR CODE HERE

        # deriv(e) = dy/dx
        # calculates the derivative when the list is not empty
        deriv_error = 0.0
        if len(self.error_buff) != 0:
            # dx = x2-x1
            dy = error - self.error_buff[-1][0]
            dx = now - self.error_buff[-1][1]
            deriv_error = dy / dx

        # Add the current error to the buffer
        self.error_buff.append((error, now))

        # Compute the integral error by applying rectangular integration to the elements
        # of self.error_buff: https://chemicalstatistician.wordpress.com/2014/01/20/rectangular-integration-a-k-a-the-midpoint-rule/
        # YOUR CODE HERE

        # integral(e) = sum(area of rectangle)
        # = sum(base * height)
        # =  sum(x_(i+1) - x_i) * f(0.5(x_i + x_(i+1))

        # initialize the integral error
        integ_error = 0.0

        # calculating interal
        for i in range(len(self.error_buff) - 1):
            # base = time_(i+1) - time_i
            base = self.error_buff[i + 1][1] - self.error_buff[i][1]
            # height = 0.5*(error_i + error_(i+1))
            height = 0.5 * (self.error_buff[i][0] + self.error_buff[i + 1][0])
            area = base * height
            integ_error += area

        # Compute the steering angle as the sum of the pid errors
        # YOUR CODE HERE
        return self.kp * error + self.ki * integ_error + self.kd * deriv_error

    # !!! please fix: steering_angle should be a number in radian but compute_steering_angle is returning a matrix !!!

    '''
    Callback for the current pose of the car
      msg: A PoseStamped representing the current pose of the car
      This is the exact callback that we used in our solution, but feel free to change it
    '''

    def write2csv(self):
        dir = "/home/robot/catkin_ws/src/lab3/err_csv/"
        filename = 'Kp: '+ str(self.kp)+'; Ki: '+str(self.ki)+'; Kd: '+str(self.kd)+'; trans w: '+str(round(self.translation_weight,2))+'; rot w: '+str(round(self.rotation_weight,2))+".csv"
        np.savetxt(dir+filename, np.asarray(self.err_hist), delimiter=",")


    def pose_cb(self, msg):
        if len(self.plan) == 0 and self.plt_finished is False:
            self.write2csv()
            print("err_hist wrote to csv.")
            self.plt_finished = True
            pass

        cur_pose = np.array([msg.pose.position.x,
                             msg.pose.position.y,
                             utils.quaternion_to_angle(msg.pose.orientation)])
        success, error = self.compute_error(cur_pose)

        self.err_hist.append(error)

        if not success:
            # We have reached our goal
            self.pose_sub = None  # Kill the subscriber
            self.speed = 0.0  # Set speed to zero so car stops

        # !!! please fix: steering_angle should be a number in radian but compute_steering_angle is returning a matrix !!!
        delta = self.compute_steering_angle(error)

        # Setup the control message
        ads = AckermannDriveStamped()
        ads.header.frame_id = '/map'
        ads.header.stamp = rospy.Time.now()
        ads.drive.steering_angle = delta
        ads.drive.speed = self.speed

        # Send the control message
        self.cmd_pub.publish(ads)


def main():
    rospy.init_node('line_follower', anonymous=True)  # Initialize the node

    # Load these parameters from launch file
    # We provide suggested starting values of params, but you should
    # tune them to get the best performance for your system
    # Look at constructor of LineFollower class for description of each var
    # 'Default' values are ones that probably don't need to be changed (but you could for fun)
    # 'Starting' values are ones you should consider tuning for your system
    # YOUR CODE HERE
    plan_topic = rospy.get_param('~plan_topic', '/planner_node/car_plan')  # Default val: '/planner_node/car_plan'
    pose_topic = rospy.get_param('~pose_topic', '/car/pose')  # Default val: '/car/pose'
    plan_lookahead = rospy.get_param('plan_lookahead', 5)  # Starting val: 5
    translation_weight = rospy.get_param('~translation_weight', 1.0)  # Starting val: 1.0
    rotation_weight = rospy.get_param('~rotation_weight', 0.0)  # Starting val: 0.0
    kp = rospy.get_param('~kp', 1.0)  # Startinig val: 1.0
    ki = rospy.get_param('~ki', 0.0)  # Starting val: 0.0
    kd = rospy.get_param('~kd', 0.0)  # Starting val: 0.0
    error_buff_length = rospy.get_param('~error_buff_length', 10)  # Starting val: 10
    speed = rospy.get_param('~speed', 1.0)  # Default val: 1.0

    raw_input("Press Enter to when plan available...")  # Waits for ENTER key press

    # Use rospy.wait_for_message to get the plan msg
    # Convert the plan msg to a list of 3-element numpy arrays
    #     Each array is of the form [x,y,theta]
    # Create a LineFollower object
    # YOUR CODE HERE
    # rostopic info /planner_node/car_plan Type: geometry_msgs/PoseArray
    print('start to load plan from topic: ', plan_topic)
    converted_plan = []
    for msg in rospy.wait_for_message('/planner_node/car_plan', PoseArray).poses:
        converted_plan.append([msg.position.x, msg.position.y, utils.quaternion_to_angle(msg.orientation)])

    # Create a LineFollower object
    LineFollower(converted_plan, pose_topic, plan_lookahead, translation_weight, rotation_weight, kp, ki, kd,
                 error_buff_length, speed)

    rospy.spin()  # Prevents node from shutting down

if __name__ == '__main__':
    main()
