#!/usr/bin/env python

import collections
import sys

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped, PoseWithCovariance 
from ackermann_msgs.msg import AckermannDriveStamped

import Utils
import math

# The topic to publish control commands to
PUB_TOPIC = '/car/mux/ackermann_cmd_mux/input/navigation'
initialpose_topic = '/initialpose'
'''
Follows a given plan using constant velocity and PID control of the steering angle
'''

# a or b are np.array with 2 elements like this: [x, y]
def L2dist(a, b): #return L2/eucleadian distance between a and b
  pa = np.array(a)
  pb = np.array(b)
  return np.linalg.norm(pa-pb)

# class to ignore error messages
class DevNull:
  def write(self, msg):
    pass

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
      while len(self.plan) > 0:
        rot = np.array(np.matmul(Utils.rotation_matrix(np.pi/2 - cur_pose[2]), [[self.plan[0][0] - cur_pose[0]],[self.plan[0][1] - cur_pose[1]]]))
        if (self.plan[0][0] - cur_pose[0]) < 0:  
          if math.sqrt((self.plan[1][0] - cur_pose[0])**2 + (self.plan[1][1] - cur_pose[1])**2) < math.sqrt((self.plan[0][0] - cur_pose[0])**2 + (self.plan[0][1] - cur_pose[1])**2) :    
            self.plan.pop(0)
          else:
            break
        else:
          if 0 > rot[1]:
            self.plan.pop(0)
          else:
            break

      goal_idx = min(0+self.plan_lookahead, len(self.plan)-1)

      if len(self.plan) <= 0 or goal_idx < 0:
        return (False, 0.0)
      translation_error = -np.array(np.matmul(Utils.rotation_matrix(np.pi/2 - cur_pose[2]), np.array([[self.plan[int(goal_idx)][0] - cur_pose[0]], [self.plan[int(goal_idx)][1] - cur_pose[1]]])))[0]
      if self.plan[int(goal_idx)][2] < 0 and cur_pose[2] > 0:
        rotation_error = (np.pi - abs(self.plan[int(goal_idx)][2])) + (np.pi - cur_pose[2])
      elif self.plan[int(goal_idx)][2] > 0 and cur_pose[2] < 0:
        rotation_error = -1 * ((np.pi - abs(cur_pose[2])) + (np.pi - self.plan[int(goal_idx)][2]))
      else:
        rotation_error = self.plan[int(goal_idx)][2] - cur_pose[2]

      return True, (self.translation_weight * translation_error + self.rotation_weight * rotation_error)

    '''
    Uses a PID control policy to generate a steering angle from the passed error
      error: The current error
    Returns: The steering angle that should be executed
    '''

    def compute_steering_angle(self, error):
      self.error_buff.append((error, rospy.Time.now().to_sec()))
      return self.kp * error + self.ki * (0.5 * (error**2)) + self.kd * (error - self.error_buff[0][0])

    def pose_cb(self, msg):
      if len(self.plan) <= 0: #and self.plt_finished is False:
        # self.write2csv()
        # print("err_hist wrote to csv.")
        # self.plt_finished = True
        pass
      elif len(self.plan) > 0:
        cur_pose = np.array([msg.pose.position.x,
                             msg.pose.position.y,
                             Utils.quaternion_to_angle(msg.pose.orientation)])
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

    '''
    Callback for the current pose of the car
      msg: A PoseStamped representing the current pose of the car
      This is the exact callback that we used in our solution, but feel free to change it
    '''

    def write2csv(self):
      dir = "/home/robot/catkin_ws/src/lab3/err_csv/"
      filename = 'Kp: '+ str(self.kp)+'; Ki: '+str(self.ki)+'; Kd: '+str(self.kd)+'; trans w: '+str(round(self.translation_weight,2))+'; rot w: '+str(round(self.rotation_weight,2))+".csv"
      np.savetxt(dir+filename, np.asarray(self.err_hist), delimiter=",")


def main():
  # sys.stderr = DevNull() #ignore error messages

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

  # print('set the car initialpose to the start waypoint')
  raw_input("Press Enter to set initialpose...")  # Waits for ENTER key press
  initialpose_pub = rospy.Publisher(initialpose_topic, PoseWithCovarianceStamped, 
                                       queue_size=10)
  pcs = PoseWithCovarianceStamped()
  pcs.header.frame_id = "map"
  pcs.header.stamp = rospy.Time()
  pcs.pose.pose.position.x = 49.9
  pcs.pose.pose.position.y = 11.938
  pcs.pose.pose.position.z = 0.0
  pcs.pose.pose.orientation.x = 0.0
  pcs.pose.pose.orientation.y = 0.0
  pcs.pose.pose.orientation.z = -0.105
  pcs.pose.pose.orientation.w = 0.995
  rospy.sleep(1.0) 
  initialpose_pub.publish(pcs)
  # Use rospy.wait_for_message to get the plan msg
  # Convert the plan msg to a list of 3-element numpy arrays
  #     Each array is of the form [x,y,theta]
  # Create a LineFollower object
  # YOUR CODE HERE
  # rostopic info /planner_node/car_plan Type: geometry_msgs/PoseArray
  print('waiting for the plan to be published from PlannerNode, plan_topic:  ', plan_topic)
  converted_plan = []
  for msg in rospy.wait_for_message('/planner_node/car_plan', PoseArray).poses:
    converted_plan.append([msg.position.x, msg.position.y, Utils.quaternion_to_angle(msg.orientation)])

  print('plan is received, car starts to drive along the plan...')
  # Create a LineFollower object
  LineFollower(converted_plan, pose_topic, plan_lookahead, translation_weight, rotation_weight, kp, ki, kd,
               error_buff_length, speed)

  rospy.spin()  # Prevents node from shutting down

if __name__ == '__main__':
  main()
