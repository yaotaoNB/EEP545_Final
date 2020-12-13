#!/usr/bin/env python

import rospy 
import numpy as np
from threading import Lock
import sys 

from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, PoseArray
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from lab5.srv import *

from HaltonPlanner import HaltonPlanner
from HaltonEnvironment import HaltonEnvironment
import GraphGenerator
import Utils

import csv
import os
from pathlib import Path

class PlannerNode(object):

  def __init__(self, map_service_name, 
                     halton_points, 
                     disc_radius,
                     collision_delta,                      
                     pub_topic,
                     car_width,
                     car_length,
                     pose_arr,
                     start_waypoint_topic, #for visualization in rviz
                     good_waypoint_topic, #for visualization in rviz
                     bad_waypoint_topic, #for visualization in rviz
                     start_waypoint_pose, #for visualization in rviz
                     good_waypoint_pose, #for visualization in rviz
                     bad_waypoint_pose #for visualization in rviz
                     ):
    
    print("[Planner Node] Getting map from service...")
    rospy.wait_for_service(map_service_name)
    self.map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
    print("[Planner Node] ...got map")
    
    print("[Planner Node] Generating graph file...")
    graph_file = GraphGenerator.generate_graph_file(self.map_msg, halton_points, disc_radius, car_width, car_length, collision_delta)
    print("[Planner Node] ..graph generated")
    
    self.environment = HaltonEnvironment(self.map_msg, graph_file, None, None, car_width, car_length, disc_radius, collision_delta)
    self.planner = HaltonPlanner(self.environment)
    
    self.source_yaw = None #0
    self.target_yaw = None #0
    
    self.cur_plan = None
    self.plan_lock = Lock()

    self.pose_arr = pose_arr
    
    self.orientation_window_size = 21

    self.start_waypoint_pose = start_waypoint_pose
    self.good_waypoint_pose = good_waypoint_pose
    self.bad_waypoint_pose = bad_waypoint_pose
    #waypoints visualization purpose
    self.start_waypoint_pub = rospy.Publisher(start_waypoint_topic, Marker, queue_size=100)  
    self.good_waypoint_pub = rospy.Publisher(good_waypoint_topic, MarkerArray, queue_size=100)  
    self.bad_waypoint_pub = rospy.Publisher(bad_waypoint_topic, MarkerArray, queue_size=100) 

    print('pub topic: ', pub_topic)
    if pub_topic is not None:
      self.plan_pub = rospy.Publisher(pub_topic, PoseArray, queue_size=1)          
    else:
      self.plan_pub = None
    print '[Planner Node] Ready to plan'
    
  #green = start, blue = good points, red = bad points
  def publish_waypoints_viz(self): 
    green_color = ColorRGBA()
    green_color.r = 0.0
    green_color.g = 1.0 #or 1.0
    green_color.b = 0.0
    green_color.a = 1.0

    blue_color = ColorRGBA()
    blue_color.r = 0
    blue_color.g = 0 
    blue_color.b = 1.0
    blue_color.a = 1.0

    red_color = ColorRGBA()
    red_color.r = 1.0
    red_color.g = 0 
    red_color.b = 0
    red_color.a = 1.0

    w_id = 0

    p_start = Marker()
    p_good = MarkerArray()
    p_bad = MarkerArray()
    p_start.header.frame_id = "map"

    #start: is a single waypoint
    config = self.start_waypoint_pose[:]

    p_start.ns = 'start_waypoint'
    p_start.id = w_id
    w_id += 1
    p_start.header.stamp = rospy.Time()
    p_start.action = p_start.ADD
    p_start.scale.x = 0.35
    p_start.scale.y = 0.35
    p_start.scale.z = 0.1

    p_start.pose.position.x = config[0]
    p_start.pose.position.y = config[1]
    p_start.pose.position.z = 0.0
    p_start.type = p_start.SPHERE
    p_start.color = green_color
    p_start.pose.orientation = Utils.angle_to_quaternion(0.0) 
    
    rospy.sleep(0.5) 

    self.start_waypoint_pub.publish(p_start) 

    #good points: an array of waypoints
    for i in xrange(len(self.good_waypoint_pose)):
      config = self.good_waypoint_pose[i]
      marker = Marker()

      marker.ns = 'good_waypoint'
      marker.id = w_id
      w_id += 1
      marker.header.stamp = rospy.Time()
      marker.action = marker.ADD
      marker.scale.x = 0.35
      marker.scale.y = 0.35
      marker.scale.z = 0.1

      marker.header.frame_id = "map"
      marker.pose.position.x = config[0]
      marker.pose.position.y = config[1]
      marker.pose.position.z = 0.0
      marker.type = marker.SPHERE
      marker.color = blue_color
      marker.pose.orientation = Utils.angle_to_quaternion(0.0)
      p_good.markers.append(marker)

    rospy.sleep(0.5) 
    self.good_waypoint_pub.publish(p_good) 

    #bad points
    for i in xrange(len(self.bad_waypoint_pose)):
      config = self.bad_waypoint_pose[i]
      marker = Marker()

      marker.ns = 'bad_waypoint'
      marker.id = w_id
      w_id += 1
      marker.header.stamp = rospy.Time()
      marker.action = marker.ADD
      marker.scale.x = 0.35
      marker.scale.y = 0.35
      marker.scale.z = 0.1

      marker.header.frame_id = "map"
      marker.pose.position.x = config[0]
      marker.pose.position.y = config[1]
      marker.pose.position.z = 0.0
      marker.type = marker.SPHERE
      marker.color = red_color
      marker.pose.orientation = Utils.angle_to_quaternion(0.0)
      p_bad.markers.append(marker)

    rospy.sleep(0.5) 
    self.bad_waypoint_pub.publish(p_bad)

  def publish_plan(self, plan):
    pa = PoseArray()
    pa.header.frame_id = "/map"
    for i in xrange(len(plan)):
      config = plan[i]
      pose = Pose()
      pose.position.x = config[0]
      pose.position.y = config[1]
      pose.position.z = 0.0
      pose.orientation = Utils.angle_to_quaternion(config[2])
      pa.poses.append(pose)
    self.plan_pub.publish(pa) 

      
  def add_orientation(self, plan):
    plan = np.array(plan)

    oriented_plan = np.zeros((plan.shape[0],3),dtype=np.float)
    oriented_plan[:,0:2] = plan[:,:]
 
    if plan.shape[0] >= 2:  
      oriented_plan[0,2] = self.source_yaw
      oriented_plan[oriented_plan.shape[0]-1, 2] = self.target_yaw   
          
      plan_diffs = np.zeros(plan.shape, np.float)
      plan_diffs[0:plan_diffs.shape[0]-1] = plan[1:plan.shape[0]]-plan[0:plan.shape[0]-1]
      plan_diffs[plan_diffs.shape[0]-1] = np.array([np.cos(self.target_yaw), np.sin(self.target_yaw)], dtype=np.float) 
    
      avg_diffs = np.empty(plan_diffs.shape, dtype=np.float)
      for i in xrange(plan_diffs.shape[0]):
        avg_diffs[i] = np.mean(plan_diffs[np.max((0,i-self.orientation_window_size/2)):
                                          np.min((plan_diffs.shape[0]-1, i+self.orientation_window_size/2+1))],
                               axis=0)

      oriented_plan[1:oriented_plan.shape[0]-1,2] = np.arctan2(avg_diffs[1:oriented_plan.shape[0]-1,1],
                                                               avg_diffs[1:oriented_plan.shape[0]-1,0])
   
    elif plan.shape[0] == 2:
      oriented_plan[:,2] = np.arctan2(plan[1,1]-plan[0,1],plan[1,0]-plan[0,0])

    return oriented_plan
  
  def final_plan(self):
    final_plan = []
    for i in range(0, len(self.pose_arr)-1):
      source_pose = self.pose_arr[i][0:2]
      target_pose = self.pose_arr[i+1][0:2]

      if(np.abs(source_pose-target_pose).sum() < sys.float_info.epsilon):
        print '[Planner Node] Source and target are the same, will not plan'
        return

      if not self.environment.manager.get_state_validity(source_pose):
        print '[Planner Node] Source in collision, will not plan'
        return

      if not self.environment.manager.get_state_validity(target_pose):
        print '[Planner Node] Target in collision, will not plan'
        return

      print('[Planner Node] Inserting source and target, source index: ', i, '; target index: ', i+1, 'pose_arr len: ', len(pose_arr))
      self.environment.set_source_and_target(source_pose, target_pose)

      print '[Planner Node] Computing plan...'
      temp_plan = self.planner.plan()    
      
      if temp_plan is not None:
        temp_plan = self.planner.post_process(temp_plan, 5)
        if self.source_yaw is None or self.target_yaw is None:
          self.source_yaw = 0
          self.target_yaw = 0
        temp_plan = self.add_orientation(temp_plan)
        self.source_yaw = temp_plan[-1][2]
        self.target_yaw = temp_plan[-1][2]
        print('[Planner Node] plan segment at index: ', i, ' and index ', i+1 ,'is finished, start to compute the path for the next segment')
      else:
        print('[Planner Node] plan segment at index: ', i, ' could not compute a plan')

      final_plan.append(temp_plan)

    plan_list = []
    for plan in final_plan:
      for i in range(1,len(plan)-3): #get rid of the 1st and the last few elements from each plan segment to avoid sharp orientation difference
        plan_list.append(plan[i][:])

    self.cur_plan = np.array(plan_list)

    if (self.cur_plan is not None) and (self.plan_pub is not None): #this step is only for visualization of the final plan in rviz
      rospy.sleep(0.5) 
      print('[Planner Node] the whole plan is complete, publish plan to pub_topic')
      self.publish_plan(self.cur_plan) 

#take a csv file path and return a list of waypoints
def get_waypoint(path):
  dir_path = os.path.dirname(os.path.realpath(__file__))
  dir_path = str(Path(dir_path).parent)
  csv_path = dir_path + path
  pose = []
  with open(csv_path) as csv_file: 
    csv_reader = csv.reader(csv_file, delimiter=',')
    count = 0;
    for row in csv_reader:
      if count == 0:
        count += 1
        continue
      else:
        new_pose = [float(row[0]), float(row[1]), 0.0]
        pose.append(new_pose)
  return pose

# a or b are np.array with 2 elements like this: [x, y]
def L2dist(a, b): #return L2 distance between a and b
  pa = np.array(a)
  pb = np.array(b)
  return np.linalg.norm(pa-pb)

def waypoint_map2world(waypoints,mapinfo):
  world_waypoints = []
  for i in waypoints:
    world_waypoints.append(Utils.map_to_world(i,mapinfo))
  return world_waypoints

#start_point and good_points are initially python list
def get_pose_arr(start_point, good_points): 
  pose_arr = []
  pose_arr.append(start_point)
  pool = good_points[:]
  cur = start_point[:]
  while pool: #brute force search keep finding the next nearest point until pool is empty
    mindist = 99999.99 #the current min dist
    minidx = 0
    for i in range(len(pool)):
      dist = min(L2dist(cur, pool[i]), mindist)
      if dist < mindist:
        minidx = i
        mindist = dist
    cur = pool[minidx]
    pose_arr.append(pool[minidx])
    del pool[minidx]
  return np.array(pose_arr) #not sure about numpy to list or vice versa conversion yet but it depends on if planner takes np.array or a list, finish this later

if __name__ == '__main__':
  rospy.init_node('planner_node', anonymous=True)
  
  map_service_name = rospy.get_param("~static_map", "static_map")
  halton_points = rospy.get_param("~halton_points", 500)
  disc_radius = rospy.get_param("~disc_radius", 3)
  collision_delta = rospy.get_param("~collision_delta", 0.13)  
  pub_topic = rospy.get_param("~pub_topic", "planner_node/car_plan") ##for ma_server.launch viz planning path only, change this topic to "/PlannerNode/car_plan"
  car_width = rospy.get_param("/car_kinematics/car_width", 0.33)
  car_length = rospy.get_param("/car_kinematics/car_length", 0.33)

  csv_start = '/waypoints/real_car/start.csv'
  csv_good = '/waypoints/real_car/good_waypoints.csv'
  csv_bad = '/waypoints/real_car/bad_waypoints.csv'

  mapinfo = rospy.ServiceProxy(map_service_name, GetMap)().map.info
  start_point = get_waypoint(csv_start)
  start_point = waypoint_map2world(start_point, mapinfo)
  start_point = start_point[0][:]

  good_points = get_waypoint(csv_good)
  good_points = waypoint_map2world(good_points, mapinfo)

  bad_points = get_waypoint(csv_bad)
  bad_points = waypoint_map2world(bad_points, mapinfo)

  start_waypoint_topic = "/waypoint/start"
  good_waypoint_topic = "/waypoint/good"
  bad_waypoint_topic = "/waypoint/bad"

  pose_arr = get_pose_arr(start_point, good_points)

  pn = PlannerNode(map_service_name, 
                   halton_points, 
                   disc_radius,
                   collision_delta,                    
                   pub_topic,
                   car_width,
                   car_length,
                   pose_arr,
                   start_waypoint_topic,
                   good_waypoint_topic,
                   bad_waypoint_topic,
                   start_point,
                   good_points,
                   bad_points)

  pn.publish_waypoints_viz()
                   
  if pub_topic is not None:
    pn.plan_lock.acquire()
    pn.final_plan()
    pn.plan_lock.release()

  while not rospy.is_shutdown():
    raw_input("Press Enter to publish car_plan...")
    pn.publish_plan(pn.cur_plan)
    rospy.sleep(1.0) 
