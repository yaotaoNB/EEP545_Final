import cv2
import math
import numpy
import Utils
import csv
import os
from pathlib import Path

BAD_WAYPOINTS = '/waypoints/real_car/bad_waypoints.csv'

class ObstacleManager(object):

  def __init__(self, mapMsg, car_width, car_length, collision_delta):
    self.map_info = mapMsg.info
    self.mapImageGS = numpy.array(mapMsg.data, dtype=numpy.uint8).reshape(
      (mapMsg.info.height, mapMsg.info.width, 1))

    # Retrieve the map dimensions
    height, width, channels = self.mapImageGS.shape
    self.mapHeight = height
    self.mapWidth = width
    self.mapChannels = channels

    # Binarize the Image
    self.mapImageBW = 255 * numpy.ones_like(self.mapImageGS, dtype=numpy.uint8)
    self.mapImageBW[self.mapImageGS == 0] = 0

    # Obtain the car length and width in pixels
    self.robotWidth = int(car_width / self.map_info.resolution + 0.5)
    self.robotLength = int(car_length / self.map_info.resolution + 0.5)
    self.collision_delta = collision_delta

    box_length = numpy.sqrt(self.robotLength**2 + self.robotWidth**2)
    self.box = numpy.array([int(box_length), int(box_length)])

    dir_path = os.path.dirname(os.path.realpath(__file__))
    dir_path = str(Path(dir_path).parent)
    csv_path = dir_path + BAD_WAYPOINTS
    with open(csv_path) as csv_file: 
      csv_reader = csv.reader(csv_file, delimiter=',')
      count = 0;
      for row in csv_reader:
        if count == 0:
          count += 1
          continue
        else:
          bad_x = int(row[0]) #or float???
          bad_y = int(self.mapHeight - int(row[1])) #or float???
          self.mapImageBW[2 * bad_y - (self.box[0] / 2):(self.box[0] / 2), 2 * bad_x - (self.box[1] / 2):(self.box[1] / 2), 0] = 255

  # Check if the passed config is in collision
  # config: The configuration to check (in meters and radians)
  # Returns False if in collision, True if not in collision
  def get_state_validity(self, config):

    # Convert the configuration to map-coordinates -> mapConfig is in pixel-space
    mapConfig = Utils.world_to_map(config, self.map_info)
    

    # ---------------------------------------------------------
    # YOUR CODE HERE
    #
    # Return true or false based on whether the robot's configuration is in collision
    # Use a square to represent the robot, return true only when all points within
    # the square are collision free
    #
    # Also return false if the robot is out of bounds of the map
    #
    # Although our configuration includes rotation, assume that the
    # square representing the robot is always aligned with the coordinate axes of the
    # map for simplicity
    # ----------------------------------------------------------
    x = mapConfig[0]
    y = mapConfig[1]
    half_length = int(((self.robotWidth**2 + self.robotLength**2) / 2.0)**0.5)
    
    left_topy = y - half_length
    left_topx = x - half_length
    right_botx = x + half_length
    right_boty = y + half_length
    
    if left_topx >= self.mapWidth or right_botx >= self.mapWidth or left_topy >= self.mapHeight or right_boty >= self.mapHeight:
      return False
    elif self.mapImageBW[left_topy : right_boty, left_topx : right_botx].sum():
      return False
    
    return True

  # Discretize the path into N configurations, where N = path_length / self.collision_delta
  #
  # input: an edge represented by the start and end configurations
  #
  # return three variables:
  # list_x - a list of x values of all intermediate points in the path
  # list_y - a list of y values of all intermediate points in the path
  # edgeLength - The euclidean distance between config1 and config2
  def discretize_edge(self, config1, config2):
    list_x, list_y = [], []
    edgeLength = 0
    # # -----------------------------------------------------------
    # # YOUR CODE HERE
    dif = numpy.array(config1) - numpy.array(config2)
    edgeLength = (dif[0]**2 + dif[1]**2)**0.5
    
    if edgeLength > 0:
      theta = (numpy.array(config2) - numpy.array(config1)) / edgeLength
      N = int(edgeLength / self.collision_delta)
      intpoint = numpy.array(config1) + numpy.linspace(0, edgeLength, N + 1)[:, numpy.newaxis] * theta[numpy.newaxis, :]
      list_x = intpoint[:, 0].tolist()
      list_y = intpoint[:, 1].tolist()
    else:
      intpoint = numpy.array([config1])
      list_x = intpoint[:, 0].tolist()
      list_y = intpoint[:, 1].tolist()
    
    return list_x, list_y, edgeLength


  # Check if there is an unobstructed edge between the passed configs
  # config1, config2: The configurations to check (in meters and radians)
  # Returns false if obstructed edge, True otherwise
  def get_edge_validity(self, config1, config2):
    # -----------------------------------------------------------
    # YOUR CODE HERE
    #
    # Check if endpoints are obstructed, if either is, return false
    # Find path between two configs by connecting them with a straight line
    # Discretize the path with the discretized_edge function above
    # Check if all configurations along path are obstructed
    if not self.get_state_validity(config1) or not self.get_state_validity(config2):
      return False
    else:
      list_x, list_y, _ = self.discretize_edge(config1, config2)
      size = len(list_x)
      for j in range(size) :
        if not self.get_state_validity([list_x[j], list_y[j]]):
          return False
        
    return True

# Write Your Test Code For Debugging
#if __name__ == '__main__':
# return
  # Write test code here!
