#!/usr/bin/env python

import rospy
import numpy as np
from threading import Lock

'''
  Provides methods for re-sampling from a distribution represented by weighted samples
'''
class ReSampler:

  '''
    Initializes the resampler
    particles: The particles to sample from
    weights: The weights of each particle
    state_lock: Controls access to particles and weights
  '''
  def __init__(self, particles, weights, state_lock=None):
    self.particles = particles 
    self.weights = weights
    
    # For speed purposes, you may wish to add additional member variable(s) that 
    # cache computations that will be reused in the re-sampling functions
    # YOUR CODE HERE?
    
    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
  
  '''
    Performs independently, identically distributed in-place sampling of particles
  '''
  def resample_naiive(self):
    self.state_lock.acquire()   
   
    # YOUR CODE HERE
    ptc_len = self.particles.shape[0] #length of particles/weights array
    sel_idx = np.random.choice(np.arange(ptc_len), size = ptc_len, p=self.weights) #selected indices
    self.particles[:] = self.particles[sel_idx, :]
    # set all weights to uniform so that bigger weights won't keep getting bigger (sensor model will take these weights and re-measure at next itr)
    self.weights[:] = 1.0 / self.particles.shape[0] 
    
    self.state_lock.release()
  
  '''
    Performs in-place, lower variance sampling of particles
    (As discussed on pg 86 of Probabilistic Robotics)
  '''
  def resample_low_variance(self):
    self.state_lock.acquire()
    
    # YOUR CODE HERE
    size = self.particles.shape[0]
    avg = 1.0 / size
    w_sum = np.cumsum(self.weights) #weights distribution
    ptrs = np.arange(size) * avg + np.random.uniform(0.0,avg) #all ptr pose with a ptr init bias for all elements
    sel_ptcl = np.zeros_like(self.particles) #selected particles based on weight dist

    w_idx = 0
    ptcl_idx = 0

    while ptcl_idx < size:
      if ptrs[ptcl_idx] < w_sum[w_idx]:
        sel_ptcl[ptcl_idx] = self.particles[w_idx]
        ptcl_idx += 1
      else:
        w_idx += 1

    self.particles[:] = sel_ptcl[:] 
    # set all weights to uniform so that bigger weights won't keep getting bigger (sensor model will take these weights and re-measure at next itr)
    self.weights[:] = avg
    
    self.state_lock.release()
    
import matplotlib.pyplot as plt

if __name__ == '__main__':

  rospy.init_node("sensor_model", anonymous=True) # Initialize the node

  n_particles = int(rospy.get_param("~n_particles",100)) # The number of particles    
  k_val = int(rospy.get_param("~k_val", 80)) # Number of particles that have non-zero weight
  resample_type = rospy.get_param("~resample_type", "naiive") # Whether to use naiive or low variance sampling
  trials = int(rospy.get_param("~trials", 10)) # The number of re-samplings to do
  
  histogram = np.zeros(n_particles, dtype=np.float) # Keeps track of how many times
                                                    # each particle has been sampled
                                                    # across trials
  
  for i in xrange(trials):
    particles = np.repeat(np.arange(n_particles)[:,np.newaxis],3, axis=1) # Create a set of particles
                                                                          # Here their value encodes their index
    # Have increasing weights up until index k_val
    weights = np.arange(n_particles, dtype=np.float)
    weights[k_val:] = 0.0
    weights[:] = weights[:] / np.sum(weights)
    
    rs = ReSampler(particles, weights) # Create the Resampler
  
    # Resample
    if resample_type == "naiive":
      rs.resample_naiive()
    elif resample_type == "low_variance":
      rs.resample_low_variance()
    else:
      print "Unrecognized resampling method: "+ resample_type     

    # Add the number times each particle was sampled    
    for j in xrange(particles.shape[0]):
      histogram[particles[j,0]] = histogram[particles[j,0]] + 1
    
  # Display as histogram
  plt.bar(np.arange(n_particles), histogram)
  plt.xlabel('Particle Idx')
  plt.ylabel('# Of Times Sampled')
  plt.show()    
