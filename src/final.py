#!/usr/bin/env python

# this is the main file which will get a plan path from planner and let the bot follow the path via PID and particle filter, please see project proposal for more detail

#utility functions
def bag_file_handler():
	# ...

car2bad_treshold = 0.13 # the minimum allowed distance from car to a bad point
def car_dist2bad(): #this function measure the distance of the car to each bad way point frame by frame, if one of them is smaller than car2bad_treshold and error will be generated
	# ...

if __name__ == '__main__':
	# ...