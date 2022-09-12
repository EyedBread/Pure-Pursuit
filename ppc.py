import math

import time
import numpy as np
from shapely.geometry import LineString
from shapely.geometry import Point
import matplotlib.pyplot as plt
"""
Pure pursuit controller using a kinematic bicycle model, where the reference point is on the center of the rear axle. Not implemented with ROS currently.
"""

length = 1 #Bicycle length
v = 2 #Velocity is assumed constant in this model
lookahead = 1 #Look-ahead distance
waypoints = [[4,3], [7,0], [9,3], [7,5], [4,3.01]] #In order to create a closed circuit, the last waypoint has to differ a little bit from first waypoint
startPos = [4,3] #Starting position for car
startAngle = 0  #Starting angle for car
passed = [] #Waypoints which the lookahead has passed and are no longer relevant for the current loop
dt = 0.1 #time_dot

def update(theta,delta):
    """ 
    Update the vehicle state according to the bicycle kinematic model.
    Reference point is the rear axle on the bicycle.
        Inputs: 
    theta,delta where theta is heading angle(position the vehicle is pointing at the given moment),
    and delta is steering angle.
    
    @return: state change rate: [x_dot, y_dot, theta_dot]
    """

    x_dot = v*math.cos(theta)*dt

    y_dot = v*math.sin(theta)*dt
    theta_dot = dt*v*math.tan(delta)/length

    return [x_dot, y_dot, theta_dot]


def point_on_line(a, b, p):
    """
    Function taken from stack: https://stackoverflow.com/questions/61341712/calculate-projected-point-location-x-y-on-given-line-startx-y-endx-y
    Function that returns the projected point p on line a-b
    """
    ap = p - a
    ab = b - a
    result = a + np.dot(ap, ab) / np.dot(ab, ab) * ab
    return result

def get_angle(p0, p1=np.array([0,0]), p2=None): 
    """
    Function taken from stack (Can't find link)
    compute angle (in degrees) for p0p1p2 corner
    Inputs:
        p0,p1,p2 - points in the form of [x,y]
    """
    if p2 is None:
        p2 = p1 + np.array([1, 0])
    v0 = np.array(p0) - np.array(p1)
    v1 = np.array(p2) - np.array(p1)

    angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
    return np.degrees(angle)

#theta = angle of the bikes forwards direction with respect to x-axis
def get_steering_angle(currentPos, goalPos, theta):
    """
    returns the steering angle to reach the designated lookahead given our current pose (position and angle theta) and the position of the lookahead
    """
    alpha = math.atan2(goalPos[1] - currentPos[1],goalPos[0] - currentPos[0]) - theta
    steering_angle = math.atan2(2*length*math.sin(alpha)/lookahead, 1.0)

    return steering_angle


def get_goalPos(currentPos, lookahead):
    """
    Function that returns the target position of the lookahead.
    """

    less_than_lookahead = [] 
    for waypoint in waypoints:
        if waypoint not in passed: #If we have not passed the waypoint
            delta_x = currentPos[0] - waypoint[0]
            delta_y = currentPos[1] - waypoint[1]
            hypothenuse = math.hypot(delta_x, delta_y)
            if hypothenuse < lookahead: #if the distance between currentPos and current waypoint is smaller than the length of lookahead, append waypoint to less_than_lookahead
                less_than_lookahead.append(waypoint) 
            else: 
                """
                When we encounter the first waypoint where the distance is greater than our lookahead.
                This means that the target point where the lookahead looks at should be on the linear interpolation between current waypoint and the previous waypoint, and the interpolated line starts at our current positions projection on the interpolated line.
                """
                p = Point(currentPos[0], currentPos[1])
                c = p.buffer(lookahead).boundary #Circle with center p and radius lookahead
                if len(less_than_lookahead) == 0 and waypoints.index(waypoint) == 0: #If this is our first waypoint and it is further away than our lookahead, our current position will then be the start of the line
                    startLine = currentPos
                else:
                    if len(less_than_lookahead) >= 1: #all waypoints that have a shorter distance than the lookahead are added to passed list, our lookahead has passed them
                        for index in range(len(less_than_lookahead)):
                            passed.append(less_than_lookahead[index]) 
                    A = np.array(waypoints[waypoints.index(waypoint)-1]) 
                    B = np.array(waypoint)
                    P = np.array(currentPos) 
                    startLine = point_on_line(A,B,P) #Project point P onto the line A-B
                endLine = waypoint
                l = LineString([startLine, endLine])
                goalPos = c.intersection(l) #Find the intersection between the interpolated line and the circle
                return  goalPos.coords[0]
    del passed[:] #Clear passed list when all waypoints have been visited
    return waypoints[0]

def main():
    #Initialise all starting values
    racing = True
    time = 0
    currentPos = startPos
    currentAngle = startAngle
    goalPos = get_goalPos(currentPos, lookahead)
    steering_angle = get_steering_angle(currentPos, goalPos, currentAngle)
    state = [currentPos[0],currentPos[1], currentAngle]
    while racing:

        state_dot = update(state[2], steering_angle) 

        next_state = [sum(value) for value in zip(state, state_dot)] #next_state = state + state_dot

        if time >= 100: #Total time limit exceeded, end program
            racing = False

        state = next_state
        currentPos[0], currentPos[1], theta = state 
        goalPos = get_goalPos(currentPos, lookahead)
        steering_angle = get_steering_angle(currentPos, goalPos, theta)

        #Plotting
        plt.clf()
        x_val = [x[0] for x in waypoints]
        y_val = [x[1] for x in waypoints]
        plt.plot(x_val, y_val, "-r", label="course")
        plt.plot(currentPos[0], currentPos[1], 'bo', label="Vehicle")
        plt.plot(goalPos[0], goalPos[1], 'go', label="Lookahead")
        plt.axis("equal")
        plt.grid(True)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.legend()
        plt.pause(0.001)

        time += dt #increment the total time
main()

        

        

