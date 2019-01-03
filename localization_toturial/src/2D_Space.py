# The function localize takes the following arguments:
#
# colors:
#        2D list, each entry either 'R' (for red cell) or 'G' (for green cell)
#
# measurements:
#        list of measurements taken by the robot, each entry either 'R' or 'G'
#
# motions:
#        list of actions taken by the robot, each entry of the form [dy,dx],
#        where dx refers to the change in the x-direction (positive meaning
#        movement to the right) and dy refers to the change in the y-direction
#        (positive meaning movement downward)
#        NOTE: the *first* coordinate is change in y; the *second* coordinate is
#              change in x
#
# sensor_right:
#        float between 0 and 1, giving the probability that any given
#        measurement is correct; the probability that the measurement is
#        incorrect is 1-sensor_right
#
# p_move:
#        float between 0 and 1, giving the probability that any given movement
#        command takes place; the probability that the movement command fails
#        (and the robot remains still) is 1-p_move; the robot will NOT overshoot
#        its destination in this exercise
#
# The function should RETURN (not just show or print) a 2D list (of the same
# dimensions as colors) that gives the probabilities that the robot occupies
# each cell in the world.
#
# Compute the probabilities by assuming the robot initially has a uniform
# probability of being in any cell.
#
# Also assume that at each step, the robot:
# 1) first makes a movement,
# 2) then takes a measurement.
#
# Motion:
#  [0,0] - stay
#  [0,1] - right
#  [0,-1] - left
#  [1,0] - down
#  [-1,0] - up

import numpy as np

def localize(colors,measurements,motions,sensor_right,p_move):
    # initializes p to a uniform distribution over a grid of the same dimensions as colors
    pinit = 1.0 / float(len(colors)) / float(len(colors[0]))
    p = [[pinit for row in range(len(colors[0]))] for col in range(len(colors))]
    # >>> Insert your code here <<<
    for k in range(len(measurements)):
        p = move(p, motions[k], p_move)
        p = sense(colors, p, measurements[k], sensor_right)
    return p

def sense (colors, p, measurement, sensor_right):
    q = p
    #Update Belief
    s = 0.0
    for i, row in enumerate(p):
        for j, item in enumerate(row):
            if measurement == colors[i][j]:
                q[i][j] = p[i][j] * sensor_right
            else:
                q[i][j] = p[i][j] * (1 - sensor_right)
            s += q[i][j]
    #normalization
    q[:] = [[ele / s for ele in sub] for sub in q]

    return q

def move(p, motion, p_move):
    #p_move suggest success rate of the movement
    q = [[0.00 for row in range(len(p[0]))] for col in range(len(p))]
    for i in range(len(p)):
        for j in range(len(p[0])):
            q[i][j] = p[(i - motion[0]) % len(p)][(j - motion[1]) % len(p[i])] * p_move + p[i][j] * (1 - p_move)
    return q

def move2(p, motion, p_move): # alternative method
    if(motion[0] == 0 and motion[1] == 0):
        return p
    q = p
    if(motion[1] != 0):
        q = np.roll(q, motion[1], axis=1) #left & right shift
    else:
        q = np.roll(q, motion[0], axis=0) #up & down shift
    q = np.array(q)
    p = np.array(p)
    q = q * p_move + p * (1 - p_move)
    return q 

def show(p):
    rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
    print '[' + ',\n '.join(rows) + ']'

#Driver Function

colors = [['G', 'G', 'G'],
          ['G', 'R', 'R'],
          ['G', 'G', 'G']]
measurements = ['R', 'R']
motions = [[0,0], [0,1]]
sensor_right = 1.0
p_move = 1.0
p = localize(colors,measurements,motions,sensor_right,p_move)

show(p) # displays your answer
