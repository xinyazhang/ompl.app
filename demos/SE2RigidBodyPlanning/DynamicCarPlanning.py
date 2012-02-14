#!/usr/bin/env python

######################################################################
# Rice University Software Distribution License
#
# Copyright (c) 2010, Rice University
# All Rights Reserved.
#
# For a full description see the file named LICENSE.
#
######################################################################

# Author: Mark Moll

import sys
from os.path import abspath, dirname, join
from math import pi

ompl_app_root = dirname(dirname(dirname(abspath(__file__))))

try:
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import control as oc
    from ompl import app as oa
except:
    sys.path.insert(0, join(ompl_app_root, 'ompl/py-bindings' ) )
    from ompl import base as ob
    from ompl import geometric as og
    from ompl import control as oc
    from ompl import app as oa

def dynamicCarDemo(setup):
    print "\n\n***** Planning for a %s *****\n" % setup.getName()
    # plan for dynamic car in SE(2)
    stateSpace = setup.getStateSpace()

    # set the bounds for the R^2 part of SE(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-10)
    bounds.setHigh(10)
    stateSpace.getSubSpace(0).setBounds(bounds)

    # define start state
    start = ob.State(stateSpace)
    start[0] = start[1] = start[2] = start[3] = start[4] = 0.

    # define goal state
    goal = ob.State(stateSpace)
    goal[0] = goal[1] = 8.
    goal[2] = 0
    goal[3] = goal[4] = 0.

    # set the start & goal states
    setup.setStartAndGoalStates(start, goal, .5)

    # set the planner
    planner = oc.RRT(setup.getSpaceInformation())
    setup.setPlanner(planner)

    # try to solve the problem
    if setup.solve(40):
        # print the (approximate) solution path: print states along the path
        # and controls required to get from one state to the next
        path = setup.getSolutionPath()
        #path.interpolate(); # uncomment if you want to plot the path
        for i in range(path.getStateCount()):
            s0 = path.getState(i)[0]
            s1 = path.getState(i)[1]
            print s0.getX(), s0.getY(), s0.getYaw(), s1[0], s1[1],
            if i==0:
                # null controls applied for zero seconds to get to start state
                print "0 0 0"
            else:
                # print controls and control duration needed to get from state i-1 to state i
                c = path.getControl(i-1)
                print c[0], c[1], path.getControlDuration(i-1)
        if not setup.haveExactSolutionPath():
            print "Solution is approximate. Distance to actual goal is ", \
                setup.getGoal().getDifference()

if __name__ == '__main__':
    car = oa.DynamicCarPlanning()
    dynamicCarDemo(car)
