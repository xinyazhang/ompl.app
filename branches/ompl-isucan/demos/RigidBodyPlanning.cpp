/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    /// cast the abstract state type to the type we expect
    const ob::SE3StateManifold::StateType *se3state = state->as<ob::SE3StateManifold::StateType>();

    /// extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateManifold::StateType *pos = se3state->as<ob::RealVectorStateManifold::StateType>(0);

    /// extract the second component of the state and cast it to what we expect
    const ob::SO3StateManifold::StateType *rot = se3state->as<ob::SO3StateManifold::StateType>(1);
    
    /// check validity of state defined by pos & rot
    

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return (void*)rot != (void*)pos;
}

void plan(void)
{
    /// construct the manifold we are planning in
    ob::StateManifoldPtr manifold(new ob::SE3StateManifold());

    /// set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    
    manifold->as<ob::SE3StateManifold>()->setBounds(bounds);
    
    /// construct an instance of  space information from this manifold
    ob::SpaceInformationPtr si(new ob::SpaceInformation(manifold));

    /// set state validity checking for this space
    si->setStateValidityChecker(boost::bind(&isStateValid, _1));
    
    /// create a random start state
    ob::MappedState<> start(manifold);
    start.random();

    /// create a random goal state
    ob::MappedState<> goal(manifold);
    goal.random();
    
    /// create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    /// set the start and goal states
    pdef->setStartAndGoalStates(start, goal);
    

    
    /// create a planner for the defined space
    ob::PlannerPtr planner(new og::RRTConnect(si));

    /// set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    /// perform setup steps for the planner
    planner->setup();


    /// print the settings for this space
    si->print(std::cout);

    /// print the problem settings
    pdef->print(std::cout);    
    
    /// attempt to solve the problem within one second of planning time
    bool solved = planner->solve(1.0);

    if (solved)
    {
	/// get the goal representation from the problem definition (not the same as the goal state)
	/// and inquire about the found path
	ob::PathPtr path = pdef->getGoal()->getSolutionPath();
	std::cout << "Found solution:" << std::endl;

	/// print the path to screen
	path->print(std::cout);
    }
    else
	std::cout << "No solution found" << std::endl;
}

void planWithSimpleSetup(void)
{
    /// construct the manifold we are planning in
    ob::StateManifoldPtr manifold(new ob::SE3StateManifold());

    /// set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    
    manifold->as<ob::SE3StateManifold>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(manifold);

    /// set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid, _1));
    
    /// create a random start state
    ob::MappedState<> start(manifold);
    start.random();

    /// create a random goal state
    ob::MappedState<> goal(manifold);
    goal.random();
    
    /// set the start and goal states; this call allows SimpleSetup to infer the planning manifold, if needed
    ss.setStartAndGoalStates(start, goal);
        
    /// attempt to solve the problem within one second of planning time
    bool solved = ss.solve(1.0);

    if (solved)
    {
	std::cout << "Found solution:" << std::endl;
	/// print the path to screen
	ss.simplifySolution();
	ss.getSolutionPath().print(std::cout);
    }
    else
	std::cout << "No solution found" << std::endl;
}

int main(int, char **)
{
    plan();
    
    std::cout << std::endl << std::endl;
    
    planWithSimpleSetup();
    
    return 0;
}
