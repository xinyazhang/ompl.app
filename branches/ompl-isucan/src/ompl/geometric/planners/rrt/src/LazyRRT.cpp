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

#include "ompl/geometric/planners/rrt/LazyRRT.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <limits>
#include <cassert>

void ompl::geometric::LazyRRT::setup(void)
{
    Planner::setup();
    if (maxDistance_ < std::numeric_limits<double>::epsilon())
    {
	maxDistance_ = si_->estimateExtent() / 5.0;
	msg_.warn("Maximum motion extension distance is %f", maxDistance_);
    }
}

void ompl::geometric::LazyRRT::clear(void)
{
    freeMemory();
    nn_.clear();
    addedStartStates_ = 0;
}

void ompl::geometric::LazyRRT::freeMemory(void)
{
    std::vector<Motion*> motions;
    nn_.list(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
	if (motions[i]->state)
	    si_->freeState(motions[i]->state);		    
	delete motions[i];
    }
}

bool ompl::geometric::LazyRRT::solve(double solveTime)
{
    base::Goal                 *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
    
    if (!goal)
    {
	msg_.error("Goal undefined");
	return false;
    }

    time::point endTime = time::now() + time::seconds(solveTime);
    
    for (unsigned int i = addedStartStates_ ; i < pdef_->getStartStateCount() ; ++i, ++addedStartStates_)
    {
	const base::State *st = pdef_->getStartState(i);
	if (si_->satisfiesBounds(st) && si_->isValid(st))
	{ 
	    Motion *motion = new Motion(si_);
	    si_->copyState(motion->state, st);
	    motion->valid = true;
	    nn_.add(motion);
	}	
	else
	    msg_.error("Initial state is invalid!");
    }
    
    if (nn_.size() == 0)
    {
	msg_.error("There are no valid initial states!");
	return false;	
    }    

    msg_.inform("Starting with %u states", nn_.size());

    Motion *solution = NULL;
    double  distsol  = -1.0;
    Motion *rmotion  = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    
 RETRY:

    while (time::now() < endTime)
    {
	/* sample random state (with goal biasing) */
	if (goal_s && rng_.uniform01() < goalBias_)
	    goal_s->sampleGoal(rstate);
	else
	    sampler_->sample(rstate);

	/* find closest state in the tree */
	Motion *nmotion = nn_.nearest(rmotion);
	assert(nmotion != rmotion);
	base::State *dstate = rstate;
	
	/* find state to add */
	double d = si_->distance(nmotion->state, rstate);
	if (d > maxDistance_)
	{
	    si_->getStateManifold()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
	    dstate = xstate;
	}

	/* create a motion */
	Motion *motion = new Motion(si_);
	si_->copyState(motion->state, dstate);
	motion->parent = nmotion;
	nmotion->children.push_back(motion);
	nn_.add(motion);
	
	double dist = 0.0;
	if (goal->isSatisfied(motion->state, &dist))
	{
	    distsol = dist;
	    solution = motion;
	    break;
	}
    }
    
    if (solution != NULL)
    {
	/* construct the solution path */
	std::vector<Motion*> mpath;
	while (solution != NULL)
	{
	    mpath.push_back(solution);
	    solution = solution->parent;
	}
	
	/* check the path */
	for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	    if (!mpath[i]->valid)
	    {
		if (si_->checkMotion(mpath[i]->parent->state, mpath[i]->state))
		    mpath[i]->valid = true;
		else
		{
		    removeMotion(mpath[i]);
		    goto RETRY;
		}
	    }
	
	/* set the solution path */
	PathGeometric *path = new PathGeometric(si_);
	for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	    path->states.push_back(si_->cloneState(mpath[i]->state));
	
	goal->setDifference(distsol);
	goal->setSolutionPath(base::PathPtr(path));
	
    }
    
    si_->freeState(xstate);
    si_->freeState(rstate);
    delete rmotion;
    
    msg_.inform("Created %u states", nn_.size());

    return goal->isAchieved();
}

void ompl::geometric::LazyRRT::removeMotion(Motion *motion)
{
    nn_.remove(motion);
    
    /* remove self from parent list */
    
    if (motion->parent)
    {
	for (unsigned int i = 0 ; i < motion->parent->children.size() ; ++i)
	    if (motion->parent->children[i] == motion)
	    {
		motion->parent->children.erase(motion->parent->children.begin() + i);
		break;
	    }
    }    

    /* remove children */
    for (unsigned int i = 0 ; i < motion->children.size() ; ++i)
    {
	motion->children[i]->parent = NULL;
	removeMotion(motion->children[i]);
    }
}

void ompl::geometric::LazyRRT::getPlannerData(base::PlannerData &data) const
{
    std::vector<Motion*> motions;
    nn_.list(motions);
    data.states.resize(motions.size());
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	data.states[i] = motions[i]->state;
}
