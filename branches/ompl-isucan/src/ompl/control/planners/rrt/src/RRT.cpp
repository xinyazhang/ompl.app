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

#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <limits>

bool ompl::control::RRT::solve(double solveTime)
{
    base::Goal                   *goal = pdef_->getGoal().get(); 
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
	    Motion *motion = new Motion(siC_);
	    si_->copyState(motion->state, st);
	    siC_->nullControl(motion->control);
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
    
    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    
    Motion      *rmotion = new Motion(siC_);
    base::State  *rstate = rmotion->state;
    Control       *rctrl = rmotion->control;
    base::State  *xstate = si_->allocState();
    
    while (time::now() < endTime)
    {	
	/* sample random state (with goal biasing) */
	if (goal_s && rng_.uniform01() < goalBias_)
	    goal_s->sampleGoal(rstate);
	else
	    sampler_->sample(rstate);
	
	/* find closest state in the tree */
	Motion *nmotion = nn_.nearest(rmotion);
	
	/* sample a random control */
	cCore_->sample(rctrl);
	unsigned int cd = cCore_->sampleStepCount(siC_->getMinControlDuration(), siC_->getMaxControlDuration());
	cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, xstate);

	if (cd >= siC_->getMinControlDuration())
	{
	    /* create a motion */
	    Motion *motion = new Motion(siC_);
	    si_->copyState(motion->state, xstate);
	    siC_->copyControl(motion->control, rctrl);
	    motion->steps = cd;
	    motion->parent = nmotion;
	    
	    nn_.add(motion);
	    double dist = 0.0;
	    bool solved = goal->isSatisfied(motion->state, &dist);
	    if (solved)
	    {
		approxdif = dist;
		solution = motion;
		break;
	    }
	    if (dist < approxdif)
	    {
		approxdif = dist;
		approxsol = motion;
	    }
	}
    }
    
    bool approximate = false;
    if (solution == NULL)
    {
	solution = approxsol;
	approximate = true;
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

	/* set the solution path */
	PathControl *path = new PathControl(si_);
   	for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	{   
	    path->states.push_back(si_->cloneState(mpath[i]->state));
	    if (mpath[i]->parent)
	    {
		path->controls.push_back(siC_->cloneControl(mpath[i]->control));
		path->controlDurations.push_back(mpath[i]->steps * siC_->getPropagationStepSize());
	    }
	}
	goal->setDifference(approxdif);
	goal->setSolutionPath(base::PathPtr(path), approximate);

	if (approximate)
	    msg_.warn("Found approximate solution");
    }
    
    if (rmotion->state)
	si_->freeState(rmotion->state);
    if (rmotion->control)
	siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);
    
    msg_.inform("Created %u states", nn_.size());
    
    return goal->isAchieved();
}

void ompl::control::RRT::getPlannerData(base::PlannerData &data) const
{
    std::vector<Motion*> motions;
    nn_.list(motions);
    data.states.resize(motions.size());
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	data.states[i] = motions[i]->state;
}
