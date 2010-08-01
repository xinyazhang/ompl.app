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

#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <algorithm>
#include <queue>
#include <limits>

void ompl::geometric::PRM::setup(void)
{
    Planner::setup();
    sampler_ = si_->allocStateSampler();
}

void ompl::geometric::PRM::clear(void)
{
    freeMemory();
    nn_.clear();
    componentCount_ = 0;
    componentSizes_.clear();
}

void ompl::geometric::PRM::freeMemory(void)
{
    std::vector<Milestone*> milestones;
    nn_.list(milestones);
    for (unsigned int i = 0 ; i < milestones.size() ; ++i)
    {
	if (milestones[i]->state)
	    si_->freeState(milestones[i]->state);
	delete milestones[i];
    }
}

void ompl::geometric::PRM::growRoadmap(const std::vector<Milestone*> &start,
				       const std::vector<Milestone*> &goal,
				       double growTime, base::State *workState)
{
    time::point endTime = time::now() + time::seconds(growTime);
    while (time::now() < endTime)
    {
	// search for a valid state
	bool found = false;
	while (!found && time::now() < endTime)
	{
	    int attempts = 0;
	    do
	    {
		sampler_->sample(workState);
		found = si_->isValid(workState);
		attempts++;
	    } while (attempts < 10 && !found);
	}	
	// add it as a milestone
	if (found)
	{
	    addMilestone(si_->cloneState(workState));
	    if (haveSolution(start, goal))
		break;
	}
    }
}

ompl::geometric::PRM::Milestone* ompl::geometric::PRM::haveSolution(const std::vector<Milestone*> &start,
								    const std::vector<Milestone*> &goal)
{
    base::Goal *g = pdef_->getGoal().get();
    for (unsigned int i = 0 ; i < goal.size() ; ++i)
	for (unsigned int j = 0 ; j < start.size() ; ++j)
	    if (goal[i]->component == start[j]->component && g->isStartGoalPairValid(goal[i]->state, start[j]->state))
		return goal[i];
    return NULL;
}

bool ompl::geometric::PRM::solve(double solveTime)
{
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());
    
    if (!goal)
    {
	msg_.error("Goal undefined or unknown type of goal");
	return false;
    }
    
    time::point endTime = time::now() + time::seconds(solveTime);

    std::vector<Milestone*> startM;
    std::vector<Milestone*> goalM;

    // add the valid start states as milestones
    for (unsigned int i = 0 ; i < pdef_->getStartStateCount() ; ++i)
    {
	const base::State *st = pdef_->getStartState(i);
	if (si_->satisfiesBounds(st) && si_->isValid(st))
	    startM.push_back(addMilestone(si_->cloneState(st)));
	else
	    msg_.error("Initial state is invalid!");
    }
    
    if (startM.size() == 0)
    {
	msg_.error("There are no valid initial states!");
	return false;	
    }
    
    if (goal->maxSampleCount() <= 0)
    {
	msg_.error("Insufficient states in sampleable goal region");
	return false;
    }
    
    unsigned int nrStartStates = nn_.size();
    msg_.inform("Starting with %u states", nrStartStates);
    
    base::State *xstate = si_->allocState();

    while (time::now() < endTime)
    {
	// find at least one valid goal state
	if (goal->maxSampleCount() > goalM.size())
	{
	    while (time::now() < endTime)
	    {
		goal->sampleGoal(xstate);
		if (si_->satisfiesBounds(xstate) && si_->isValid(xstate))
		{
		    goalM.push_back(addMilestone(si_->cloneState(xstate)));
		    break;
		}
	    }
	    if (goalM.empty())
	    {
		msg_.error("Unable to find any valid goal states");
		break;
	    }
	}

	// if there already is a solution, construct it
	if (haveSolution(startM, goalM))
	{
	    constructSolution(startM, goalM);
	    break;
	}
	// othewise, spend some time building a roadmap
	else
	{
	    // if it is worth looking at other goal regions, plan for part of the time
	    if (goal->maxSampleCount() > goalM.size())
		growRoadmap(startM, goalM, time::seconds(endTime - time::now()) / 4.0, xstate);
	    // otherwise, just go ahead and build the roadmap
	    else
		growRoadmap(startM, goalM, time::seconds(endTime - time::now()), xstate);
	    // if a solution has been found, construct it
	    if (haveSolution(startM, goalM))
	    {
		constructSolution(startM, goalM);
		break;
	    }
	}
    }
    si_->freeState(xstate);
    
    msg_.inform("Created %u states", nn_.size() - nrStartStates);
    
    return goal->isAchieved();
}

ompl::geometric::PRM::Milestone* ompl::geometric::PRM::addMilestone(base::State *state) 
{
    Milestone *m = new Milestone();
    m->state = state;
    m->component = componentCount_;
    componentSizes_[m->component] = 1;
    
    // connect to nearest neighbors
    std::vector<Milestone*> nbh;
    nn_.nearest(m, maxNearestNeighbors_, nbh);
    for (unsigned int i = 0 ; i < nbh.size() ; ++i)
	if (si_->checkMotion(m->state, nbh[i]->state))
	{	    
	    m->adjacent.push_back(nbh[i]);
	    nbh[i]->adjacent.push_back(m);
	    m->costs.push_back(si_->distance(m->state, nbh[i]->state));
	    nbh[i]->costs.push_back(m->costs.back());
	    uniteComponents(m, nbh[i]);
	}
    
    // if the new milestone was no absorbed in an existing component, 
    // increase the number of components
    if (m->component == componentCount_)
	componentCount_++;
    nn_.add(m);
    return m;    
}

void ompl::geometric::PRM::uniteComponents(Milestone *m1, Milestone *m2)
{
    if (m1->component == m2->component)
	return;
    
    if (componentSizes_[m1->component] > componentSizes_[m2->component])
	std::swap(m1, m2);
    
    const unsigned long c = m2->component;
    componentSizes_[c] += componentSizes_[m1->component];
    componentSizes_.erase(m1->component);
    
    std::queue<Milestone*> q;
    q.push(m1);
    
    while (!q.empty())
    {
	Milestone *m = q.front();
	m->component = c;
	q.pop();
	for (unsigned int i = 0 ; i < m->adjacent.size() ; ++i)
	    if (m->adjacent[i]->component != c)
		q.push(m->adjacent[i]);
    }
}

bool ompl::geometric::PRM::findPath(const std::vector<Milestone*> &start, std::map<Milestone*, bool> &seen, std::vector<Milestone*> &path)
{
    Milestone *m = path.back();
    for (unsigned int i = 0 ; i < start.size() ; ++i)
	if (m == start[i])
	    return true;
    seen[m] = true;
    for (unsigned int i = 0 ; i < m->adjacent.size() ; ++i)
    {
	if (seen.find(m->adjacent[i]) == seen.end())
	{
	    path.push_back(m->adjacent[i]);
	    if (findPath(start, seen, path))
		return true;
	    path.pop_back();
	}
    }
    return false;
}

void ompl::geometric::PRM::constructSolution(const std::vector<Milestone*> &start, const std::vector<Milestone*> &goal)
{
    Milestone *g = haveSolution(start, goal);
    if (g == NULL)
	return;
    
    std::map<Milestone*, bool> seen;
    std::vector<Milestone*> path;
    path.push_back(g);
    if (!findPath(start, seen, path))
	return;

    /* set the solution path */
    PathGeometric *p = new PathGeometric(si_);
    for (int i = path.size() - 1 ; i >= 0 ; --i)
	p->states.push_back(si_->cloneState(path[i]->state));
    pdef_->getGoal()->setSolutionPath(base::PathPtr(p));
}

void ompl::geometric::PRM::getPlannerData(base::PlannerData &data) const
{
    std::vector<Milestone*> milestones;
    nn_.list(milestones);
    data.states.resize(milestones.size());
    for (unsigned int i = 0 ; i < milestones.size() ; ++i)
	data.states[i] = milestones[i]->state;
}
