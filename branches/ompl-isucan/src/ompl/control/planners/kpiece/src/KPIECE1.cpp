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

#include "ompl/control/planners/kpiece/KPIECE1.h"
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/util/Exception.h"
#include <limits>
#include <cassert>

void ompl::control::KPIECE1::setup(void)
{
    Planner::setup();
    if (!projectionEvaluator_)
    {
	projectionEvaluator_ = si_->getStateManifold()->getProjection();
	msg_.inform("Attempt to use default projection");
    }
    if (!projectionEvaluator_)
	throw Exception("No projection evaluator specified");
    projectionEvaluator_->checkCellDimensions();
    tree_.grid.setDimension(projectionEvaluator_->getDimension());
}

void ompl::control::KPIECE1::clear(void)
{
    freeMemory();
    tree_.grid.clear();
    tree_.size = 0;
    tree_.iteration = 1;
    addedStartStates_ = 0;
}

void ompl::control::KPIECE1::freeMemory(void)
{
    freeGridMotions(tree_.grid);
}

void ompl::control::KPIECE1::freeGridMotions(Grid &grid)
{
    for (Grid::iterator it = grid.begin(); it != grid.end() ; ++it)
	freeCellData(it->second->data);
}

void ompl::control::KPIECE1::freeCellData(CellData *cdata)
{
    for (unsigned int i = 0 ; i < cdata->motions.size() ; ++i)
	freeMotion(cdata->motions[i]);
    delete cdata;
}

void ompl::control::KPIECE1::freeMotion(Motion *motion)
{
    if (motion->state)
	si_->freeState(motion->state);
    if (motion->control)
	siC_->freeControl(motion->control);
    delete motion;
}

// return the index of the state to be used in the next motion (the reached state of the motion)
unsigned int ompl::control::KPIECE1::findNextMotion(const Grid::Coord &origin, const std::vector<Grid::Coord> &coords, unsigned int index, unsigned int last)
{
    // if the first state is already out of the origin cell, then most of the motion is in that cell
    if (coords[index] != origin)
    {
	for (unsigned int i = index + 1 ; i <= last ; ++i)
	    if (coords[i] != coords[index])
		return i - 1;
    }
    else
    {
	for (unsigned int i = index + 1 ; i <= last ; ++i)
	    if (coords[i] != origin)
		return i - 1;
    }
    return last;
}

bool ompl::control::KPIECE1::solve(double solveTime)
{
    base::Goal                       *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion     *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
    
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
	    addMotion(motion, 1.0);
	}
	else
	    msg_.error("Initial state is invalid!");
    }
    
    if (tree_.grid.size() == 0)
    {
	msg_.error("There are no valid initial states!");
	return false;	
    }    

    msg_.inform("Starting with %u states", tree_.size);
    
    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();

    Control *rctrl = siC_->allocControl();
    Grid::Coord origin;
    std::vector<Grid::Coord> coords(siC_->getMaxControlDuration() + 1);
    std::vector<base::State*> states(siC_->getMaxControlDuration() + 1);
    for (unsigned int i = 0 ; i < states.size() ; ++i)
	states[i] = si_->allocState();
    
    // coordinates of the goal state and the best state seen so far
    Grid::Coord best_coord, better_coord;
    bool haveBestCoord = false;
    bool haveBetterCoord = false;
    if (goal_s)
    {
	goal_s->sampleGoal(states[0]);
	projectionEvaluator_->computeCoordinates(states[0], best_coord);
	haveBestCoord = true;
    }
    
    while (time::now() < endTime)
    {
	tree_.iteration++;
	
	/* Decide on a state to expand from */
	Motion     *existing = NULL;
	Grid::Cell *ecell = NULL;

	if (rng_.uniform01() < goalBias_)
	{
	    if (haveBestCoord)
		ecell = tree_.grid.getCell(best_coord);
	    if (!ecell && haveBetterCoord)
		ecell = tree_.grid.getCell(better_coord);
	    if (ecell)
		existing = ecell->data->motions[rng_.halfNormalInt(0, ecell->data->motions.size() - 1)];
	    else
		selectMotion(existing, ecell);
	}
	else
	    selectMotion(existing, ecell);
	assert(existing);

	/* sample a random control */
	cCore_->sample(rctrl);
	
	/* propagate */
	unsigned int cd = cCore_->sampleStepCount(siC_->getMinControlDuration(), siC_->getMaxControlDuration());
	cd = siC_->propagateWhileValid(existing->state, rctrl, cd, states, false);

	/* if we have enough steps */
	if (cd >= siC_->getMinControlDuration())
	{

	    // split the motion into smaller ones, so we do not cross cell boundaries
	    projectionEvaluator_->computeCoordinates(existing->state, origin);
	    for (unsigned int i = 0 ; i < cd ; ++i)
		projectionEvaluator_->computeCoordinates(states[i], coords[i]);
	    
	    unsigned int last = cd - 1;
	    unsigned int index = 0;
	    while (index < last)
	    {		
		unsigned int nextIndex = findNextMotion(origin, coords, index, last);
		Motion *motion = new Motion(siC_);
		si_->copyState(motion->state, states[nextIndex]);
		siC_->copyControl(motion->control, rctrl);
		motion->steps = nextIndex - index + 1;
		motion->parent = existing;

		double dist = 0.0;
		bool solved = goal->isSatisfied(motion->state, &dist);
		addMotion(motion, dist);
		
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
		    better_coord = coords[nextIndex];
		    haveBetterCoord = true;
		}
		
		// new parent will be the newly created motion
		existing = motion;
		
		origin = coords[nextIndex];
		index = nextIndex + 1;
	    }
	    
	    if (solution)
		break;
	    
	    // update cell score 
	    ecell->data->score *= goodScoreFactor_;
	}
	else
	    ecell->data->score *= badScoreFactor_;
	
	tree_.grid.update(ecell);
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

    siC_->freeControl(rctrl);
    for (unsigned int i = 0 ; i < states.size() ; ++i)
	si_->freeState(states[i]);
    
    msg_.inform("Created %u states in %u cells (%u internal + %u external)", tree_.size, tree_.grid.size(),
		 tree_.grid.countInternal(), tree_.grid.countExternal());
    
    return goal->isAchieved();
}

bool ompl::control::KPIECE1::selectMotion(Motion* &smotion, Grid::Cell* &scell)
{
    scell = rng_.uniform01() < std::max(selectBorderPercentage_, tree_.grid.fracExternal()) ?
	tree_.grid.topExternal() : tree_.grid.topInternal();
    if (scell && !scell->data->motions.empty())
    {
	scell->data->selections++;
	smotion = scell->data->motions[rng_.halfNormalInt(0, scell->data->motions.size() - 1)];
	return true;
    }
    else
	return false;
}

unsigned int ompl::control::KPIECE1::addMotion(Motion *motion, double dist)
{
    Grid::Coord coord;
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    Grid::Cell* cell = tree_.grid.getCell(coord);
    unsigned int created = 0;
    if (cell)
    {
	cell->data->motions.push_back(motion);
	cell->data->coverage += motion->steps;
	tree_.grid.update(cell);
    }
    else
    {
	cell = tree_.grid.createCell(coord);
	cell->data = new CellData();
	cell->data->motions.push_back(motion);
	cell->data->coverage = motion->steps;
	cell->data->iteration = tree_.iteration;
	cell->data->selections = 1;
	cell->data->score = 1.0 / (1e-3 + dist);
	tree_.grid.add(cell);
	created = 1;
    }
    tree_.size++;
    return created;
}

void ompl::control::KPIECE1::getPlannerData(base::PlannerData &data) const
{
    data.states.resize(0);
    data.states.reserve(tree_.size);
    
    std::vector<CellData*> cdata;
    tree_.grid.getContent(cdata);
    for (unsigned int i = 0 ; i < cdata.size() ; ++i)
	for (unsigned int j = 0 ; j < cdata[i]->motions.size() ; ++j)
	    data.states.push_back(cdata[i]->motions[j]->state); 
}
