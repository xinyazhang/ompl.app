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

/** \author Ioan Sucan */

#include "ompl/geometric/planners/kpiece/LBKPIECE1.h"
#include "ompl/base/GoalSampleableRegion.h"
#include <limits>
#include <cassert>

void ompl::geometric::LBKPIECE1::setup(void)
{
    Planner::setup();
    if (!projectionEvaluator_)
	throw Exception("No projection evaluator specified");
    projectionEvaluator_->checkCellDimensions();
    if (projectionEvaluator_->getDimension() <= 0)
	throw Exception("Dimension of projection needs to be larger than 0");
    if (maxDistance_ < std::numeric_limits<double>::epsilon())
    {
	maxDistance_ = si_->estimateExtent() / 5.0;
	msg_.warn("Maximum motion extension distance is %f", maxDistance_);
    }
    tStart_.grid.setDimension(projectionEvaluator_->getDimension());
    tGoal_.grid.setDimension(projectionEvaluator_->getDimension());
}

bool ompl::geometric::LBKPIECE1::solve(double solveTime)
{
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());
    
    if (!goal)
    {
	msg_.error("Unknown type of goal (or goal undefined)");
	return false;
    }

    time::point endTime = time::now() + time::seconds(solveTime);
    
    for (unsigned int i = addedStartStates_ ; i < pdef_->getStartStateCount() ; ++i, ++addedStartStates_)
    {
	const base::State *st = pdef_->getStartState(i);
	if (si_->satisfiesBounds(st) && si_->isValid(st))
	{
	    Motion* motion = new Motion(si_);
	    si_->copyState(motion->state, st);
	    motion->root = st;
	    motion->valid = true;
	    addMotion(tStart_, motion);
	}
	else
	    msg_.error("Initial state is invalid!");
    }

    
    if (tStart_.size == 0)
    {
	msg_.error("Motion planning start tree could not be initialized!");
	return false;
    }

    if (goal->maxSampleCount() <= 0)
    {
	msg_.error("Insufficient states in sampleable goal region");
	return false;
    }
    
    msg_.inform("Starting with %d states", (int)(tStart_.size + tGoal_.size));
    
    std::vector<Motion*> solution;
    base::State *xstate = si_->allocState();
    base::State *gstate = si_->allocState();
    bool      startTree = true;
        
    while (time::now() < endTime)
    {
	TreeData &tree      = startTree ? tStart_ : tGoal_;
	startTree = !startTree;
	TreeData &otherTree = startTree ? tStart_ : tGoal_;
	tree.iteration++;
	
	// if there are any goals left to sample
	if (sampledGoalsCount_ < goal->maxSampleCount())
	{
	    // if we have not sampled too many goals already
	    if (tGoal_.size == 0 || sampledGoalsCount_ < tGoal_.size / 2)
	    {
		bool firstAttempt = true;
		
		while ((tGoal_.size == 0 || firstAttempt) && sampledGoalsCount_ < goal->maxSampleCount() && time::now() < endTime)
		{
		    firstAttempt = false;
		    goal->sampleGoal(gstate);
		    sampledGoalsCount_++;
		    if (si_->satisfiesBounds(gstate) && si_->isValid(gstate))
		    {
			Motion* motion = new Motion(si_);
			si_->copyState(motion->state, gstate);
			motion->root = motion->state;
			motion->valid = true;
			addMotion(tGoal_, motion);
		    }
		}
		
		if (tGoal_.size == 0)
		{
		    msg_.error("Unable to sample any valid states for goal tree");
		    break;
		}
	    }
	}

	
	Motion* existing = selectMotion(tree);
	assert(existing);
	sCore_->sampleNear(xstate, existing->state, maxDistance_);
	
	/* create a motion */
	Motion* motion = new Motion(si_);
	si_->copyState(motion->state, xstate);
	motion->parent = existing;
	motion->root = existing->root;
	existing->children.push_back(motion);
	
	addMotion(tree, motion);
	
	if (checkSolution(!startTree, tree, otherTree, motion, solution))
	{
	    PathGeometric *path = new PathGeometric(si_);
	    for (unsigned int i = 0 ; i < solution.size() ; ++i)
		path->states.push_back(si_->cloneState(solution[i]->state));
	    
	    goal->setDifference(0.0);
	    goal->setSolutionPath(base::PathPtr(path));
	    break;
	}
    }
    
    si_->freeState(gstate);
    si_->freeState(xstate);
    
    msg_.inform("Created %u (%u start + %u goal) states in %u cells (%u start + %u goal)", 
		 tStart_.size + tGoal_.size, tStart_.size, tGoal_.size,
		 tStart_.grid.size() + tGoal_.grid.size(), tStart_.grid.size(), tGoal_.grid.size());
    
    return goal->isAchieved();
}

bool ompl::geometric::LBKPIECE1::checkSolution(bool start, TreeData &tree, TreeData &otherTree, Motion* motion, std::vector<Motion*> &solution)
{
    Grid::Coord coord;
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    Grid::Cell* cell = otherTree.grid.getCell(coord);
    
    if (cell && !cell->data->motions.empty())
    {
	Motion* connectOther = cell->data->motions[rng_.uniformInt(0, cell->data->motions.size() - 1)];
	
	if (pdef_->getGoal()->isStartGoalPairValid(start ? motion->root : connectOther->root, start ? connectOther->root : motion->root))
	{
	    Motion* connect = new Motion(si_);
	    
	    si_->copyState(connect->state, connectOther->state);
	    connect->parent = motion;
	    connect->root = motion->root;
	    motion->children.push_back(connect);
	    addMotion(tree, connect);
	    
	    if (isPathValid(tree, connect) && isPathValid(otherTree, connectOther))
	    {
		/* extract the motions and put them in solution vector */
		
		std::vector<Motion*> mpath1;
		while (motion != NULL)
		{
		    mpath1.push_back(motion);
		    motion = motion->parent;
		}
		
		std::vector<Motion*> mpath2;
		while (connectOther != NULL)
		{
		    mpath2.push_back(connectOther);
		    connectOther = connectOther->parent;
		}
		
		if (!start)
		    mpath1.swap(mpath2);
		
		for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
		    solution.push_back(mpath1[i]);
		solution.insert(solution.end(), mpath2.begin(), mpath2.end());
		
		return true;
	    }
	}
    }
    return false;
}

bool ompl::geometric::LBKPIECE1::isPathValid(TreeData &tree, Motion *motion)
{
    std::vector<Motion*> mpath;
    
    /* construct the solution path */
    while (motion != NULL)
    {  
	mpath.push_back(motion);
	motion = motion->parent;
    }
    
    /* check the path */
    for (int i = mpath.size() - 1 ; i >= 0 ; --i)
	if (!mpath[i]->valid)
	{
	    if (si_->checkMotion(mpath[i]->parent->state, mpath[i]->state))
		mpath[i]->valid = true;
	    else
	    {
		removeMotion(tree, mpath[i]);
		return false;
	    }
	}
    return true;
}

ompl::geometric::LBKPIECE1::Motion* ompl::geometric::LBKPIECE1::selectMotion(TreeData &tree)
{
    Grid::Cell* cell = rng_.uniform01() < std::max(selectBorderPercentage_, tree.grid.fracExternal()) ?
	tree.grid.topExternal() : tree.grid.topInternal();
    if (cell && !cell->data->motions.empty())
    {
	cell->data->selections++;
	tree.grid.update(cell);
	return cell->data->motions[rng_.halfNormalInt(0, cell->data->motions.size() - 1)];
    }
    else
	return NULL;
}

void ompl::geometric::LBKPIECE1::removeMotion(TreeData &tree, Motion *motion)
{
    /* remove from grid */
    
    Grid::Coord coord;
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    Grid::Cell* cell = tree.grid.getCell(coord);
    if (cell)
    {
	for (unsigned int i = 0 ; i < cell->data->motions.size(); ++i)
	    if (cell->data->motions[i] == motion)
	    {
		cell->data->motions.erase(cell->data->motions.begin() + i);
		tree.size--;
		break;
	    }
	if (cell->data->motions.empty())
	{
	    tree.grid.remove(cell);
	    freeCellData(cell->data);
	    tree.grid.destroyCell(cell);
	}
    }
    
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
	removeMotion(tree, motion->children[i]);
    }
    
    freeMotion(motion);
}

void ompl::geometric::LBKPIECE1::addMotion(TreeData &tree, Motion *motion)
{
    Grid::Coord coord;
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    Grid::Cell *cell = tree.grid.getCell(coord);
    if (cell)
    {
	cell->data->motions.push_back(motion);
	cell->data->coverage += 1.0;
	tree.grid.update(cell);
    }
    else
    {
	cell = tree.grid.createCell(coord);
	cell->data = new CellData();
	cell->data->motions.push_back(motion);
	cell->data->coverage = 1.0;
	cell->data->iteration = tree.iteration;
	cell->data->selections = 1;
	tree.grid.add(cell);
    }
    tree.size++;
}

void ompl::geometric::LBKPIECE1::freeMemory(void)
{
    freeGridMotions(tStart_.grid);
    freeGridMotions(tGoal_.grid);
}

void ompl::geometric::LBKPIECE1::freeGridMotions(Grid &grid)
{
    for (Grid::iterator it = grid.begin(); it != grid.end() ; ++it)
	freeCellData(it->second->data);
}

void ompl::geometric::LBKPIECE1::freeCellData(CellData *cdata)
{
    for (unsigned int i = 0 ; i < cdata->motions.size() ; ++i)
	freeMotion(cdata->motions[i]);
    delete cdata;
}

void ompl::geometric::LBKPIECE1::freeMotion(Motion *motion)
{
    if (motion->state)
	si_->freeState(motion->state);
    delete motion;
}

void ompl::geometric::LBKPIECE1::clear(void)
{
    freeMemory();
    
    tStart_.grid.clear();
    tStart_.size = 0;
    tStart_.iteration = 1;
    
    tGoal_.grid.clear();
    tGoal_.size = 0;	    
    tGoal_.iteration = 1;
    
    sampledGoalsCount_ = 0;
    addedStartStates_ = 0;
}

void ompl::geometric::LBKPIECE1::getPlannerData(base::PlannerData &data) const
{
    data.states.resize(0);
    data.states.reserve(tStart_.size + tGoal_.size);
    
    std::vector<CellData*> cdata;
    tStart_.grid.getContent(cdata);
    for (unsigned int i = 0 ; i < cdata.size() ; ++i)
	for (unsigned int j = 0 ; j < cdata[i]->motions.size() ; ++j)
	    data.states.push_back(cdata[i]->motions[j]->state); 
    
    cdata.clear();
    tGoal_.grid.getContent(cdata);
    for (unsigned int i = 0 ; i < cdata.size() ; ++i)
	for (unsigned int j = 0 ; j < cdata[i]->motions.size() ; ++j)
	    data.states.push_back(cdata[i]->motions[j]->state); 
}
