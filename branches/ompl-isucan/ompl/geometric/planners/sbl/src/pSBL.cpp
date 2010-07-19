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

#include "ompl/geometric/planners/sbl/pSBL.h"
#include "ompl/base/GoalState.h"
#include <boost/thread.hpp>
#include <limits>
#include <cassert>

void ompl::geometric::pSBL::setup(void)
{
    Planner::setup();
    if (!projectionEvaluator_)
	throw Exception("No projection evaluator specified");
    projectionEvaluator_->checkCellDimensions();
    if (projectionEvaluator_->getDimension() <= 0)
	throw Exception("Dimension of projection needs to be larger than 0");
    if (maxDistance_ < std::numeric_limits<double>::epsilon())
    {
	maxDistance_ = si_->getStateValidityCheckingResolution() * 10.0;
	msg_.warn("Maximum motion extension distance is %f", maxDistance_);
    }
    tStart_.grid.setDimension(projectionEvaluator_->getDimension());
    tGoal_.grid.setDimension(projectionEvaluator_->getDimension());
}

void ompl::geometric::pSBL::clear(void)
{
    freeMemory();
    
    tStart_.grid.clear();
    tStart_.size = 0;
    
    tGoal_.grid.clear();
    tGoal_.size = 0;
    
    removeList_.motions.clear();
    
    addedStartStates_ = 0;
}

void ompl::geometric::pSBL::freeGridMotions(Grid<MotionSet> &grid)
{
    for (Grid<MotionSet>::iterator it = grid.begin(); it != grid.end() ; ++it)
	for (unsigned int i = 0 ; i < it->second->data.size() ; ++i)
	{
	    if (it->second->data[i]->state)
		si_->freeState(it->second->data[i]->state);
	    delete it->second->data[i];
	}
}

void ompl::geometric::pSBL::threadSolve(unsigned int tid, time::point endTime, SolutionInfo *sol)
{   
    base::GoalState *goal = static_cast<base::GoalState*>(pdef_->getGoal().get());
    
    std::vector<Motion*> solution;
    base::State *xstate = si_->allocState();
    bool      startTree = sCoreArray_[tid]->getRNG().uniformBool();
    
    while (!sol->found && time::now() < endTime)
    {
	bool retry = true;
	while (retry && !sol->found && time::now() < endTime)
	{
	    removeList_.lock.lock();
	    if (!removeList_.motions.empty())
	    {
		if (loopLock_.try_lock())
		{
		    retry = false;
		    std::map<Motion*, bool> seen;
		    for (unsigned int i = 0 ; i < removeList_.motions.size() ; ++i)
			if (seen.find(removeList_.motions[i].motion) == seen.end())
			    removeMotion(*removeList_.motions[i].tree, removeList_.motions[i].motion, seen);
		    removeList_.motions.clear();
		    loopLock_.unlock();
		}
	    }
	    else
		retry = false;
	    removeList_.lock.unlock();
	}
	
	if (sol->found || time::now() > endTime)
	    break;
	
	loopLockCounter_.lock();
	if (loopCounter_ == 0)
	    loopLock_.lock();
	loopCounter_++;
	loopLockCounter_.unlock();
	

	TreeData &tree      = startTree ? tStart_ : tGoal_;
	startTree = !startTree;
	TreeData &otherTree = startTree ? tStart_ : tGoal_;
	
	Motion *existing = selectMotion(sCoreArray_[tid]->getRNG(), tree);
	sCoreArray_[tid]->sampleNear(xstate, existing->state, maxDistance_);
	
	/* create a motion */
	Motion *motion = new Motion(si_);
	si_->copyState(motion->state, xstate);
	motion->parent = existing;
	motion->root = existing->root;
	
	existing->lock.lock();
	existing->children.push_back(motion);
	existing->lock.unlock();
	
	addMotion(tree, motion);

	if (checkSolution(sCoreArray_[tid]->getRNG(), !startTree, tree, otherTree, motion, solution))
	{
	    sol->lock.lock();
	    if (!sol->found)
	    {
		sol->found = true;
		PathGeometric *path = new PathGeometric(si_);
		for (unsigned int i = 0 ; i < solution.size() ; ++i)
		    path->states.push_back(si_->cloneState(solution[i]->state));
		goal->setDifference(0.0);
		goal->setSolutionPath(base::PathPtr(path));
	    }
	    sol->lock.unlock();
	}

	
	loopLockCounter_.lock();
	loopCounter_--;
	if (loopCounter_ == 0)
	    loopLock_.unlock();
	loopLockCounter_.unlock();
    }
    
    si_->freeState(xstate);    
}

bool ompl::geometric::pSBL::solve(double solveTime)
{
    base::GoalState *goal = dynamic_cast<base::GoalState*>(pdef_->getGoal().get());
    
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
	    Motion *motion = new Motion(si_);
	    si_->copyState(motion->state, st);
	    motion->valid = true;
	    motion->root = st;
	    addMotion(tStart_, motion);
	}
	else
	    msg_.error("Initial state is invalid!");
    }
    
    if (tGoal_.size == 0)
    {	   
	if (si_->satisfiesBounds(goal->state) && si_->isValid(goal->state))
	{
	    Motion *motion = new Motion(si_);
	    si_->copyState(motion->state, goal->state);
	    motion->valid = true;
	    motion->root = goal->state;
	    addMotion(tGoal_, motion);
	}
	else
	    msg_.error("Goal state is invalid!");
    }
    
    if (tStart_.size == 0 || tGoal_.size == 0)
    {
	msg_.error("Motion planning trees could not be initialized!");
	return false;
    }
    
    msg_.inform("Starting with %d states", (int)(tStart_.size + tGoal_.size));
    
    SolutionInfo sol;
    sol.found = false;
    loopCounter_ = 0;
    
    std::vector<boost::thread*> th(threadCount_);
    for (unsigned int i = 0 ; i < threadCount_ ; ++i)
	th[i] = new boost::thread(boost::bind(&pSBL::threadSolve, this, i, endTime, &sol));
    for (unsigned int i = 0 ; i < threadCount_ ; ++i)
    {
	th[i]->join();
	delete th[i];
    }
        
    msg_.inform("Created %u (%u start + %u goal) states in %u cells (%u start + %u goal)", tStart_.size + tGoal_.size, tStart_.size, tGoal_.size,
	     tStart_.grid.size() + tGoal_.grid.size(), tStart_.grid.size(), tGoal_.grid.size());
    
    return goal->isAchieved();
}

bool ompl::geometric::pSBL::checkSolution(RNG &rng, bool start, TreeData &tree, TreeData &otherTree, Motion *motion, std::vector<Motion*> &solution)
{
    Grid<MotionSet>::Coord coord;
    projectionEvaluator_->computeCoordinates(motion->state, coord);

    otherTree.lock.lock();    
    Grid<MotionSet>::Cell* cell = otherTree.grid.getCell(coord);
    
    if (cell && !cell->data.empty())
    {
	Motion *connectOther = cell->data[rng.uniformInt(0, cell->data.size() - 1)];
	otherTree.lock.unlock();    
	
	if (pdef_->getGoal()->isStartGoalPairValid(start ? motion->root : connectOther->root, start ? connectOther->root : motion->root))
	{
	    Motion *connect = new Motion(si_);
	    
	    si_->copyState(connect->state, connectOther->state);
	    connect->parent = motion;
	    connect->root = motion->root;
	    
	    motion->lock.lock();
	    motion->children.push_back(connect);
	    motion->lock.unlock();
	    
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
    else
	otherTree.lock.unlock();    
    
    return false;
}

bool ompl::geometric::pSBL::isPathValid(TreeData &tree, Motion *motion)
{
    std::vector<Motion*> mpath;
    
    /* construct the solution path */
    while (motion != NULL)
    {  
	mpath.push_back(motion);
	motion = motion->parent;
    }
    
    bool result = true;
    
    /* check the path */
    for (int i = mpath.size() - 1 ; result && i >= 0 ; --i)
    {
	mpath[i]->lock.lock();
	if (!mpath[i]->valid)
	{
	    if (si_->checkMotion(mpath[i]->parent->state, mpath[i]->state))
		mpath[i]->valid = true;
	    else
	    {
		// remember we need to remove this motion
		PendingRemoveMotion prm;
		prm.tree = &tree;
		prm.motion = mpath[i];
		removeList_.lock.lock();
		removeList_.motions.push_back(prm);
		removeList_.lock.unlock();
		result = false;
	    }
	}
	mpath[i]->lock.unlock();
    }
    
    return result;
}

ompl::geometric::pSBL::Motion* ompl::geometric::pSBL::selectMotion(RNG &rng, TreeData &tree)
{
    double sum  = 0.0;
    Grid<MotionSet>::Cell* cell = NULL;
    tree.lock.lock();
    double prob = rng.uniform01() * (tree.grid.size() - 1);
    for (Grid<MotionSet>::iterator it = tree.grid.begin(); it != tree.grid.end() ; ++it)
    {
	sum += (double)(tree.size - it->second->data.size()) / (double)tree.size;
	if (prob < sum)
	{
	    cell = it->second;
	    break;
	}
    }
    if (!cell && tree.grid.size() > 0)
	cell = tree.grid.begin()->second;
    ompl::geometric::pSBL::Motion* result = cell->data[rng.uniformInt(0, cell->data.size() - 1)];
    tree.lock.unlock();
    return result;
}

void ompl::geometric::pSBL::removeMotion(TreeData &tree, Motion *motion, std::map<Motion*, bool> &seen)
{
    /* remove from grid */
    seen[motion] = true;

    Grid<MotionSet>::Coord coord;
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    Grid<MotionSet>::Cell* cell = tree.grid.getCell(coord);
    if (cell)
    {
	for (unsigned int i = 0 ; i < cell->data.size(); ++i)
	    if (cell->data[i] == motion)
	    {
		cell->data.erase(cell->data.begin() + i);
		tree.size--;
		break;
	    }
	if (cell->data.empty())
	{
	    tree.grid.remove(cell);
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
	removeMotion(tree, motion->children[i], seen);
    }

    if (motion->state)
	si_->freeState(motion->state);
    delete motion;
}

void ompl::geometric::pSBL::addMotion(TreeData &tree, Motion *motion)
{
    Grid<MotionSet>::Coord coord;
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    tree.lock.lock();
    Grid<MotionSet>::Cell* cell = tree.grid.getCell(coord);
    if (cell)
	cell->data.push_back(motion);
    else
    {
	cell = tree.grid.createCell(coord);
	cell->data.push_back(motion);
	tree.grid.add(cell);
    }
    tree.size++;
    tree.lock.unlock();
}

void ompl::geometric::pSBL::getPlannerData(base::PlannerData &data) const
{
    data.states.resize(0);
    data.states.reserve(tStart_.size + tGoal_.size);
    
    std::vector<MotionSet> motions;
    tStart_.grid.getContent(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	for (unsigned int j = 0 ; j < motions[i].size() ; ++j)
	    data.states.push_back(motions[i][j]->state);    

    motions.clear();
    tGoal_.grid.getContent(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
	for (unsigned int j = 0 ; j < motions[i].size() ; ++j)
	    data.states.push_back(motions[i][j]->state);    
}

void ompl::geometric::pSBL::setThreadCount(unsigned int nthreads)
{
    assert(nthreads > 0);		
    threadCount_ = nthreads;
    sCoreArray_.resize(threadCount_);
}
