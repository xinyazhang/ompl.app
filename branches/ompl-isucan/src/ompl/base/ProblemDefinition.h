/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

#ifndef OMPL_BASE_PROBLEM_DEFINITION_
#define OMPL_BASE_PROBLEM_DEFINITION_

#include "ompl/base/State.h"
#include "ompl/base/Goal.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/Time.h"
#include "ompl/base/ScopedState.h"

#include <vector>
#include <cstdlib>
#include <iostream>
#include <limits>

#include <boost/noncopyable.hpp>

namespace ompl
{
    namespace base
    {
	
	/** \brief Forward declaration of ompl::base::ProblemDefinition */
	ClassForward(ProblemDefinition);
	
	/** \class ompl::base::ProblemDefinitionPtr
	    \brief A boost shared pointer wrapper for ompl::base::ProblemDefinition */

	/** \brief Definition of a problem to be solved. This includes
	    the start state(s) for the system and a goal specification */
	class ProblemDefinition : private boost::noncopyable
	{
	public:

	    /** \brief Create a problem definition given the SpaceInformation it is part of */
	    ProblemDefinition(const SpaceInformationPtr &si) : si_(si)
	    {
	    }
	    
	    virtual ~ProblemDefinition(void)
	    {
		clearStartStates();
	    }
	    
	    /** \brief Add a start state. The state is copied. */
	    void addStartState(const State *state)
	    {
		startStates_.push_back(si_->cloneState(state));
	    }
	    
	    /** \copydoc addStartState() */
	    void addStartState(const ScopedState<> &state)
	    {
		startStates_.push_back(si_->cloneState(state.get()));
	    }
	    
	    /** \brief Check whether a specified starting state is
		already included in the problem definition and
		optionally return the index of that starting state */
	    bool hasStartState(const State *state, unsigned int *startIndex = NULL);
	    
	    /** \brief Clear all start states (memory is freed) */
	    void clearStartStates(void)
	    {
		for (unsigned int i = 0 ; i < startStates_.size() ; ++i)
		    si_->freeState(startStates_[i]);
		startStates_.clear();
	    }
	    
	    /** \brief Returns the number of start states */
	    unsigned int getStartStateCount(void) const
	    {
		return startStates_.size();
	    }
	    
	    /** \brief Returns a specific start state */
	    const State* getStartState(unsigned int index) const
	    {
		return startStates_[index];
	    }

	    /** \copydoc getStartState() */
	    State* getStartState(unsigned int index)
	    {
		return startStates_[index];
	    }

	    /** \brief Set the goal. */
	    void setGoal(const GoalPtr &goal)
	    {
		goal_ = goal;
	    }
	    
	    /** \brief Clear the goal. Memory is freed. */
	    void clearGoal(void)
	    {
		goal_.reset();
	    }
	    
	    /** \brief Return the current goal */
	    const GoalPtr& getGoal(void) const
	    {
		return goal_;
	    }
	    
	    /** \brief Get all the input states. This includes start
		states and states that are part of goal regions that
		can be casted as ompl::base::GoalState or
		ompl::base::GoalStates. */
	    void getInputStates(std::vector<const State*> &states) const;
	    
	    /** \brief In the simplest case possible, we have a single
		starting state and a goal state.

		This function simply configures the problem definition
		using these states (performs the needed calls to
		addStartState(), creates an instance of
		ompl::base::GoalState and calls setGoal() on it. */
	    void setStartAndGoalStates(const State *start, const State *goal, const double threshold = std::numeric_limits<double>::epsilon());

	    /** \copydoc setStartAndGoalStates() */
	    void setStartAndGoalStates(const ScopedState<> &start, const ScopedState<> &goal, const double threshold = std::numeric_limits<double>::epsilon())
	    {
		setStartAndGoalStates(start.get(), goal.get(), threshold);
	    }
	    
	    /** \brief A problem is trivial if a given starting state already
		in the goal region, so we need no motion planning. startID
		will be set to the index of the starting state that
		satisfies the goal. The distance to the goal can
		optionally be returned as well. */
	    bool isTrivial(unsigned int *startIndex = NULL, double *distance = NULL) const;
	    
	    /** \brief Many times the start or goal state will barely touch an obstacle. In this case, we may want to automaticaly
	      * find a nearby state that is valid so motion planning can be performed. This function enables this behaviour.
	      * The allowed distance for both start and goal states is specified. The number of attempts
	      * is also specified. Returns true if all states are valid after completion. */
	    bool fixInvalidInputStates(double distStart, double distGoal, unsigned int attempts);
	    
	    /** \brief Print information about the start and goal states */
	    void print(std::ostream &out = std::cout) const;

	protected:
	    
	    /** \brief Helper function for fixInvalidInputStates(). Attempts to fix an individual state */
	    bool fixInvalidInputState(State *state, double dist, bool start, unsigned int attempts);

	    /** \brief The space information this problem definition is for */
	    SpaceInformationPtr  si_;

	    /** \brief The set of start states */
	    std::vector<State*>  startStates_;

	    /** \brief The goal representation */
	    GoalPtr              goal_;
	    
	    /** \brief Interface for console output */
	    msg::Interface       msg_;
	};

	
	/** \brief Helper class to extract valid start & goal
	    states. Usually used internally by planners.

	    This class is meant to behave correctly if the user
	    updates the problem definition between subsequent calls to
	    ompl::base::Planner::solve() \b without calling
	    ompl::base::Planner::clear() in between. Only allowed
	    changes to the problem definition are accounted for:
	    adding of starring states or adding of goal states for
	    instances of ompl::base::GoalSampleableRegion. */
	class PlannerInputStates
	{
	public:
	    
	    /** \brief Default constructor. No work is performed. A
		call to use() is expected before calls to nextStart()
		or nextGoal(). */
	    PlannerInputStates(void)
	    {
		tempState_ = NULL;
		clear();
	    }
	    
	    /** \brief Destructor. Clear allocated memory */
	    ~PlannerInputStates(void)
	    {
		clear();
	    }
	    
	    /** \brief Clear all stored information. A call to use()
		is needed before further calls to nextStart() or
		nextGoal(). */
	    void clear(void);
	    
	    /** \brief Set the space information and problem
		definition this class operates on. This call must be
		made before any calls to newStart() or newGoal(). If
		the problem definition or space information have been
		changed since the previous call, clear() is called and
		the function returns true. Otherwise, the function
		returns false. */
	    bool use(const SpaceInformationPtr &si, const ProblemDefinitionPtr &pdef);
	    
	    /** \brief Set the space information and problem
		definition this class operates on. This call must be
		made before any calls to newStart() or newGoal(). If
		the problem definition or space information have been
		changed since the previous call, clear() is called and
		the function returns true. Otherwise, the function
		returns false. */
	    bool use(const SpaceInformation *si, const ProblemDefinition *pdef);
	    
	    /** \brief Return the next valid start state or NULL if no
		more valid start states are available. */
	    const State* nextStart(void);
	    
	    /** \brief Return the next valid goal state or NULL if no
		more valid goal states are available.  Because
		sampling of goal states may also produce invalid
		goals, this function takes an optional argument that
		specifies the time point when it should give up
		searching for valid goals. Only one attempt is made if
		no such argument is given. */
	    const State* nextGoal(time::point maxEndTime = time::now());
	    
	    /** \brief Check if there are more potential start states */
	    bool haveMoreStartStates(void) const;

	    /** \brief Check if there are more potential start states */
	    bool haveMoreGoalStates(void) const;
	    
	    /** \brief Get the number of start states from the problem
		definition that were already seen, including invalid
		ones. */
	    unsigned int getSeenStartStatesCount(void) const
	    {
		return addedStartStates_;
	    }

	    /** \brief Get the number of sampled goal states, including invalid ones */
	    unsigned int getSampledGoalsCount(void) const
	    {
		return sampledGoalsCount_;
	    }
	    
	private:
	    
	    unsigned int                addedStartStates_;
	    unsigned int                sampledGoalsCount_;
	    State                      *tempState_;
	    
	    const ProblemDefinition    *pdef_;
	    const SpaceInformation     *si_;
	};
	
    }    
}

#endif

