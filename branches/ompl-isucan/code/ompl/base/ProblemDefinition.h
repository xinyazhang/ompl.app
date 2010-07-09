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

/* \author Ioan Sucan */

#ifndef OMPL_BASE_PROBLEM_DEFINITION_
#define OMPL_BASE_PROBLEM_DEFINITION_

#include "ompl/base/State.h"
#include "ompl/base/Goal.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"

#include <vector>
#include <cstdlib>
#include <iostream>

namespace ompl
{
    namespace base
    {
	
	ClassForward(ProblemDefinition);
	
	/** \brief Definition of a problem to be solved. This includes
	    the start state(s) for the system and a goal specification */
	class ProblemDefinition
	{
	public:
	    
	    ProblemDefinition(const SpaceInformationPtr &si) : m_si(si)
	    {
	    }
	    
	    virtual ~ProblemDefinition(void)
	    {
		clearStartStates();
	    }
	    
	    /** \brief Add a start state */
	    void addStartState(const State *state)
	    {
		State *copy = m_si->allocState();
		m_si->copyState(copy, state);
		m_startStates.push_back(copy);
	    }
	    
	    /** \brief Check whether a specified starting state is
		already included in the problem definition and
		optionally return the index of that starting state */
	    bool hasStartState(const State *state, unsigned int *startIndex = NULL);
	    
	    /** \brief Clear all start states (memory is freed) */
	    void clearStartStates(void)
	    {
		for (unsigned int i = 0 ; i < m_startStates.size() ; ++i)
		    m_si->freeState(m_startStates[i]);
		m_startStates.clear();
	    }
	    
	    /** \brief Returns the number of start states */
	    unsigned int getStartStateCount(void) const
	    {
		return m_startStates.size();
	    }
	    
	    /** \brief Returns a specific start state */
	    const State* getStartState(unsigned int index) const
	    {
		return m_startStates[index];
	    }

	    /** \brief Returns a specific start state */
	    State* getStartState(unsigned int index)
	    {
		return m_startStates[index];
	    }

	    /** \brief Set the goal. The memory for a previous goal is freed. */
	    void setGoal(const GoalPtr &goal)
	    {
		m_goal = goal;
	    }
	    
	    /** \brief Clear the goal. Memory is freed. */
	    void clearGoal(void)
	    {
		m_goal.reset();
	    }
	    
	    /** \brief Return the current goal */
	    GoalConstPtr getGoal(void) const
	    {
		return m_goal;
	    }

	    /** \brief Return the current goal */
	    const GoalPtr& getGoal(void)
	    {
		return m_goal;
	    }
	    
	    /** \brief A problem is trivial if the given starting state already
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

	    SpaceInformationPtr  m_si;
	    std::vector<State*>  m_startStates;
	    GoalPtr              m_goal;
	    
	    msg::Interface       m_msg;
	};
    }    
}

#endif

