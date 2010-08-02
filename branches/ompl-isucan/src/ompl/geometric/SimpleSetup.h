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

#ifndef OMPL_GEOMETRIC_SIMPLE_SETUP_
#define OMPL_GEOMETRIC_SIMPLE_SETUP_

#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

namespace ompl
{

    namespace geometric
    {
		
	/** \brief Create the set of classes typically needed to solve a
	    geometric problem */
	class SimpleSetup
	{
	public:
	    
	    /** \brief Constructor needs the manifold needed for planning. */
	    explicit
	    SimpleSetup(const base::StateManifoldPtr &manifold) : configured_(false), planTime_(0.0), msg_("SimpleSetup")
	    {
		si_.reset(new base::SpaceInformation(manifold));
		pdef_.reset(new base::ProblemDefinition(si_));
		psk_.reset(new PathSimplifier(si_));
		
		psk_->setMaxSteps(50);
		psk_->setMaxEmptySteps(5);
	    }
	    
	    virtual ~SimpleSetup(void)
	    {
	    }
	    
	    /** \brief Get the current instance of the space information */
	    const base::SpaceInformationPtr& getSpaceInformation(void) const
	    {
		return si_;
	    }
	    
	    /** \brief Get the current instance of the problem definition */
	    const base::ProblemDefinitionPtr& getProblemDefinition(void) const
	    {
		return pdef_;
	    }
	    
	    /** \brief Get the current instance of the state manifold */
	    const base::StateManifoldPtr& getStateManifold(void) const
	    {
		return si_->getStateManifold();
	    }
	    
	    /** \brief Get the current instance of the state validity checker */
	    const base::StateValidityCheckerPtr& getStateValidityChecker(void) const
	    {
		return si_->getStateValidityChecker();
	    }

	    /** \brief Get the current goal definition */
	    const base::GoalPtr& getGoal(void) const
	    {
		return pdef_->getGoal();
	    }

	    /** \brief Get the current planner */
	    const base::PlannerPtr& getPlanner(void) const
	    {
		return planner_;
	    }

	    /** \brief Get the path simplifier */
	    const PathSimplifierPtr& getPathSimplifier(void) const
	    {
		return psk_;
	    }
	    
	    /** \brief Get the solution path. Throw an exception if no solution is available */
	    PathGeometric& getSolutionPath(void) const;
	    
	    
	    /** \brief Set the state validity checker to use */
	    void setStateValidityChecker(const base::StateValidityCheckerPtr &svc)
	    {
		si_->setStateValidityChecker(svc);
	    }
	    
	    /** \brief Set the state validity checker to use */
	    void setStateValidityChecker(const base::StateValidityCheckerFn &svc)
	    {
		si_->setStateValidityChecker(svc);
	    }

	    /** \brief Set the start and goal states to use. The state
		manifold is inferred, if not yet set. */
	    void setStartAndGoalStates(const base::ScopedState<> &start, const base::ScopedState<> &goal, const double threshold = std::numeric_limits<double>::epsilon())
	    {
		pdef_->setStartAndGoalStates(start, goal, threshold);
	    }
	    
	    /** \brief Add a starting state for planning. The state
		manifold is inferred, if not yet set. This call is not
		needed if setStartAndGoalStates() has been called. */
	    void addStartState(const base::ScopedState<> &state)
	    {
		pdef_->addStartState(state);
	    }
	    
	    /** \brief Set the goal for planning. This call is not
		needed if setStartAndGoalStates() has been called. */
	    void setGoal(const base::GoalPtr &goal)
	    {
		pdef_->setGoal(goal);
	    }

	    /** \brief Set the planner to use. If the planner is not
		set, an attempt is made to use the planner
		allocator. If no planner allocator is available
		either, a default planner is set. */
	    void setPlanner(const base::PlannerPtr &planner)
	    {
		planner_ = planner;
	    }
	    
	    /** \brief Set the planner allocator to use. This is only
		used if no planner has been set. This is optional -- a default
		planner will be used if no planner is otherwise specified. */
	    void setPlannerAllocator(const base::PlannerAllocator &pa)
	    {
		pa_ = pa;
	    }
	    	    
	    /** \brief Run the planner for a specified amount of time (default is 1 second) */
	    virtual bool solve(double time = 1.0)
	    {
		setup();
		time::point start = time::now();
		bool result = planner_->solve(time);
		planTime_ = time::seconds(time::now() - start);
		return result;
	    }
	    
	    /** \brief Get the amount of time (in seconds) spent during the last planning step */
	    double getLastPlanComputationTime(void) const
	    {
		return planTime_;
	    }
	    
	    /** \brief Attempt to simplify the current solution path */
	    void simplifySolution(void);
	    
	    /** \brief Clear all planning data */
	    virtual void clear(void);

	    /** \brief Print information about the current setup */
	    virtual void print(std::ostream &out = std::cout) const
	    {
		if (si_)
		    si_->print(out);
		if (pdef_)
		    pdef_->print(out);
	    }
	    
	    /** \brief This method will create the necessary classes
		for planning. The solve() method will call this
		function automatically. */
	    virtual void setup(void);

	protected:
	    
	    /// the created space information 
	    base::SpaceInformationPtr     si_;

	    /// the created problem definition 
	    base::ProblemDefinitionPtr    pdef_;

	    /// the maintained planner instance
	    base::PlannerPtr              planner_;

	    /// the optional planner allocator
	    base::PlannerAllocator        pa_;

	    /// the instance of the path simplifier 
	    PathSimplifierPtr             psk_;

	    /// flag indicating whether the classes needed for planning are set up
	    bool                          configured_;
	    
	    /// the amount of time the last planning step took
	    double                        planTime_;
	    
	    /// interface for console output
	    msg::Interface                msg_;
	    
	};
    }
    
}
#endif
