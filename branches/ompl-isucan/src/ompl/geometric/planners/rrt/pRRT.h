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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_pRRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_pRRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/StateSamplerArray.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <boost/thread/mutex.hpp>

namespace ompl
{
    
    namespace geometric
    {
	
	/**
	   @anchor gpRRT
	   
	   @par Short description
	   
	   The basic idea of RRT is that it samples a random state @b qr
	   in the state space, then finds the state @b qc among the
	   previously seen states that is closest to @b qr and expands
	   from @b qc towards @b qr, until a state @b qm is reached and @b
	   qm is the new state to be visited.
	   
	   
	   @par External documentation

	   @htmlonly
	   <a href="http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree">http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree</a>
	   <br>
	   <a href="http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html">http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html</a>
	   @endhtmlonly 
	*/

	/** \brief Parallel RRT */
	class pRRT : public base::Planner
	{
	public:
	    
	    pRRT(const base::SpaceInformationPtr &si) : base::Planner(si),
							samplerArray_(si)
	    {
		type_ = base::PLAN_TO_GOAL_ANY;
		msg_.setPrefix("pRRT");

		setThreadCount(2);
		goalBias_ = 0.05;
		maxDistance_ = 0.0;
	    }
	    
	    virtual ~pRRT(void)
	    {
		freeMemory();
	    }
	    
	    virtual void getPlannerData(base::PlannerData &data) const;

	    virtual bool solve(double solveTime);
	    
	    virtual void clear(void);
	    
	    /** \brief Set the goal bias.

		In the process of randomly selecting states in the state
		space to attempt to go towards, the algorithm may in fact
		choose the actual goal state, if it knows it, with some
		probability. This probability is a real number between 0.0
		and 1.0; its value should usually be around 0.05 and
		should not be too large. It is probably a good idea to use
		the default value. */
	    void setGoalBias(double goalBias)
	    {
		goalBias_ = goalBias;
	    }
	    
	    /** \brief Get the goal bias the planner is using */
	    double getGoalBias(void) const
	    {
		return goalBias_;
	    }
	    
	    /** \brief Set the range the planner is supposed to use.

		This parameter greatly influences the runtime of the
		algorithm. It represents the maximum length of a
		motion to be added in the tree of motions. */
	    void setRange(double distance)
	    {
		maxDistance_ = distance;
	    }
	    
	    /** \brief Get the range the planner is using */
	    double getRange(void) const
	    {
		return maxDistance_;
	    }
	    	    
	    /** \brief Set the number of threads the planner should use. Default is 2. */
	    void setThreadCount(unsigned int nthreads);
	    
	    unsigned int getThreadCount(void) const
	    {
		return threadCount_;
	    }

	    /** \brief Set a different nearest neighbors datastructure */
	    template<template<typename T> class NN>
	    void setNearestNeighbors(void)
	    {
		nn_.reset(new NN<Motion*>());
	    }
	    
	    virtual void setup(void);
	    
	protected:
	    
	    class Motion
	    {
	    public:
		
		Motion(void) : state(NULL), parent(NULL)
		{
		}
		
		Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL)
		{
		}
		
		~Motion(void)
		{
		}
		
		base::State       *state;
		Motion            *parent;
		
	    };
	    
	    struct SolutionInfo
	    {
		Motion      *solution;
		Motion      *approxsol;
		double       approxdif;
		boost::mutex lock;
	    };

	    void threadSolve(unsigned int tid, time::point endTime, SolutionInfo *sol);
	    void freeMemory(void);
	    
	    double distanceFunction(const Motion* a, const Motion* b) const
	    {
		return si_->distance(a->state, b->state);
	    }
	    
	    base::StateSamplerArray                        samplerArray_;
	    boost::shared_ptr< NearestNeighbors<Motion*> > nn_;
	    boost::mutex                                   nnLock_;
	    
	    unsigned int                                   threadCount_;
	    
	    double                                         goalBias_;
	    double                                         maxDistance_;
	};
	
    }
}

#endif
