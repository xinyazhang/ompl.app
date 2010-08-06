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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRT_CONNECT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRT_CONNECT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{
    
    namespace geometric
    {
	
	/**
	   @anchor gRRTC
	   
	   @par Short description
	   
	   The basic idea is to grow to RRTs, one from the start and
	   one from the goal, and attempt to connect them.
	   
	   
	   @par External documentation
	   @htmlonly
	   <a href="http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree">http://en.wikipedia.org/wiki/Rapidly-exploring_random_tree</a>
	   <br>
	   <a href="http://msl.cs.uiuc.edu/~lavalle/papers/KufLav00.pdf">http://msl.cs.uiuc.edu/~lavalle/papers/KufLav00.pdf</a>	   
	   @endhtmlonly
	*/

	/** \brief RRT-Connect (RRTConnect) */
	class RRTConnect : public base::Planner
	{
	public:
	    
	    RRTConnect(const base::SpaceInformationPtr &si) : base::Planner(si)
	    {
		type_ = base::PLAN_TO_GOAL_SAMPLEABLE_REGION;
		msg_.setPrefix("RRTConnect");
		
		maxDistance_ = 0.0;
	    }
	    
	    virtual ~RRTConnect(void)
	    {
		freeMemory();
	    }

	    virtual void getPlannerData(base::PlannerData &data) const;

	    virtual bool solve(double solveTime);
	    
	    virtual void clear(void);
	    
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

	    /** \brief Set a different nearest neighbors datastructure */
	    template<template<typename T> class NN>
	    void setNearestNeighbors(void)
	    {
		tStart_.reset(new NN<Motion*>());
		tGoal_.reset(new NN<Motion*>());
	    }
	    
	    virtual void setup(void);

	protected:
	    
	    class Motion
	    {
	    public:
		
		Motion(void) : root(NULL), state(NULL), parent(NULL)
		{
		    parent = NULL;
		    state  = NULL;
		}
		
		Motion(const base::SpaceInformationPtr &si) : root(NULL), state(si->allocState()), parent(NULL)
		{
		}
		
		~Motion(void)
		{
		}
		
		const base::State *root;
		base::State       *state;
		Motion            *parent;
		
	    };
	    
	    typedef boost::shared_ptr< NearestNeighbors<Motion*> > TreeData;

	    struct TreeGrowingInfo
	    {
		base::State         *xstate;
		Motion              *xmotion;
	    };
	    
	    enum GrowState 
		{
		    TRAPPED, ADVANCED, REACHED
		};
	    
	    void freeMemory(void);
	    
	    double distanceFunction(const Motion* a, const Motion* b) const
	    {
		return si_->distance(a->state, b->state);
	    }

	    GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);
	    
	    base::StateSamplerPtr      sampler_;
	    
	    TreeData                   tStart_;
	    TreeData                   tGoal_;
	    
	    double                     maxDistance_;
	    RNG                        rng_;
	};
	
    }
}

#endif
    
