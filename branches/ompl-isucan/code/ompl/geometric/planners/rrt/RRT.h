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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"

namespace ompl
{
    
    namespace geometric
    {
	
	/**
	   @anchor gRRT
	   
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

	/** \brief Rapidly-exploring Random Trees */
	class RRT : public base::Planner
	{
	public:
	    
	    RRT(const base::SpaceInformationPtr &si) : base::Planner(si),
						       m_sCore(si->allocStateSampler())
	    {
		m_type = base::PLAN_TO_GOAL_ANY;
		m_msg.setPrefix("RRT");
		
		m_addedStartStates = 0;
		m_nn.setDistanceFunction(boost::bind(&RRT::distanceFunction, this, _1, _2));
		m_goalBias = 0.05;	    
		m_rho = 0.5;
	    }
	    
	    virtual ~RRT(void)
	    {
		freeMemory();
	    }

	    virtual void getPlannerData(base::PlannerData &data) const;

	    virtual bool solve(double solveTime);
	    
	    virtual void clear(void)
	    {
		freeMemory();
		m_nn.clear();
		m_addedStartStates = 0;
	    }
	    
	    /** \brief Set the goal bias

		In the process of randomly selecting states in
		the state space to attempt to go towards, the
		algorithm may in fact choose the actual goal state, if
		it knows it, with some probability. This probability
		is a real number between 0.0 and 1.0; its value should
		usually be around 0.05 and should not be too large. It
		is probably a good idea to use the default value. */
	    void setGoalBias(double goalBias)
	    {
		m_goalBias = goalBias;
	    }
	    
	    /** \brief Get the goal bias the planner is using */
	    double getGoalBias(void) const
	    {
		return m_goalBias;
	    }
	    
	    /** \brief Set the range the planner is supposed to use.

		This parameter greatly influences the runtime of
		the algorithm. It is probably a good idea to find what
		a good value is for each model the planner is used
		for. The range parameter influences how this @b qm
		along the path between @b qc and @b qr is chosen. @b
		qr may be too far, and it may not be best to have @b
		qm = @b qr all the time (range = 1.0 implies @b qm =
		@b qr. range should be less than 1.0). However, in a
		large space, it is also good to leave the neighborhood
		of @b qc (range = 0.0 implies @b qm = @b qc and no
		progress is made. rande should be larger than
		0.0). Multiple values of this range parameter should
		be tried until a suitable one is found. */
	    void setRange(double rho)
	    {
		m_rho = rho;
	    }
	    
	    /** \brief Get the range the planner is using */
	    double getRange(void) const
	    {
		return m_rho;
	    }
	    
	protected:
	    

	    /** \brief Representation of a motion

		This only contains pointers to parent motions as we
		only need to go backwards in the tree. */
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
	    
	    void freeMemory(void);
	    
	    double distanceFunction(const Motion* a, const Motion* b) const
	    {
		return m_si->distance(a->state, b->state);
	    }
	    
	    base::StateSamplerPtr                m_sCore;

	    NearestNeighborsSqrtApprox<Motion*>  m_nn;
	    unsigned int                         m_addedStartStates;
	    
	    double                               m_goalBias;
	    double                               m_rho;	
	    RNG                                  m_rng;	
	};
	
    }
}

#endif
    
