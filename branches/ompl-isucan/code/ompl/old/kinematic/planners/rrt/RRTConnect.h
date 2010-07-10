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

#ifndef OMPL_KINEMATIC_PLANNERS_RRT_RRT_CONNECT_
#define OMPL_KINEMATIC_PLANNERS_RRT_RRT_CONNECT_

#include "ompl/base/Planner.h"
#include "ompl/kinematic/SpaceInformationKinematic.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"

namespace ompl
{
    
    namespace kinematic
    {
	
	/**
	   @anchor kRRTC
	   
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
	    
	    RRTConnect(SpaceInformationKinematic *si) : base::Planner(si),
							m_sCore(si)
	    {
		m_type = base::PLAN_TO_GOAL_SAMPLEABLE_REGION;
		m_msg.setPrefix("RRTConnect");
		
		m_addedStartStates = 0;
		m_sampledGoalsCount = 0;
		
		m_tStart.setDistanceFunction(boost::bind(&RRTConnect::distanceFunction, this, _1, _2));
		m_tGoal.setDistanceFunction(boost::bind(&RRTConnect::distanceFunction, this, _1, _2));
		m_rho = 0.5;
	    }
	    
	    virtual ~RRTConnect(void)
	    {
		freeMemory();
	    }

	    virtual void getStates(std::vector</*const*/ base::State*> &states) const;

	    virtual bool solve(double solveTime);
	    
	    virtual void clear(void)
	    {
		freeMemory();
		m_tStart.clear();
		m_tGoal.clear();
		m_addedStartStates = 0;
		m_sampledGoalsCount = 0;
	    }
	    
	    /** \brief Set the range the planner is supposed to use.

		This parameter greatly influences the runtime of the
		algorithm. It is probably a good idea to find what a
		good value is for each model the planner is used
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
	    
	    class Motion
	    {
	    public:
		
		Motion(void) : root(NULL), state(NULL), parent(NULL)
		{
		    parent = NULL;
		    state  = NULL;
		}
		
		Motion(unsigned int dimension) : root(NULL), state(new base::State(dimension)), parent(NULL)
		{
		}
		
		~Motion(void)
		{
		    if (state)
			delete state;
		}
		
		const base::State *root;
		base::State       *state;
		Motion            *parent;
		
	    };
	    
	    typedef NearestNeighborsSqrtApprox<Motion*> TreeData;

	    struct TreeGrowingInfo
	    {
		std::vector<double>  range;
		base::State         *xstate;
		Motion              *xmotion;
		unsigned int         dim;
	    };
	    
	    enum GrowState 
		{
		    TRAPPED, ADVANCED, REACHED
		};
	    
	    void freeMemory(void)
	    {
		std::vector<Motion*> motions;
		m_tStart.list(motions);
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		    delete motions[i];
		
		m_tGoal.list(motions);
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		    delete motions[i];
	    }
	    
	    double distanceFunction(const Motion* a, const Motion* b) const
	    {
		return m_si->distance(a->state, b->state);
	    }

	    GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);
	    
	    base::StateSamplerInstance m_sCore;
	    
	    TreeData                   m_tStart;
	    TreeData                   m_tGoal;
	    unsigned int               m_addedStartStates;
	    unsigned int               m_sampledGoalsCount;

	    double                     m_rho;
	    RNG                        m_rng;
	};
	
    }
}

#endif
    
