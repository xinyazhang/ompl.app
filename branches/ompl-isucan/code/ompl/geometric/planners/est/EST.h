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

#ifndef OMPL_GEOMETRIC_PLANNERS_EST_EST_
#define OMPL_GEOMETRIC_PLANNERS_EST_EST_

#include "ompl/datastructures/Grid.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include <vector>

namespace ompl
{
    
    namespace geometric
    {
	
	/**
	   @anchor gEST
	   
	   @par Short description
	   
	   EST attempts to detect the less explored area of the space
	   through the use of a grid imposed on a projection of the state
	   space. Using this information, EST continues tree expansion
	   primarily from less explored areas.
	   
	   @par External documentation
	   Path planning in expansive configuration spaces
	   Hsu, D.; Latombe, J.-C.; Motwani, R.
	   IEEE International Conference on Robotics and Automation, 1997.
	   Volume 3, Issue , 20-25 Apr 1997 Page(s):2719 - 2726 vol.3
	*/

	/** \brief Expansive Space Trees */
	class EST : public base::Planner
	{
	public:
	    
	    EST(const base::SpaceInformationPtr &si) : base::Planner(si),
						       m_sCore(si->allocStateSampler())
	    {
		m_type = base::PLAN_TO_GOAL_ANY;
		m_msg.setPrefix("EST");
		
		m_addedStartStates = 0;

		m_goalBias = 0.05;
		m_maxDistance = 0.0;
	    }
	    
	    virtual ~EST(void)
	    {
		freeMemory();
	    }
	    
	    virtual bool solve(double solveTime);
	    
	    virtual void clear(void)
	    {
		freeMemory();
		m_tree.grid.clear();
		m_tree.size = 0;
		m_addedStartStates = 0;
	    }
	    
	    /** In the process of randomly selecting states in the state
		space to attempt to go towards, the algorithm may in fact
		choose the actual goal state, if it knows it, with some
		probability. This probability is a real number between 0.0
		and 1.0; its value should usually be around 0.05 and
		should not be too large. It is probably a good idea to use
		the default value. */
	    void setGoalBias(double goalBias)
	    {
		m_goalBias = goalBias;
	    }
	    
	    /** Get the goal bias the planner is using */
	    double getGoalBias(void) const
	    {
		return m_goalBias;
	    }
	    
	    /** \brief Set the range the planner is supposed to use.

		This parameter greatly influences the runtime of the
		algorithm. It represents the maximum length of a
		motion to be added in the tree of motions. */
	    void setRange(double distance)
	    {
		m_maxDistance = distance;
	    }
	    
	    /** \brief Get the range the planner is using */
	    double getRange(void) const
	    {
		return m_maxDistance;
	    }
	    	    
	    /** Set the projection evaluator. This class is able to
		compute the projection of a given state. The simplest
		option is to use an orthogonal projection; see
		OrthogonalProjectionEvaluator */
	    void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
	    {
		m_projectionEvaluator = projectionEvaluator;
	    }
	    
	    const base::ProjectionEvaluatorPtr& getProjectionEvaluator(void) const
	    {
		return m_projectionEvaluator;
	    }
	    
	    virtual void setup(void);

	    virtual void getPlannerData(base::PlannerData &data) const;
	    
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
	    
	    typedef std::vector<Motion*> MotionSet;
	    
	    struct TreeData
	    {
		TreeData(void) : grid(0), size(0)
		{
		}
		
		Grid<MotionSet> grid;
		unsigned int    size;
	    };
	    
	    void freeMemory(void);
	    
	    void addMotion(Motion *motion);
	    Motion* selectMotion(void);
	    
	    base::StateSamplerPtr        m_sCore;
	    
	    TreeData                     m_tree;
	    unsigned int                 m_addedStartStates;
	    
	    base::ProjectionEvaluatorPtr m_projectionEvaluator;
	    
	    double                       m_goalBias;
	    double                       m_maxDistance;	
	    RNG                          m_rng;	
	};
	
    }
}

#endif
    
