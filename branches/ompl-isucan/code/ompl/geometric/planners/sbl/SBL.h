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

#ifndef OMPL_GEOMETRIC_PLANNERS_SBL_SBL_
#define OMPL_GEOMETRIC_PLANNERS_SBL_SBL_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/Grid.h"
#include <vector>

namespace ompl
{

    namespace geometric
    {
	
	/**
	   @anchor gSBL
	   
	   @par Short description
	   
	   SBL is a tree-based motion planner that attempts to grow two
	   trees at once: one grows from the starting state and the other
	   from the goal state. Attempts are made to connect these trees
	   at every step of the expansion. If they are connected, a
	   solution path is obtained. However, this solution path is not
	   certain to be valid (the lazy part of the algorithm) so it is
	   checked for validity. If invalid parts are found, they are
	   removed from the tree and exploration of the state space
	   continues until a solution is found. 
	   
	   To guide the exploration, and additional grid data structure is
	   maintained. Grid cells contain states that have been previously
	   visited. When deciding which state to use for further
	   expansion, this grid is used and least filled grid cells have
	   most chances of being selected. The grid is usually imposed on
	   a projection of the state space. This projection needs to be
	   set before using the planner.
	   
	   @par External documentation
	   
	   G. Sanchez and J.C. Latombe.A Single-Query Bi-Directional
	   Probabilistic Roadmap Planner with Lazy Collision
	   Checking. Int. Symposium on Robotics Research (ISRR'01), Lorne,
	   Victoria, Australia, November 2001.
	*/

	/** \brief Single-Query Bi-Directional Probabilistic Roadmap
	   Planner with Lazy Collision Checking */
	class SBL : public base::Planner
	{
	public:
	    
	    SBL(const base::SpaceInformationPtr &si) : base::Planner(si),
						       m_sCore(si->allocStateSampler())
	    {
		m_type = base::PLAN_TO_GOAL_SAMPLEABLE_REGION;
		m_msg.setPrefix("SBL");
		
		m_sampledGoalsCount = 0;
		m_addedStartStates = 0;
		m_maxDistance = 0.0;
	    }
	    
	    virtual ~SBL(void)
	    {
		freeMemory();
	    }
	    
	    /** \brief Set the projection evaluator.  

		This class is able to compute the projection of a
		given state. The simplest option is to use an
		orthogonal projection; see
		OrthogonalProjectionEvaluator */
	    void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
	    {
		m_projectionEvaluator = projectionEvaluator;
	    }
	    
	    /** \brief Get the projection evaluator. */
	    const base::ProjectionEvaluatorPtr& getProjectionEvaluator(void) const
	    {
		return m_projectionEvaluator;
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

	    virtual void setup(void);
	    
	    virtual bool solve(double solveTime);
	    virtual void clear(void);
	    
	    virtual void getPlannerData(base::PlannerData &data) const;
	    
	protected:
	    
	    class Motion;	    
	    typedef std::vector<Motion*> MotionSet;	
	    
	    class Motion
	    {
	    public:
		
		Motion(void) : root(NULL), state(NULL), parent(NULL), valid(false)
		{
		}
		
		Motion(const base::SpaceInformationPtr &si) : root(NULL), state(si->allocState()), parent(NULL), valid(false)
		{
		}
		
		~Motion(void)
		{
		}
		
		const base::State *root;
		base::State       *state;
		Motion            *parent;
		bool               valid;
		MotionSet          children;
	    };
	    
	    struct TreeData
	    {
		TreeData(void) : grid(0), size(0)
		{
		}
		
		Grid<MotionSet> grid;
		unsigned int    size;
	    };
	    
	    void freeMemory(void)
	    {
		freeGridMotions(m_tStart.grid);
		freeGridMotions(m_tGoal.grid);
	    }
	    
	    void freeGridMotions(Grid<MotionSet> &grid);
	    
	    void addMotion(TreeData &tree, Motion *motion);
	    Motion* selectMotion(TreeData &tree);	
	    void removeMotion(TreeData &tree, Motion *motion);
	    bool isPathValid(TreeData &tree, Motion *motion);
	    bool checkSolution(bool start, TreeData &tree, TreeData &otherTree, Motion *motion, std::vector<Motion*> &solution);
	    
	    base::StateSamplerPtr                      m_sCore;
	    
	    base::ProjectionEvaluatorPtr               m_projectionEvaluator;
	    
	    TreeData                                   m_tStart;
	    TreeData                                   m_tGoal;

	    /// number of goal states that have been sampled already;
	    /// helps the planner know when to stop sampling the goal
	    /// region
	    unsigned int                               m_sampledGoalsCount;

	    /// number of added start states; if between subsequent
	    /// calls to solve() start states have been added to the
	    /// problem definition, this variable helps in determining
	    /// which ones to add to the start tree
	    unsigned int                               m_addedStartStates;
	    
	    double                                     m_maxDistance;	
	    RNG                                        m_rng;	
	};
	
    }
}

#endif
    
