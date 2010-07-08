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

#ifndef OMPL_KINEMATIC_PLANNERS_SBL_pSBL_
#define OMPL_KINEMATIC_PLANNERS_SBL_pSBL_

#include "ompl/base/Planner.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/StateSamplerArray.h"
#include "ompl/datastructures/Grid.h"
#include "ompl/kinematic/SpaceInformationKinematic.h"
#include <boost/thread/mutex.hpp>
#include <vector>


namespace ompl
{

    namespace kinematic
    {
	
	/**
	   @anchor kpSBL
	   
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


	/** \brief Parallel Single-query Bi-directional Lazy collision checking planner */
	class pSBL : public base::Planner
	{
	public:
	    
	    pSBL(SpaceInformationKinematic *si) : base::Planner(si),
		                                  m_sCoreArray(si)
	    {
		m_type = base::PLAN_TO_GOAL_STATE;
		m_msg.setPrefix("pSBL");
		m_projectionEvaluator = NULL;
		m_projectionDimension = 0;
		m_rho = 0.5;
		m_addedStartStates = 0;
		setThreadCount(2);
	    }
	    
	    virtual ~pSBL(void)
	    {		 
		freeMemory();
	    }
	    
	    /** \brief Set the projection evaluator. 

		This class is able to compute the projection of a
		given state. The simplest option is to use an
		orthogonal projection; see
		OrthogonalProjectionEvaluator */
	    void setProjectionEvaluator(base::ProjectionEvaluator *projectionEvaluator)
	    {
		m_projectionEvaluator = projectionEvaluator;
	    }
	    
	    /** \brief Get the projection evaluator. */
	    base::ProjectionEvaluator* getProjectionEvaluator(void) const
	    {
		return m_projectionEvaluator;
	    }
	    
	    /** \brief Set the range the planner is supposed to use.

		This parameter greatly influences the runtime of the
		algorithm. It is probably a good idea to find what a
		good value is for each model the planner is used
		for. The basic idea of SBL is that it samples a random
		state around a state that was already added to the
		tree. The distance withing which this new state is
		sampled is controled by the range. This should be a
		value larger than 0.0 and less than 1.0 */
	    void setRange(double rho)
	    {
		m_rho = rho;
	    }
	    
	    /** \brief Get the range the planner is using */
	    double getRange(void) const
	    {
		return m_rho;
	    }

	    /** \brief Set the number of threads the planner should use. Default is 2. */
	    void setThreadCount(unsigned int nthreads);
	    
	    /** \brief Get the thread count */
	    unsigned int getThreadCount(void) const
	    {
		return m_threadCount;
	    }
	    	   
	    virtual void setup(void)
	    {
		assert(m_projectionEvaluator);
		m_projectionDimension = m_projectionEvaluator->getDimension();
		assert(m_projectionDimension > 0);
		m_projectionEvaluator->getCellDimensions(m_cellDimensions);
		assert(m_cellDimensions.size() == m_projectionDimension);
		m_tStart.grid.setDimension(m_projectionDimension);
		m_tGoal.grid.setDimension(m_projectionDimension);
		Planner::setup();
	    }
	    
	    virtual bool solve(double solveTime);
	    
	    virtual void clear(void)
	    {
		freeMemory();
		
		m_tStart.grid.clear();
		m_tStart.size = 0;
		
		m_tGoal.grid.clear();
		m_tGoal.size = 0;
		
		m_removeList.motions.clear();
		
		m_addedStartStates = 0;
	    }

	    virtual void getStates(std::vector</*const*/ base::State*> &states) const;
	    
	protected:
	    
	    class Motion;	    
	    typedef std::vector<Motion*> MotionSet;	
	    
	    class Motion
	    {
	    public:
		
		Motion(void) : root(NULL), state(NULL), parent(NULL), valid(false)
		{
		}
		
		Motion(unsigned int dimension) : root(NULL), state(new base::State(dimension)), parent(NULL), valid(false)
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
		bool               valid;
		MotionSet          children;
		boost::mutex       lock;
	    };
	    
	    struct TreeData
	    {
		TreeData(void) : grid(0), size(0)
		{
		}

		Grid<MotionSet> grid;
		unsigned int    size;
		boost::mutex    lock;
	    };

	    struct SolutionInfo
	    {		
		std::vector<Motion*> solution;
		bool                 found;
		boost::mutex         lock;
	    };

	    struct PendingRemoveMotion
	    {
		TreeData *tree;
		Motion   *motion;
	    };
	    
	    struct MotionsToBeRemoved
	    {	
		std::vector<PendingRemoveMotion> motions;
		boost::mutex                     lock;
	    };    
	    
	    void threadSolve(unsigned int tid, time::point endTime, SolutionInfo *sol);
	    
	    void freeMemory(void)
	    {
		freeGridMotions(m_tStart.grid);
		freeGridMotions(m_tGoal.grid);
	    }
	    
	    void freeGridMotions(Grid<MotionSet> &grid)
	    {
		for (Grid<MotionSet>::iterator it = grid.begin(); it != grid.end() ; ++it)
		    for (unsigned int i = 0 ; i < it->second->data.size() ; ++i)
			delete it->second->data[i];
	    }
	    
	    void addMotion(TreeData &tree, Motion *motion);
	    Motion* selectMotion(RNG &rng, TreeData &tree);	
	    void removeMotion(TreeData &tree, Motion *motion, std::map<Motion*, bool> &seen);
	    bool isPathValid(TreeData &tree, Motion *motion);
	    bool checkSolution(RNG &rng, bool start, TreeData &tree, TreeData &otherTree, Motion *motion, std::vector<Motion*> &solution);
	    
	    
	    base::StateSamplerArray                    m_sCoreArray;
	    base::ProjectionEvaluator                 *m_projectionEvaluator;
	    unsigned int                               m_projectionDimension;
	    std::vector<double>                        m_cellDimensions;
	    
	    TreeData                                   m_tStart;
	    TreeData                                   m_tGoal;
	    
	    unsigned int                               m_addedStartStates;

	    MotionsToBeRemoved                         m_removeList;
	    boost::mutex                               m_loopLock;
	    boost::mutex                               m_loopLockCounter;
	    unsigned int                               m_loopCounter;
	    
	    double                                     m_rho;	

	    unsigned int                               m_threadCount;
	};
	
    }
}

#endif
    
