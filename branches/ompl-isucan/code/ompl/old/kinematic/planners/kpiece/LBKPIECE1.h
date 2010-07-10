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

#ifndef OMPL_KINEMATIC_PLANNERS_KPIECE_LBKPIECE1_
#define OMPL_KINEMATIC_PLANNERS_KPIECE_LBKPIECE1_

#include "ompl/base/Planner.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/GridB.h"
#include "ompl/kinematic/SpaceInformationKinematic.h"
#include <vector>


namespace ompl
{
    
    namespace kinematic
    {
	
	/**
	   @anchor kLBKPIECE1
	   with one level of discretization
	   
	   @par Short description
	   
	   KPIECE is a tree-based planner that uses a discretization
	   (multiple levels, in general) to guide the exploration of the
	   continous space. 
	   
	   @par External documentation
	   
	   Ioan A. Sucan, Lydia E. Kavraki, Kinodynamic Planning by
	   Interior-Exterior Cell Exploration, International Workshop on
	   the Algorithmic Foundations of Robotics, 2008.

	   @htmlonly
	   <a href="http://ioan.sucan.ro/files/pubs/wafr2008.pdf">http://ioan.sucan.ro/files/pubs/wafr2008.pdf</a>
	   @endhtmlonly
	*/
	
	/** \brief Lazy Bi-directional KPIECE */
	class LBKPIECE1 : public base::Planner
	{
	public:
	    
	    LBKPIECE1(SpaceInformationKinematic *si) : base::Planner(si),
					               m_sCore(si)
	    {
		m_type = base::PLAN_TO_GOAL_SAMPLEABLE_REGION;
		m_msg.setPrefix("LBKPIECE1");
		
		m_projectionEvaluator = NULL;
		m_projectionDimension = 0;
		m_selectBorderPercentage = 0.9;
		m_rho = 0.5;
		m_sampledGoalsCount = 0;
		m_addedStartStates = 0;
		m_tStart.grid.onCellUpdate(computeImportance, NULL);
		m_tGoal.grid.onCellUpdate(computeImportance, NULL);
	    }
	    
	    virtual ~LBKPIECE1(void)
	    {
		freeMemory();
	    }
	    
	    /** Set the projection evaluator. This class is able to
		compute the projection of a given state. The simplest
		option is to use an orthogonal projection; see
		OrthogonalProjectionEvaluator */
	    void setProjectionEvaluator(base::ProjectionEvaluator *projectionEvaluator)
	    {
		m_projectionEvaluator = projectionEvaluator;
	    }
	    
	    base::ProjectionEvaluator* getProjectionEvaluator(void) const
	    {
		return m_projectionEvaluator;
	    }
	    
	    /** Set the range the planner is supposed to use. This
		parameter greatly influences the runtime of the
		algorithm. It is probably a good idea to find what a good
		value is for each model the planner is used for. The basic
		idea of KPIECE is that it samples a random state around a
		state that was already added to the tree. The distance
		withing which this new state is sampled is controled by
		the range. This should be a value larger than 0.0 and less
		than 1.0 */
	    void setRange(double rho)
	    {
		m_rho = rho;
	    }
	    
	    /** Get the range the planner is using */
	    double getRange(void) const
	    {
		return m_rho;
	    }
	    
	    /** Set the percentage of time for focusing on the
		border. This is the minimum percentage used to select
		cells that are exterior (minimum because if 95% of cells
		are on the border, they will be selected with 95%
		chance, even if this percentage is set to 90%)*/
	    void setBorderPercentage(double bp)
	    {
		m_selectBorderPercentage = bp;
	    }
	    
	    double getBorderPercentage(void) const
	    {
		return m_selectBorderPercentage;
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
	    virtual void clear(void);

	    virtual void getStates(std::vector</*const*/ base::State*> &states) const;

	protected:
	    
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
		
		const base::State   *root;
		base::State         *state;
		Motion              *parent;
		bool                 valid;
		std::vector<Motion*> children;
	    };
	    
	    struct CellData
	    {
		CellData(void) : coverage(0.0), selections(1), score(1.0), iteration(0), importance(0.0)
		{
		}
		
		~CellData(void)
		{
		    for (unsigned int i = 0 ; i < motions.size() ; ++i)
			delete motions[i];
		}
		
		std::vector<Motion*> motions;
		double               coverage;
		unsigned int         selections;
		double               score;
		unsigned int         iteration;
		double               importance;
	    };
	    
	    struct OrderCellsByImportance
	    {
		bool operator()(const CellData * const a, const CellData * const b) const
		{
		    return a->importance > b->importance;
		}
	    };
	    
	    typedef GridB<CellData*, OrderCellsByImportance> Grid;
	    
	    struct TreeData
	    {
		TreeData(void) : grid(0), size(0), iteration(1)
		{
		}
		
		Grid         grid;
		unsigned int size;
		unsigned int iteration;
	    };
	    
	    static void computeImportance(Grid::Cell *cell, void*)
	    {
		CellData &cd = *(cell->data);
		cd.importance =  cd.score / ((cell->neighbors + 1) * cd.coverage * cd.selections);
	    }
	    
	    void freeMemory(void);
	    void freeGridMotions(Grid &grid);
	    
	    void addMotion(TreeData &tree, Motion* motion);
	    Motion* selectMotion(TreeData &tree);	
	    void removeMotion(TreeData &tree, Motion* motion);
	    bool isPathValid(TreeData &tree, Motion* motion);
	    bool checkSolution(bool start, TreeData &tree, TreeData &otherTree, Motion* motion, std::vector<Motion*> &solution);
	    
	    base::StateSamplerInstance                 m_sCore;

	    base::ProjectionEvaluator                 *m_projectionEvaluator;
	    unsigned int                               m_projectionDimension;
	    std::vector<double>                        m_cellDimensions;
	    
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
	    
	    double                                     m_selectBorderPercentage;
	    double                                     m_rho;	
	    RNG                                        m_rng;	
	};
	
    }
}


#endif
