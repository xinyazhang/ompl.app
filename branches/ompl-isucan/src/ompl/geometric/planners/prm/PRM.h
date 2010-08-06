/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_PRM_
#define OMPL_GEOMETRIC_PLANNERS_PRM_PRM_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <vector>
#include <map>

namespace ompl
{
    
    namespace geometric
    {
	
	/**
	   @anchor gPRM
	   
	   @par Short description

	   Construct a roadmap of milestones that approximate the
	   connectivity of the state space.
	   
	   @par External documentation

	   Kavraki, L. E.; Svestka, P.; Latombe, J.-C.; Overmars,
	   M. H. (1996), "Probabilistic roadmaps for path planning in
	   high-dimensional configuration spaces", IEEE Transactions
	   on Robotics and Automation 12 (4): 566–580.	   
	   @htmlonly
	   <a href="http://en.wikipedia.org/wiki/Probabilistic_roadmap">http://en.wikipedia.org/wiki/Probabilistic_roadmap</a>
	   <br>
	   <a href="http://www.kavrakilab.org/robotics/prm.html">http://www.kavrakilab.org/robotics/prm.html</a>
	   @endhtmlonly 
	   
	*/

	/** \brief Rapidly-exploring Random Trees */
	class PRM : public base::Planner
	{
	public:
	    
	    PRM(const base::SpaceInformationPtr &si) : base::Planner(si)
	    {
		type_ = base::PLAN_TO_GOAL_STATE;
		msg_.setPrefix("PRM");
		
		maxNearestNeighbors_ = 10;
		componentCount_ = 0;
	    }
	    
	    virtual ~PRM(void)
	    {
		freeMemory();
	    }

	    /** \brief Set the maximum number of neighbors for which a
		connection to will be attempted when a new milestone
		is added */
	    void setMaxNearestNeighbors(unsigned int maxNearestNeighbors)
	    {
		maxNearestNeighbors_ = maxNearestNeighbors;		
	    }
	    
	    /** \brief Get the maximum number of neighbors for which a
		connection will be attempted when a new milestone is
		added */
	    unsigned int getMaxNearestNeighbors(void) const
	    {
		return maxNearestNeighbors_;
	    }
	    	    
	    virtual void getPlannerData(base::PlannerData &data) const;

	    virtual bool solve(double solveTime);
	    
	    virtual void clear(void);

	    /** \brief Set a different nearest neighbors datastructure */
	    template<template<typename T> class NN>
	    void setNearestNeighbors(void)
	    {
		nn_.reset(new NN<Milestone*>());
	    }

	    virtual void setup(void);
	    
	protected:
	    

	    /** \brief Representation of a milestone */
	    class Milestone
	    {
	    public:
		
		Milestone(void) : state(NULL)
		{
		}
		
		Milestone(const base::SpaceInformationPtr &si) : state(si->allocState())
		{
		}
		
		~Milestone(void)
		{
		}
		
		base::State            *state;
		unsigned long           component;
	        std::vector<Milestone*> adjacent;
		std::vector<double>     costs;
	    };
	    
	    void freeMemory(void);
	    Milestone* addMilestone(base::State *state);
	    void uniteComponents(Milestone *m1, Milestone *m2);
	    void growRoadmap(const std::vector<Milestone*> &start, const std::vector<Milestone*> &goal, double growTime, base::State *workState);
	    Milestone* haveSolution(const std::vector<Milestone*> &start, const std::vector<Milestone*> &goal);
	    bool findPath(const std::vector<Milestone*> &start, std::map<Milestone*, bool> &seen, std::vector<Milestone*> &path);
	    void constructSolution(const std::vector<Milestone*> &start, const std::vector<Milestone*> &goal);
	    
	    double distanceFunction(const Milestone* a, const Milestone* b) const
	    {
		return si_->distance(a->state, b->state);
	    }
	    
	    base::StateSamplerPtr                             sampler_;
	    boost::shared_ptr< NearestNeighbors<Milestone*> > nn_;
	    unsigned int                                      maxNearestNeighbors_;
	    std::map<unsigned long, unsigned long>            componentSizes_;
	    unsigned long                                     componentCount_;
	    RNG                                               rng_;
	};
	
    }
}

#endif
    
