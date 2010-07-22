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

#ifndef OMPL_BASE_MANIFOLDS_REAL_VECTOR_STATE_MANIFOLD_
#define OMPL_BASE_MANIFOLDS_REAL_VECTOR_STATE_MANIFOLD_

#include "ompl/base/StateManifold.h"
#include <algorithm>
#include <vector>

namespace ompl
{
    namespace base
    {
	
	/** \brief The definition of a state in R<sup>n</sup> */
	class RealVectorState : public State
	{
	public:
	    /** \brief The value of the actual vector in R<sup>n</sup> */
	    double *values;
	};
	
	/** \brief The lower and upper bounds for an R<sup>n</sup> manifold */
	class RealVectorBounds
	{
	public:
	    RealVectorBounds(unsigned int dim)
	    {
		low.resize(dim, 0.0);
		high.resize(dim, 0.0);
	    }
	    
	    /** \brief Set the lower bound in each dimension to a specific value */
	    void setLow(double value)
	    {
		std::fill(low.begin(), low.end(), value);
	    }

	    /** \brief Set the upper bound in each dimension to a specific value */
	    void setHigh(double value)
	    {
		std::fill(high.begin(), high.end(), value);
	    }
	    
	    /** \brief Lower bound */
	    std::vector<double> low;

	    /** \brief Upper bound */
	    std::vector<double> high;
	};
	
	/** \brief Uniform sampler for the R<sup>n</sup> manifold */
	class RealVectorStateUniformSampler : public StateSampler
	{
	public:
	    
	    RealVectorStateUniformSampler(const StateManifold *manifold) : StateSampler(manifold)
	    {
	    }
	    
	    virtual void sample(State *state);	    
	    virtual void sampleNear(State *state, const State *near, const double distance);
	    
	};
	
	/** \brief A manifold representing R<sup>n</sup>. The distance function is the L2 norm. */
	class RealVectorStateManifold : public StateManifold
	{
	public:

	    /** \brief Define the type of state allocated by this manifold */
	    typedef RealVectorState StateType;

	    RealVectorStateManifold(unsigned int dim) : StateManifold(), dimension_(dim), stateBytes_(dim * sizeof(double)), bounds_(dim)
	    {
	    }
	    
	    virtual ~RealVectorStateManifold(void)
	    {	
	    }
	    
	    /** \brief Set the bounds of this manifold. This defines
		the range of the space in which sampling is
		performed. */
	    void setBounds(const RealVectorBounds &bounds);

	    /** \brief Get the bounds for this manifold */
	    const RealVectorBounds& getBounds(void) const
	    {
		return bounds_;
	    }
	    
	    /** \brief Get the dimension of the space */
	    virtual unsigned int getDimension(void) const;
	    
	    /** \brief Bring the state within the bounds of the state space */
	    virtual void enforceBounds(State *state) const;
	    	    
	    /** \brief Check if a state is inside the bounding box */
	    virtual bool satisfiesBounds(const State *state) const;
	    
	    /** \brief Copy a state to another */
	    virtual void copyState(State *destination, const State *source) const;
	    
	    /** \brief Computes distance to between two states */
	    virtual double distance(const State *state1, const State *state2) const;
	    
	    /** \brief Checks whether two states are equal */
	    virtual bool equalStates(const State *state1, const State *state2) const;

	    /** \brief Computes the state that lies at time t \in [0, 1] on the
		segment that connects the current state to the
		destination state */
	    virtual void interpolate(const State *from, const State *to, const double t, State *state) const;

	    /** \brief Allocate an instance of a uniform state sampler for this space */
	    virtual StateSamplerPtr allocUniformStateSampler(void) const;
	    
	    /** \brief Allocate a state that can store a point in the described space */
	    virtual State* allocState(void) const;
	    
	    /** \brief Free the memory of the allocated state */
	    virtual void freeState(State *state) const;

	    /** \brief Print a state to screen */
	    virtual void printState(const State *state, std::ostream &out) const;
	    
	    /** \brief Print the settings for this manifold to a stream */
	    virtual void printSettings(std::ostream &out) const;
	    
	    /** \brief Check if the manifold is configured correctly */
	    virtual void setup(void);
	    
	protected:
	    
	    unsigned int     dimension_;
	    std::size_t      stateBytes_;
	    RealVectorBounds bounds_;

	};
    }
}

#endif
