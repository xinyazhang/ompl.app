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

#ifndef OMPL_BASE_STATE_MANIFOLD_
#define OMPL_BASE_STATE_MANIFOLD_

#include "ompl/base/State.h"
#include "ompl/base/StateSampler.h"
#include "ompl/util/Console.h"
#include "ompl/util/ClassForward.h"
#include <boost/concept_check.hpp>
#include <boost/noncopyable.hpp>
#include <iostream>
#include <vector>

namespace ompl
{
    namespace base
    {
	
	ClassForward(StateManifold);
	
	/** \brief Representation of a space in which planning can be
	    performed. Topology specific sampling and interpolation
	    are defined. */
	class StateManifold : private boost::noncopyable
	{
	public:
	    
	    StateManifold(void)
	    {
	    }
	    
	    virtual ~StateManifold(void)
	    {
	    }
	    
	    /** \brief Cast this instance to a desired type. */
	    template<class T>
	    T* as(void)
	    {
		/** \brief Make sure the type we are casting to is indeed a state manifold */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateManifold*>));
		
		return static_cast<T*>(this);
	    }

	    /** \brief Cast this instance to a desired type. */
	    template<class T>
	    const T* as(void) const
	    {	
		/** \brief Make sure the type we are casting to is indeed a state manifold */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateManifold*>));
		
		return static_cast<const T*>(this);
	    }

	    /** \brief Get the dimension of the space */
	    virtual unsigned int getDimension(void) const = 0;

	    /** \brief Bring the state within the bounds of the state space */
	    virtual void enforceBounds(State *state) const = 0;
	    
	    /** \brief Check if a state is inside the bounding box */
	    virtual bool satisfiesBounds(const State *state) const = 0;

	    /** \brief Copy a state to another */
	    virtual void copyState(State *destination, const State *source) const = 0;
	    
	    /** \brief Computes distance to between two states */
	    virtual double distance(const State *state1, const State *state2) const = 0;
	    
	    /** \brief Checks whether two states are equal */
	    virtual bool equalStates(const State *state1, const State *state2) const = 0;

	    /** \brief Computes the state that lies at time t \in [0, 1] on the
		segment that connects the current state to the
		destination state */
	    virtual void interpolate(const State *from, const State *to, const double t, State *state) const = 0;
	    
	    /** \brief Set the allocator to use for a state sampler */
	    void setStateSamplerAllocator(const StateSamplerAllocator &ssa);
	    
	    /** \brief Allocate an instance of a uniform state sampler for this space */
	    virtual StateSamplerPtr allocUniformStateSampler(void) const = 0;

	    /** \brief Allocate an instance of a state sampler for this space. If setStateSamplerAllocator() was called,
		the specified allocator is used to produce the state sampler.  Otherwise, allocUniformStateSampler() is
		called. */
	    virtual StateSamplerPtr allocStateSampler(void) const;
	    
	    /** \brief Allocate a state that can store a point in the described space */
	    virtual State* allocState(void) const = 0;
	    
	    /** \brief Free the memory of the allocated state */
	    virtual void freeState(State *state) const = 0;
	    	    
	    /** \brief Print a state to a stream */
	    virtual void printState(const State *state, std::ostream &out) const;
	    
	    /** \brief Print the settings for this manifold to a stream */
	    virtual void printSettings(std::ostream &out) const;
	    
	    /** \brief Perform final setup steps. This function is automatically called by the SpaceInformation */
	    virtual void setup(void);
	    
	protected:
	    
	    msg::Interface        m_msg;
	    StateSamplerAllocator m_ssa;
	    
	};
	
    	class CompoundStateManifold : public StateManifold
	{
	public:
	    
	    /** \brief Define the type of state allocated by this manifold */
	    typedef CompoundState StateType;
	    
	    CompoundStateManifold(void) : StateManifold(), m_componentCount(0), m_locked(false)
	    {
	    }
	    
	    virtual ~CompoundStateManifold(void)
	    {
	    }

	    /** \brief Cast a component of this instance to a desired type. */
	    template<class T>
	    T* as(const unsigned int index) const
	    {
		/** \brief Make sure the type we are casting to is indeed a state manifold */
		BOOST_CONCEPT_ASSERT((boost::Convertible<T*, StateManifold*>));
		
		return static_cast<T*>(getSubManifold(index).get());
	    }
	    
	    /** \brief Adds a new manifold as part of the compound space. For computing distances within the compound
		space, the weight of the component also needs to be specified. */
	    virtual void addSubManifold(const StateManifoldPtr &component, double weight);
	    
	    /** \brief Get the number of manifolds that make up the compound manifold */
	    unsigned int getSubManifoldCount(void) const;
	    
	    /** \brief Get a specific manifold from the compound manifold */
	    const StateManifoldPtr& getSubManifold(const unsigned int index) const;
	    
	    /** \brief Get a specific manifold's weight from the compound manifold (used in distance computation) */
	    double getSubManifoldWeight(const unsigned int index) const;
	    
	    virtual unsigned int getDimension(void) const;

	    virtual void enforceBounds(State *state) const;
	    
	    virtual bool satisfiesBounds(const State *state) const;

	    virtual void copyState(State *destination, const State *source) const;
	    
	    virtual double distance(const State *state1, const State *state2) const;
	    
	    virtual bool equalStates(const State *state1, const State *state2) const;
	    
	    virtual void interpolate(const State *from, const State *to, const double t, State *state) const;
	    
	    virtual StateSamplerPtr allocStateSampler(void) const;

	    virtual StateSamplerPtr allocUniformStateSampler(void) const;
	    
	    virtual State* allocState(void) const;
	    
	    virtual void freeState(State *state) const;	 

	    virtual void printState(const State *state, std::ostream &out) const;

	    virtual void printSettings(std::ostream &out) const;

	protected:
	    
	    /** \brief Lock this manifold. This means no further
	     manifolds can be added as components.  This function can
	     be for instance called from the constructor of a manifold
	     that inherits from CompoundStateManifold to prevent the
	     user to add further components. */
	    void lock(void);
	    
	    std::vector<StateManifoldPtr> m_components;
	    unsigned int                  m_componentCount;
	    std::vector<double>           m_weights;
	    bool                          m_locked;
	    
	};
    }
}

#endif
