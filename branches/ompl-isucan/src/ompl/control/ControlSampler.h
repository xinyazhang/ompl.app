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

#ifndef OMPL_CONTROL_CONTROL_SAMPLER_
#define OMPL_CONTROL_CONTROL_SAMPLER_

#include "ompl/control/Control.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl/util/ClassForward.h"
#include <vector>
#include <boost/function.hpp>

namespace ompl
{
    namespace control
    {
	
	ClassForward(ControlManifold);
	ClassForward(ControlSampler);
	
	/** \brief Abstract definition of a control sampler. */
	class ControlSampler
	{	    
	public:

	    ControlSampler(const ControlManifold *manifold) : manifold_(manifold)
	    {
	    }

	    virtual ~ControlSampler(void)
	    {
	    }
	    
	    /** \brief Sample a control */
	    virtual void sample(Control *control) = 0;

	    /** \brief Sample a number of steps to execute a control for */
	    virtual unsigned int sampleStepCount(unsigned int minSteps, unsigned int maxSteps);
	    
	    /** \brief Return a reference to the random number generator used */
	    RNG& getRNG(void)
	    {
		return rng_;
	    }
	    
	protected:
	    
	    const ControlManifold *manifold_;
	    RNG                    rng_;
	};

	/** \brief Definition of a compound control sampler. This is useful to construct samplers for compound controls. */
	class CompoundControlSampler : public ControlSampler
	{	    
	public:

	    /** \brief Constructor */
	    CompoundControlSampler(const ControlManifold* manifold) : ControlSampler(manifold) 
	    {
	    }
	    
	    /** \brief Destructor. This frees the added samplers as well. */
	    virtual ~CompoundControlSampler(void)
	    {
	    }
	    
	    /** \brief Add a sampler as part of the new compound
		sampler. This sampler is used to sample part of the
		compound control.  */
	    virtual void addSampler(const ControlSamplerPtr &sampler);
	    
	    /** \brief Sample a control. */
	    virtual void sample(Control *control);
	    
	protected:
	    
	    std::vector<ControlSamplerPtr> samplers_;
	    unsigned int                   samplerCount_;
	    
	};

	/** \brief Definition of a function that can allocate a control sampler */
	typedef boost::function<ControlSamplerPtr(const ControlManifold*)> ControlSamplerAllocator;
    }
}


#endif
