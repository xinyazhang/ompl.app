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

#ifndef OMPL_CONTROL_MANIFOLDS_REAL_VECTOR_CONTROL_MANIFOLD_
#define OMPL_CONTROL_MANIFOLDS_REAL_VECTOR_CONTROL_MANIFOLD_

#include "ompl/control/ControlManifold.h"
#include <vector>

namespace ompl
{
    namespace control
    {
	
	/** \brief The definition of a control in R^n */
	class RealVectorControl : public Control
	{
	public:
	    double *values;
	};
	
	class RealVectorBounds
	{
	public:
	    RealVectorBounds(unsigned int dim)
	    {
		low.resize(dim, 0.0);
		high.resize(dim, 0.0);
	    }

	    std::vector<double> low;
	    std::vector<double> high;
	};
	
	/** \brief Uniform sampler for the R^n manifold */
	class RealVectorControlUniformSampler : public ControlSampler
	{
	public:
	    
	    RealVectorControlUniformSampler(const ControlManifold *manifold) : ControlSampler(manifold)
	    {
	    }
	    
	    virtual void sample(Control *control);	    
	};
	
	/** \brief A manifold representing R^n. The distance function is the L2 norm. */
	class RealVectorControlManifold : public ControlManifold
	{
	public:

	    /** \brief Define the type of control allocated by this manifold */
	    typedef RealVectorControl ControlType;

	    RealVectorControlManifold(const base::StateManifoldPtr &stateManifold, unsigned int dim) : ControlManifold(stateManifold), dimension_(dim), controlBytes_(dim * sizeof(double)), bounds_(dim)
	    {
	    }
	    
	    virtual ~RealVectorControlManifold(void)
	    {	
	    }
	    
	    void setBounds(const RealVectorBounds &bounds);

	    const RealVectorBounds& getBounds(void) const
	    {
		return bounds_;
	    }
	    
	    /** \brief Get the dimension of the space */
	    virtual unsigned int getDimension(void) const;
	    
	    /** \brief Copy a control to another */
	    virtual void copyControl(Control *destination, const Control *source) const;
	    
	    /** \brief Checks whether two controls are equal */
	    virtual bool equalControls(const Control *control1, const Control *control2) const;

	    /** \brief Allocate an instance of a uniform control sampler for this space */
	    virtual ControlSamplerPtr allocControlSampler(void) const;
	    
	    /** \brief Allocate a control that can store a point in the described space */
	    virtual Control* allocControl(void) const;
	    
	    /** \brief Free the memory of the allocated control */
	    virtual void freeControl(Control *control) const;

	    /** \brief Make the control have no effect if it were to be applied to a state for any amount of time. */
	    virtual void nullControl(Control *control) const;
	    
	    /** \brief Print a control to screen */
	    virtual void printControl(const Control *control, std::ostream &out) const;
	    
	    /** \brief Print the settings for this manifold to a stream */
	    virtual void printSettings(std::ostream &out) const;
	    
	protected:
	    
	    unsigned int     dimension_;
	    std::size_t      controlBytes_;
	    RealVectorBounds bounds_;
	    
	};
    }
}

#endif
