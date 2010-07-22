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

#ifndef OMPL_BASE_MANIFOLDS_REAL_VECTOR_STATE_PROJECTIONS_
#define OMPL_BASE_MANIFOLDS_REAL_VECTOR_STATE_PROJECTIONS_

#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/manifolds/RealVectorStateManifold.h"
#include <valarray>

namespace ompl
{
    namespace base
    {
	
        /** \brief Definition for a class computing linear projections
	    (multiplication of a k-by-n matrix to the the
	    R<sup>n</sup> vector state to produce an R<sup>k</sup>
	    projection. The multiplication matrix needs to be supplied
	    as input. */
	class RealVectorLinearProjectionEvaluator : public ProjectionEvaluator
	{
	public:
	    
	    RealVectorLinearProjectionEvaluator(const StateManifold *manifold, 
						const std::vector<double> &cellDimensions,
						const std::vector< std::valarray<double> > &projection);
	    
	    RealVectorLinearProjectionEvaluator(const StateManifoldPtr &manifold,
						const std::vector<double> &cellDimensions,
						const std::vector< std::valarray<double> > &projection);
	    
	    virtual unsigned int getDimension(void) const
	    {
		return projection_.size();
	    }
	    
	    virtual void project(const State *state, EuclideanProjection &projection) const;
	    
	protected:
	    
	    /** \brief The projection matrix */
	    std::vector< std::valarray<double> > projection_;
	    
	};

        /** \brief Definition for a class computing a random linear projections */
	class RealVectorRandomLinearProjectionEvaluator : public RealVectorLinearProjectionEvaluator
	{
	public:
	    
	    RealVectorRandomLinearProjectionEvaluator(const StateManifold *manifold,
						      const std::vector<double> &cellDimensions) :
		RealVectorLinearProjectionEvaluator(manifold, cellDimensions,
						    computeProjection(manifold->getDimension(), cellDimensions.size()))
	    {
	    }
	    
	    RealVectorRandomLinearProjectionEvaluator(const StateManifoldPtr &manifold,
						      const std::vector<double> &cellDimensions) :
		RealVectorLinearProjectionEvaluator(manifold.get(), cellDimensions,
						    computeProjection(manifold->getDimension(), cellDimensions.size()))
	    {
	    }
	    
	protected:
	    
	    /** \brief Compute an orthonormal projection matrix of
		specified dimensions in a form that can be supplied to
		RealVectorLinearProjectionEvaluator. */
	    std::vector< std::valarray<double> > computeProjection(unsigned int from, unsigned int to) const;
	    
	};
	
	/** \brief Definition for a class computing orthogonal projections */
	class RealVectorOrthogonalProjectionEvaluator : public ProjectionEvaluator
	{
	public:
	    
	    RealVectorOrthogonalProjectionEvaluator(const StateManifold *manifold,
						    const std::vector<double> &cellDimensions,
						    const std::vector<unsigned int> &components);
	    RealVectorOrthogonalProjectionEvaluator(const StateManifoldPtr &manifold,
						    const std::vector<double> &cellDimensions,
						    const std::vector<unsigned int> &components);
	    
	    virtual unsigned int getDimension(void) const
	    {
		return components_.size();
	    }
	    
	    virtual void project(const State *state, EuclideanProjection &projection) const;
	    
	protected:
	    
	    /** \brief The set of components selected by the projection */
	    std::vector<unsigned int> components_;
	    
	};	

	/** \brief Define the identity projection */
	class RealVectorIdentityProjectionEvaluator : public ProjectionEvaluator
	{
	public:
	    RealVectorIdentityProjectionEvaluator(const StateManifold *manifold,
						  const std::vector<double> &cellDimensions);
	    RealVectorIdentityProjectionEvaluator(const StateManifoldPtr &manifold,
						  const std::vector<double> &cellDimensions);
	    
	    virtual unsigned int getDimension(void) const
	    {
		return manifold_->getDimension();
	    }
	    
	    virtual void project(const State *state, EuclideanProjection &projection) const;

	private:
	    
	    /** \brief The amount of data to copy from projection to state */
	    std::size_t copySize_;
	    
	};
	
    }
}
#endif
