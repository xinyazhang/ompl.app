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

#ifndef OMPL_BASE_STATE_
#define OMPL_BASE_STATE_

#include "ompl/base/General.h"
#include <cstdlib>

namespace ompl
{
    namespace base
    {

	/** Definition of a state: an array of doubles */
	class State
	{
	public:
	    
	    /** The flag datatype */
	    typedef int flagType;

	    /** Number of bit flags available in the state flags variable */
	    static const int FLAG_BITS_MAX;

	    /** Number of bit flags already used in the state flags variable */
	    static const int FLAG_BITS_USED;
	    
	    enum 
		{
		    NO_FLAGS       = 0,
		    SELF_ALLOCATED = 1
		};
	    
	    State(void) : flags(NO_FLAGS)
	    {
		values = NULL;
	    }
	    
	    /** Constructor that automatically allocates memory (memory will be automatically freed at destruction) */
	    State(const unsigned int dimension) : flags(SELF_ALLOCATED)
	    {
		values = new double[dimension];
	    }
	    
	    ~State(void)
	    {
		if ((flags & SELF_ALLOCATED) && values)
		    delete[] values;
	    }
	    
	    flagType flags;
	    double  *values;
	};
	
	/** Semantic information and bounds about each state component */
	struct StateComponent
	{
	    StateComponent(void)
	    {
		type = UNKNOWN;
		minValue = maxValue = resolution = 0.0;
	    }

		/** \todo do fuzzy comparison of double members */
		bool operator==(const StateComponent& sc) const
		{
			return type == sc.type && minValue == sc.minValue &&
				maxValue == sc.maxValue && resolution == sc.resolution;
		}
	    
	    enum
		{ UNKNOWN, LINEAR, WRAPPING_ANGLE, QUATERNION, DERIVATIVE }
		type;
	    double minValue;
	    double maxValue;
	    double resolution;
	};
	
	}
	    
}

#endif
