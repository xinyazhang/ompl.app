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

#ifndef OMPL_DYNAMIC_SPACE_INFORMATION_CONTROLS_PHYSICS_
#define OMPL_DYNAMIC_SPACE_INFORMATION_CONTROLS_PHYSICS_

#include "ompl/dynamic/SpaceInformationControls.h"
#include "ompl/dynamic/StateForwardPropagator.h"


namespace ompl
{

    namespace dynamic
    {
	
	/** Space information useful for kinematic planning */
	class SpaceInformationControlsPhysics : public SpaceInformationControls
	{
	public:
	    
	    /** Constructor; setup() needs to be called as well, before use */
	    SpaceInformationControlsPhysics(void) : SpaceInformationControls()
	    {
		m_stateForwardPropagator = NULL;
	    }
	    
	    /** Destructor */
	    virtual ~SpaceInformationControlsPhysics(void)
	    {
	    }

	    /** Set the instance of the validity checker to use. No memory freeing is performed. */
	    void setStateForwardPropagator(StateForwardPropagatorWithContacts *sfp)
	    {
		m_stateForwardPropagator = sfp;
	    }
	    
	    /** Return the instance of the used state validity checker */
	    StateForwardPropagatorWithContacts* getStateForwardPropagator(void) const
	    {
		return m_stateForwardPropagator;
	    }
	    
	    /** Propagate the system forward in time, given a starting state, a control and a duration. The result is a state. */
	    void propagateForward(const base::State *begin, const Control *ctrl, unsigned int steps, base::State *end) const
	    {
		StateForwardPropagatorWithContacts::Options opt(false, false);
		StateForwardPropagatorWithContacts::Result  res(end);
		(*m_stateForwardPropagator)(begin, ctrl, steps, m_resolution, opt, res);
	    }
	    
	    /** Propagate the system forward in time, given a starting state, a control and a duration. The result is the list of states along the path.
	     *  The number of added states is returned. No invalid states are returned. Validity is tested with both the internal
	     *  contact detection and the state validator. If no collision was found  */
	    unsigned int propagateForward(const base::State *begin, const Control *ctrl, unsigned int steps, std::vector<base::State*> states, bool alloc) const;
	    
	    /** Perform additional tasks to finish the initialization of
		the space information */
	    virtual void setup(void);
	    
	protected:
	    
	    StateForwardPropagatorWithContacts *m_stateForwardPropagator;
	    
	};
    }
    
}

#endif
