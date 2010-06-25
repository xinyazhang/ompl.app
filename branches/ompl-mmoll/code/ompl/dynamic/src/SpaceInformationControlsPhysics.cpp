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

#include "ompl/dynamic/SpaceInformationControlsPhysics.h"
#include <algorithm>

void ompl::dynamic::SpaceInformationControlsPhysics::setup(void)
{
    if (!m_stateValidityChecker)
	m_msg.error("No state validity checker defined");
    
    if (!m_stateForwardPropagator)
	m_msg.error("No forward propagation routine defined");    
	
    SpaceInformationControls::setup();
}

unsigned int ompl::dynamic::SpaceInformationControlsPhysics::propagateForward(const base::State *begin, const Control *ctrl, unsigned int steps, std::vector<base::State*> states, bool alloc) const
{
    if (alloc)
    {
	states.resize(steps + 1);
	states[0] = new base::State(m_stateDimension);
    }
    else
    {
	if (states.empty())
	    return 0;
	steps = std::min(steps, (unsigned int)states.size() - 1);
    }
    
    unsigned int st = 1;
    copyState(states.front(), begin);
    
    StateForwardPropagatorWithContacts::Options opt(true, false);
    StateForwardPropagatorWithContacts::Result  res;

    while (st <= steps)
    {
	if (alloc)
	    states[st] = new base::State(m_stateDimension);
	
	res.end = states[st];
	(*m_stateForwardPropagator)(begin, ctrl, 1, m_resolution, opt, res);
	bool stop = res.first_contact_step >= 0;
	if (!stop)
	    stop = !isValid(res.end);
	
	if (stop)
	{
	    // current state was in collision
	    if (alloc)
		delete states[st];
	    break;
	}
	st++;
    }
    
    return st;
}
