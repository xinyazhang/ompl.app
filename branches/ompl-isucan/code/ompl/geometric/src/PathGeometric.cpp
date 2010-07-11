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

#include "ompl/geometric/PathGeometric.h"

ompl::geometric::PathGeometric::PathGeometric(const PathGeometric &path) : base::Path(path.m_si)
{
    states.resize(path.states.size());
    for (unsigned int i = 0 ; i < states.size() ; ++i)
	states[i] = m_si->cloneState(path.states[i]);
}

void ompl::geometric::PathGeometric::freeMemory(void)
{
    for (unsigned int i = 0 ; i < states.size() ; ++i)
	m_si->freeState(states[i]);
}

double ompl::geometric::PathGeometric::length(void) const
{
    return (double)states.size();
}

bool ompl::geometric::PathGeometric::check(void) const
{
    bool result = true;
    if (states.size() > 0)
    {
	if (m_si->isValid(states[0]))
	{
	    int last = states.size() - 1;
	    for (int j = 0 ; result && j < last ; ++j)
		if (!m_si->checkMotion(states[j], states[j + 1]))
		    result = false;
	}
	else
	    result = false;
    }
    return result;
}

void ompl::geometric::PathGeometric::interpolate(double factor) 
{
    std::vector<base::State*> newStates;
    const int n1 = states.size() - 1;
    
    for (int i = 0 ; i < n1 ; ++i)
    {
	base::State *s1 = states[i];
	base::State *s2 = states[i + 1];
	
	newStates.push_back(s1);
	
	std::vector<base::State*> block;
	m_si->getMotionStates(s1, s2, block, factor, false, true);
	newStates.insert(newStates.end(), block.begin(), block.end());
    }
    
    newStates.push_back(states[n1]);
    states.swap(newStates);
}

