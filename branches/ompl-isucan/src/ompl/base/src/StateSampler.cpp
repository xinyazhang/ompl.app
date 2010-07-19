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

#include "ompl/base/StateSampler.h"
#include "ompl/base/StateManifold.h"

void ompl::base::CompoundStateSampler::addSampler(const StateSamplerPtr &sampler, double weightImportance)
{
    samplers_.push_back(sampler);
    weightImportance_.push_back(weightImportance);
    samplerCount_ = samplers_.size();
}

void ompl::base::CompoundStateSampler::sample(State *state)
{
    State **comps = static_cast<CompoundState*>(state)->components;
    for (unsigned int i = 0 ; i < samplerCount_ ; ++i)
	samplers_[i]->sample(comps[i]);
}

void ompl::base::CompoundStateSampler::sampleNear(State *state, const State *near, const double distance)
{    
    State **comps = static_cast<CompoundState*>(state)->components;
    State **nearComps = static_cast<const CompoundState*>(near)->components;
    for (unsigned int i = 0 ; i < samplerCount_ ; ++i)
	samplers_[i]->sampleNear(comps[i], nearComps[i], weightImportance_[i] * distance);
}
