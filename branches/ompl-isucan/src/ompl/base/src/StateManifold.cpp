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

#include "ompl/base/StateManifold.h"
#include "ompl/util/Exception.h"
#include <numeric>

void ompl::base::StateManifold::setup(void)
{
}

void ompl::base::StateManifold::setStateSamplerAllocator(const StateSamplerAllocator &ssa)
{
    ssa_ = ssa;
}

ompl::base::StateSamplerPtr ompl::base::StateManifold::allocStateSampler(void) const
{
    if (ssa_)
	return ssa_(this);
    else
	return allocUniformStateSampler();
}

void ompl::base::StateManifold::printState(const State *state, std::ostream &out) const
{
    out << "State instance: " << state << std::endl;
}

void ompl::base::StateManifold::printSettings(std::ostream &out) const
{
    out << "StateManifold instance: " << this << std::endl;
    printProjections(out);
}

void ompl::base::StateManifold::printProjections(std::ostream &out) const
{
    if (projections_.empty())
	out << "No registered projections" << std::endl;
    else
    {
	out << "Registered projections:" << std::endl;
	for (std::map<std::string, ProjectionEvaluatorPtr>::const_iterator it = projections_.begin() ; it != projections_.end() ; ++it)
	{
	    out << "  - ";
	    if (it->first.empty())
		out << "<default>";
	    else
		out << it->first;
	    out << " of dimension " << it->second->getDimension() << std::endl;
	}
    }
}

ompl::base::ProjectionEvaluatorPtr ompl::base::StateManifold::getProjection(void) const
{
    return getProjection("");
}

ompl::base::ProjectionEvaluatorPtr ompl::base::StateManifold::getProjection(const std::string &name) const
{
    std::map<std::string, ProjectionEvaluatorPtr>::const_iterator it = projections_.find(name);
    if (it != projections_.end())
	return it->second;
    else
	return ProjectionEvaluatorPtr();
}

void ompl::base::StateManifold::registerProjection(const std::string &name, const ProjectionEvaluatorPtr &projection)
{
    projections_[name] = projection;
}

void ompl::base::CompoundStateManifold::addSubManifold(const StateManifoldPtr &component, double weight)
{
    if (locked_)
	throw Exception("This manifold is locked. No further components can be added");
    if (weight < 0.0)
	throw Exception("Submanifold weight cannot be negative");    
    components_.push_back(component);
    weights_.push_back(weight);
    componentCount_ = components_.size();
}

unsigned int ompl::base::CompoundStateManifold::getSubManifoldCount(void) const
{
    return componentCount_;
}

const ompl::base::StateManifoldPtr& ompl::base::CompoundStateManifold::getSubManifold(const unsigned int index) const
{
    if (componentCount_ > index)
	return components_[index];
    else
	throw Exception("Submanifold index does not exist");
}

double ompl::base::CompoundStateManifold::getSubManifoldWeight(const unsigned int index) const
{
    if (componentCount_ > index)
	return weights_[index];
    else
	throw Exception("Submanifold index does not exist");
}

void ompl::base::CompoundStateManifold::setSubManifoldWeight(const unsigned int index, double weight)
{
    if (weight < 0.0)
	throw Exception("Submanifold weight cannot be negative");
    if (componentCount_ > index)
	weights_[index] = weight;
    else
	throw Exception("Submanifold index does not exist");
}

unsigned int ompl::base::CompoundStateManifold::getDimension(void) const
{
    unsigned int dim = 0;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	dim += components_[i]->getDimension();
    return dim;
}

void ompl::base::CompoundStateManifold::enforceBounds(State *state) const
{
    CompoundState *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->enforceBounds(cstate->components[i]);
}

bool ompl::base::CompoundStateManifold::satisfiesBounds(const State *state) const
{   
    const CompoundState *cstate = static_cast<const CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	if (!components_[i]->satisfiesBounds(cstate->components[i]))
	    return false;
    return true;
}

void ompl::base::CompoundStateManifold::copyState(State *destination, const State *source) const
{   
    CompoundState      *cdest = static_cast<CompoundState*>(destination);
    const CompoundState *csrc = static_cast<const CompoundState*>(source);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->copyState(cdest->components[i], csrc->components[i]);
}

double ompl::base::CompoundStateManifold::distance(const State *state1, const State *state2) const
{
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    double dist = 0.0;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	dist += weights_[i] * components_[i]->distance(cstate1->components[i], cstate2->components[i]);
    return dist;
}

bool ompl::base::CompoundStateManifold::equalStates(const State *state1, const State *state2) const
{	
    const CompoundState *cstate1 = static_cast<const CompoundState*>(state1);
    const CompoundState *cstate2 = static_cast<const CompoundState*>(state2);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	if (!components_[i]->equalStates(cstate1->components[i], cstate2->components[i]))
	    return false;
    return true;
}

void ompl::base::CompoundStateManifold::interpolate(const State *from, const State *to, const double t, State *state) const
{
    const CompoundState *cfrom  = static_cast<const CompoundState*>(from);
    const CompoundState *cto    = static_cast<const CompoundState*>(to);
    CompoundState       *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->interpolate(cfrom->components[i], cto->components[i], t, cstate->components[i]);
}

ompl::base::StateSamplerPtr ompl::base::CompoundStateManifold::allocUniformStateSampler(void) const
{
    double totalWeight = std::accumulate(weights_.begin(), weights_.end(), 0.0);
    if (totalWeight < std::numeric_limits<double>::epsilon())
	totalWeight = 1.0;	
    CompoundStateSampler *ss = new CompoundStateSampler(this);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	ss->addSampler(components_[i]->allocUniformStateSampler(), weights_[i] / totalWeight);
    return StateSamplerPtr(ss);
}

ompl::base::StateSamplerPtr ompl::base::CompoundStateManifold::allocStateSampler(void) const
{
    if (ssa_)
	return ssa_(this);
    else
    {
	double totalWeight = std::accumulate(weights_.begin(), weights_.end(), 0.0);
	if (totalWeight < std::numeric_limits<double>::epsilon())
	    totalWeight = 1.0;
	CompoundStateSampler *ss = new CompoundStateSampler(this);
	for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	    ss->addSampler(components_[i]->allocStateSampler(), weights_[i] / totalWeight);
	return StateSamplerPtr(ss);
    }
}

ompl::base::State* ompl::base::CompoundStateManifold::allocState(void) const
{
    CompoundState *state = new CompoundState();
    allocStateComponents(state);
    return static_cast<State*>(state);
}

void ompl::base::CompoundStateManifold::allocStateComponents(CompoundState *state) const
{   
    state->components = new State*[componentCount_];
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	state->components[i] = components_[i]->allocState();
}

void ompl::base::CompoundStateManifold::freeState(State *state) const 
{	
    CompoundState *cstate = static_cast<CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->freeState(cstate->components[i]);
    delete[] cstate->components;
    delete cstate;
}

void ompl::base::CompoundStateManifold::lock(void)
{
    locked_ = true;
}

void ompl::base::CompoundStateManifold::printState(const State *state, std::ostream &out) const
{
    out << "Compound state [" << std::endl;
    const CompoundState *cstate = static_cast<const CompoundState*>(state);
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->printState(cstate->components[i], out);
    out << "]" << std::endl;
}

void ompl::base::CompoundStateManifold::printSettings(std::ostream &out) const
{
    out << "Compound state manifold [" << std::endl;
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->printSettings(out);
    out << "]" << std::endl;
    printProjections(out);
}
	
void ompl::base::CompoundStateManifold::setup(void)
{
    for (unsigned int i = 0 ; i < componentCount_ ; ++i)
	components_[i]->setup();
    StateManifold::setup();
}
