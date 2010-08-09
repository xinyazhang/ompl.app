/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

#include "ompl/geometric/planners/CheckSettings.h"
#include "ompl/util/Exception.h"
#include "ompl/util/Console.h"
#include <limits>

/** \brief If default values are to be used for the maximum
    length of motions, this constant defines what fraction of
    the space extent (computed with
    ompl::base::SpaceInformation::estimateExtent()) is to be
    used as the maximum length of a motion */
static const double MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION = 0.2;
	
void ompl::geometric::checkMotionLength(const base::Planner *planner, double &length)
{ 
    if (length < std::numeric_limits<double>::epsilon())
    {
	length = planner->getSpaceInformation()->estimateExtent() * MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
	msg::Interface msg(planner->getName());
	msg.inform("Maximum motion extension distance is assumed to be %f", length);
    }
}

void ompl::geometric::checkProjectionEvaluator(const base::Planner *planner, base::ProjectionEvaluatorPtr &proj)
{
    if (!proj)
    {
	msg::Interface msg(planner->getName());
	msg.inform("Attempting to use default projection.");	
	proj = planner->getSpaceInformation()->getStateManifold()->getProjection();
    }
    if (!proj)
	throw Exception(planner->getName(), "No projection evaluator specified");
    proj->checkCellDimensions();
}
