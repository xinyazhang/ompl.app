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

#ifndef OMPL_KINEMATIC_PATH_SIMPLIFIER_KINEMATIC_
#define OMPL_KINEMATIC_PATH_SIMPLIFIER_KINEMATIC_

#include "ompl/kinematic/SpaceInformationKinematic.h"


namespace ompl
{

    namespace kinematic
    {
	
	/** \brief This class contains smoothers that can be applied to kinematic paths.

	 These are in fact routines that shorten the path, and do not
	 necessarily make it smoother.*/
	class PathSimplifierKinematic
	{
	public:
	    PathSimplifierKinematic(SpaceInformationKinematic *si)
	    {
		m_si = si;
		m_rangeRatio = 0.2;
		m_maxSteps = 10;
		m_maxEmptySteps = 3;
	    }
	    
	    virtual ~PathSimplifierKinematic(void)
	    {
	    }
	    
	    double getRangeRatio(void) const
	    {
		return m_rangeRatio;
	    }
	    
	    void setRangeRatio(double rangeRatio)
	    {
		m_rangeRatio = rangeRatio;
	    }
	    
	    unsigned int getMaxSteps(void) const
	    {
		return m_maxSteps;
	    }
	    
	    void setMaxSteps(unsigned int maxSteps)
	    {
		m_maxSteps = maxSteps;
	    }
	    
	    unsigned int getMaxEmptySteps(void) const
	    {
		return m_maxEmptySteps;
	    }
	    
	    void setMaxEmptySteps(unsigned int maxEmptySteps)
	    {
		m_maxEmptySteps = maxEmptySteps;
	    }
	    
	    /** Given a path, attempt to remove vertices from it while keeping the path valid */
	    virtual void reduceVertices(PathKinematic *path);
	    
	    /** Given a path, attempt to reduce redundant commands */
	    virtual void removeRedundantCommands(PathKinematic *path) const;
	    
	    /** Given a path, attempt to remove vertices from it while
	     * keeping the path valid.  Then, interpolate the path, to add
	     * more vertices and try to remove them again. This should
	     * produce smoother solutions. removeRedundantCommands is also
	     * called.  */
	    virtual void simplifyMax(PathKinematic *path);
	    
	protected:
	    
	    SpaceInformationKinematic *m_si;
	    RNG                        m_rng;
	    double                     m_rangeRatio;
	    unsigned int               m_maxSteps;
	    unsigned int               m_maxEmptySteps;
	};    
    }
}

#endif
