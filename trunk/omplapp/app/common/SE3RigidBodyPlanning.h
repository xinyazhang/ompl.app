#ifndef OMPLAPP_COMMON_SE3_RIGID_BODY_PLANNING_
#define OMPLAPP_COMMON_SE3_RIGID_BODY_PLANNING_

#include "common/RigidBodyPlanning.h"
#include <ompl/base/manifolds/SE3StateManifold.h>

namespace ompl
{
    namespace app
    {
	
	class SE3RigidBodyPlanning : public RigidBodyPlanning
	{
	public:

	    SE3RigidBodyPlanning(void) : RigidBodyPlanning(base::StateManifoldPtr(new base::SE3StateManifold()))
	    {
	    }

	    virtual ~SE3RigidBodyPlanning()
	    {
	    }

	protected:
	    	    
	    virtual void inferEnvironmentBounds(const aiScene *scene);
	    virtual void inferProblemDefinitionBounds(void);

	    virtual base::StateValidityCheckerPtr allocStateValidityChecker(const aiScene *env, const aiScene *robot) const;
	};	
	
    }
}

#endif
