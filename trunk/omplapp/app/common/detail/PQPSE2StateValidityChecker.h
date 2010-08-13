#include "common/detail/PQPStateValidityChecker.h"
#include <ompl/base/manifolds/SE2StateManifold.h>

namespace ompl
{
    namespace app
    {
	
	class PQPSE2StateValidityChecker : public PQPStateValidityChecker
	{
	public:
	    
	    PQPSE2StateValidityChecker(const base::SpaceInformationPtr &si,
				       const aiScene *robot, const aiScene *obstacles):
		PQPStateValidityChecker(si, robot, obstacles)
	    {
	    }
	    
	    virtual bool isValid(const base::State *state) const;
	    
	};
	
    }
}
