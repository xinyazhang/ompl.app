/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef OMPLAPP_SE3_RIGID_BODY_PLANNING_
#define OMPLAPP_SE3_RIGID_BODY_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/manifolds/SE3StateManifold.h>

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in SE2. */
        class SE3RigidBodyPlanning : public AppBase<GEOMETRIC>
        {
        public:

            SE3RigidBodyPlanning(void) : AppBase<GEOMETRIC>(base::StateManifoldPtr(new base::SE3StateManifold()), Motion_3D)
            {
                name_ = "Rigid body planning (3D)";
            }

            virtual ~SE3RigidBodyPlanning(void)
            {
            }

            bool isSelfCollisionEnabled(void) const
            {
                return false;
            }

            virtual base::ScopedState<> getDefaultStartState(void) const;

            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
            {
                return state;
            }

            virtual const base::StateManifoldPtr& getGeometricComponentStateManifold(void) const
            {
                return getStateManifold();
            }

            virtual const base::State* getGeometricComponentState(const base::State* state, unsigned int index) const
            {
                return state;
            }

            virtual unsigned int getRobotCount(void) const
            {
                return 1;
            }

        };

    }
}

#endif
