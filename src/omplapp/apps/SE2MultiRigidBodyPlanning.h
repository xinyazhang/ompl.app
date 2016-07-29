/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2012, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ryan Luna */

#include "omplapp/config.h"
#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE2StateSpace.h>

namespace ompl
{
    namespace app
    {

        /// @brief Wrapper for ompl::app::RigidBodyPlanning that plans for multiple rigid bodies in SE2.
        class SE2MultiRigidBodyPlanning : public AppBase<GEOMETRIC>
        {
        public:
            /// @brief Constructs an instance of multiple rigid bodies for 2D geometric planning.  n is the number of independent bodies in SE(2)
            SE2MultiRigidBodyPlanning(unsigned int n);

            virtual ~SE2MultiRigidBodyPlanning() {}

            /// @brief Constructs the default start state where all robots begin at their geometric center.
            /// If robots are all using the same mesh, this state is not likely to be valid.
            virtual base::ScopedState<> getDefaultStartState() const;

            virtual void inferEnvironmentBounds();

            virtual void inferProblemDefinitionBounds();

            bool isSelfCollisionEnabled() const
            {
                // Make sure that self collision is enabled to avoid inter-rigid body collision
                return true;
            }

            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
            {
                return state;
            }

            /// @brief Returns the state space corresponding for the indexth rigid body
            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(unsigned int index) const
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(index);
            }

            virtual const base::StateSpacePtr& getGeometricComponentStateSpace() const
            {
                // Return the zeroth component.  All components are the same.
                return getGeometricComponentStateSpace(0);
            }

            virtual unsigned int getRobotCount() const
            {
                return n_;
            }

        protected:
            /// @brief Returns the state corresponding to the indexth rigid body in the compound state
            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int index) const;

            /// @brief The number of independent rigid bodies to plan for
            unsigned int n_;
        };

    }
}

