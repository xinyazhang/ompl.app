/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll */

#ifndef OMPLAPP_DYNAMIC_CAR_PLANNING_
#define OMPLAPP_DYNAMIC_CAR_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl
{
    namespace app
    {
        /** \brief A class to facilitate planning for a generic second-order
            car model

            The dynamics of the second-order car are described by the following
            equations:
            \f{eqnarray*}{
            \dot x &=& v\cos\theta,\\
            \dot y &=& v\sin\theta,\\
            \dot\theta &=& \frac{vm}{L}\tan \phi,\\
            \dot v &=& u_0,\\
            \dot\phi &=& u_1,\f}
            where \f$v\f$ is the speed, \f$\phi\f$ the steering angle, the
            controls \f$(u_0,u_1)\f$ control their rate of change, \f$m\f$ is
            the mass of the car, and \f$L\f$ is the distance between the front
            and rear axle of the car. Both \f$m\f$ and \f$L\f$ are set to 1 by
            default.
        */
        class DynamicCarPlanning : public AppBase<CONTROL>
        {
        public:
            DynamicCarPlanning()
                : AppBase<CONTROL>(constructControlSpace(), Motion_2D), timeStep_(1e-2), lengthInv_(1.), mass_(1.)
            {
                name_ = std::string("Dynamic car");
                setDefaultBounds();
                getControlSpace()->setPropagationFunction(boost::bind(&DynamicCarPlanning::propagate, this, _1, _2, _3, _4));
            }
            ~DynamicCarPlanning()
            {
            }

            bool isSelfCollisionEnabled(void) const
            {
                return false;
            }
            virtual unsigned int getRobotCount(void) const
            {
                return 1;
            }
            virtual base::ScopedState<> getDefaultStartState(void) const;
            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
            {
                base::ScopedState<> r(si_);
                r = 0.0;
                r[0] = state[0];
                r[1] = state[1];
                r[2] = state[2];
                return r;
            }

            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubSpace(0);
            }

            double getVehicleLength()
            {
                return 1./lengthInv_;
            }
            void setVehicleLength(double length)
            {
                lengthInv_ = 1./length;
            }
            double getMass()
            {
                return mass_;
            }
            void setMass(double mass)
            {
                mass_ = mass;
            }
            virtual void setDefaultBounds();

        protected:

            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int index) const
            {
                return state->as<base::CompoundState>()->components[0];
            }

            void propagate(const base::State *from, const control::Control *ctrl,
                const double duration, base::State *result);
            virtual void ode(const base::State *q, const control::Control *ctrl, base::State *qdot);

            static control::ControlSpacePtr constructControlSpace(void)
            {
                return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 2));
            }
            static base::StateSpacePtr constructStateSpace(void)
            {
                base::StateSpacePtr stateSpace = base::StateSpacePtr(new base::CompoundStateSpace());
                base::RealVectorStateSpace *rv = new base::RealVectorStateSpace(2);
                base::RealVectorBounds b(2);
                b.setLow(-1); b.setHigh(1);
                rv->setBounds(b);
                stateSpace->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(new base::SE2StateSpace()), 1.);
                stateSpace->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(rv), .3);
                return stateSpace;
            }

            double timeStep_;
            double lengthInv_;
            double mass_;
        };

    }
}

#endif
