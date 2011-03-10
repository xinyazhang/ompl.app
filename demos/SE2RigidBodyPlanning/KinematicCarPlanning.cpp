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

#include <omplapp/apps/KinematicCarPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

void kinematicCarDemo(app::KinematicCarPlanning& setup)
{
    // plan for kinematic car in SE(2)
    const base::StateManifoldPtr &SE2 = setup.getGeometricComponentStateManifold();

    // set the bounds for the R^2 part of SE(2)
    base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    SE2->as<base::SE2StateManifold>()->setBounds(bounds);

    // define start state
    base::ScopedState<base::SE2StateManifold> start(SE2);
    start->setX(0);
    start->setY(0);
    start->setYaw(0);

    // define goal state
    base::ScopedState<base::SE2StateManifold> goal(SE2);
    goal->setX(2);
    goal->setY(2);
    goal->setYaw(M_PI);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // we call setup just so print() can show more information
    setup.setup();
    std::cout<<"\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;
    setup.print();

    // try to solve the problem
    if (setup.solve(20))
    {
        // simplify & print the solution
        setup.getSolutionPath().print(std::cout);
        if (!setup.haveExactSolutionPath())
        {
            std::cout << "Solution is approximate. Distance to actual goal is " <<
                setup.getGoal()->getDifference() << std::endl;
        }
    }
}

int main(int argc, char* argv[])
{
    app::KinematicCarPlanning regularCar;
    app::ReedsSheppCarPlanning rsCar;
    app::DubinsCarPlanning dCar;

    kinematicCarDemo(regularCar);
    kinematicCarDemo(rsCar);
    kinematicCarDemo(dCar);
    return 0;
}