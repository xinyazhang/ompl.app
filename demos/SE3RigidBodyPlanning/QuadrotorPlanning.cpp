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

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <omplapp/apps/QuadrotorPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

void quadrotorSetup(app::QuadrotorPlanning& setup)
{
    base::StateSpacePtr stateSpace(setup.getStateSpace());

    // set the bounds for the R^3 part of SE(3)
    base::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);
    stateSpace->as<base::CompoundStateSpace>()->as<base::SE3StateSpace>(0)->setBounds(bounds);

    // define start state
    base::ScopedState<base::SE3StateSpace> start(setup.getGeometricComponentStateSpace());
    start->setX(0.);
    start->setY(0.);
    start->setZ(0.);
    start->rotation().setIdentity();

    // define goal state
    base::ScopedState<base::SE3StateSpace> goal(setup.getGeometricComponentStateSpace());
    goal->setX(5.);
    goal->setY(5.);
    goal->setZ(5.);
    goal->rotation().setIdentity();

    // set the start & goal states
    setup.setStartAndGoalStates(
        setup.getFullStateFromGeometricComponent(start),
        setup.getFullStateFromGeometricComponent(goal), .5);
}

void quadrotorDemo(app::QuadrotorPlanning& setup)
{
    unsigned int i, j;
    std::vector<double> coords;

    std::cout<<"\n\n***** Planning for a " << setup.getName() << " *****\n" << std::endl;
    setup.setPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));

    // try to solve the problem
    if (setup.solve(40))
    {
        // print the (approximate) solution path: print states along the path
        // and controls required to get from one state to the next
        control::PathControl& path(setup.getSolutionPath());
        //path.interpolate(); // uncomment if you want to plot the path
        for (i=0; i<path.getStateCount(); ++i)
        {
            coords = base::ScopedState<>(setup.getStateSpace(), path.getState(i)).reals();
            for (j=0; j<coords.size(); ++j)
                std::cout << coords[j] << ' ';

            if (i==0)
                // null controls applied for zero seconds to get to start state
                std::cout << "0 0 0 0 0";
            else
            {
                // print controls and control duration needed to get from state i-1 to state i
                const double* c = path.getControl(i-1)->as<control::RealVectorControlSpace::ControlType>()->values;
                for (j=0; j<4; ++j)
                    std::cout << c[j] << ' ';
                std::cout << path.getControlDuration(i-1);
            }
            std::cout << std::endl;
        }

        if (!setup.haveExactSolutionPath())
        {
            std::cout << "Solution is approximate. Distance to actual goal is " <<
                setup.getGoal()->getDifference() << std::endl;
        }
    }
}

void quadrotorBenchmark(app::QuadrotorPlanning& setup)
{
    Benchmark::Request request(100., 10000., 10); // runtime (s), memory (MB), run count

    setup.setup ();

    Benchmark b(setup, setup.getName());
    b.addPlanner(base::PlannerPtr(new control::RRT(setup.getSpaceInformation())));
    b.addPlanner(base::PlannerPtr(new control::KPIECE1(setup.getSpaceInformation())));
    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int argc, char* argv[])
{
    app::QuadrotorPlanning quadrotor;
    quadrotorSetup(quadrotor);

    // If any command line arguments are given, solve the problem multiple
    // times with different planners and collect benchmark statistics.
    // Otherwise, solve the problem once and print the path.
    if (argc>1)
        quadrotorBenchmark(quadrotor);
    else
        quadrotorDemo(quadrotor);
    return 0;
}
