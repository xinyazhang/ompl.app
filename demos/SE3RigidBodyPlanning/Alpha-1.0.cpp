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

#include "config_planner.h"
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>

using namespace ompl;

int main(int argc, char* argv[])
{
	std::cout << "-----BEGIN-----" << std::endl;
	if (argc < 4) {
		std::cout << "INVALID NUMBER OF ARGUMENTS, EXITING" << std::endl;
		return -1;
	}
	int planner_id = atoi(argv[1]);
	int sampler_id = atoi(argv[2]);
	double days = atof(argv[3]);
    // plan in SE3
    app::SE3RigidBodyPlanning setup;

    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/alpha-1.0.org.obj";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/alpha_env-1.0.org.obj";
    constexpr double sx = 16.97;
    constexpr double sy = 1.23;
    constexpr double sz = 10.2;
    constexpr double srx = 1.0;
    constexpr double sry = 0.0;
    constexpr double srz = 0.0;
    constexpr double srt = 0.0;
    constexpr double gx = 16.97;
    constexpr double gy = 1.23;
    constexpr double gz = 29.2;
    constexpr double grx = 1.0;
    constexpr double gry = 0.0;
    constexpr double grz = 0.0;
    constexpr double grt = 0.0;
    constexpr double cdres = 0.0001;

#if 0
#if 0
    auto planner = std::make_shared<geometric::RRT>(setup.getSpaceInformation());
#elif 0
    auto planner = std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation());
    planner->setRange(cdres * 5.0);
#elif 0
    auto planner = std::make_shared<geometric::PRM>(setup.getSpaceInformation());
#else
    auto planner = std::make_shared<geometric::SBL>(setup.getSpaceInformation());
#endif
    setup.setPlanner(planner);
#endif
    config_planner(setup, planner_id, sampler_id);

    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    // define start state
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(sx);
    start->setY(sy);
    start->setZ(sz);
    start->rotation().setAxisAngle(srx, sry, srz, srt);

    // define goal state
    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(gx);
    goal->setY(gy);
    goal->setZ(gz);
    goal->rotation().setAxisAngle(grx, gry, grz, grt);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(cdres);

    auto gcss = setup.getGeometricComponentStateSpace()->as<base::SE3StateSpace>();
    base::RealVectorBounds b = gcss->getBounds();
    b.setLow(-200.0);
    b.setHigh(200.0);
    gcss->setBounds(b);

#if 0
    {
	    auto p = setup.getSpaceInformation().get();
	    auto pc = dynamic_cast<const ompl::control::SpaceInformation*>(p);
	    auto pnc = const_cast<ompl::control::SpaceInformation*>(pc);
	    pnc->setPropagationStepSize(cdres);
    }
#endif
    // we call setup just so print() can show more information
    setup.setup();

    std::cout << "Trying to solve " << robot_fname << " v.s. " << env_fname << std::endl;
    setup.print();

#if 1
    // try to solve the problem
    if (setup.solve(3600 * 36 * days))
    {
        // simplify & print the solution
        setup.simplifySolution();
        setup.getSolutionPath().printAsMatrix(std::cout);
    }
#else
    int niter = 0;
    while (!setup.solve(2) || !setup.haveExactSolutionPath())
	    std::cout << "Iteration: " << niter++ << std::endl;
    setup.simplifySolution();
    setup.getSolutionPath().printAsMatrix(std::cout);
#endif

	std::cout << "-----FINAL-----" << std::endl;
    return 0;
}
