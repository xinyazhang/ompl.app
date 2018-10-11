/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Original Author: Ioan Sucan */

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
	const char* dump_plan_fn = nullptr;
	if (argc >= 5) {
		dump_plan_fn = argv[4];
		std::cout << "Planner data will be dumped into file: " << dump_plan_fn << std::endl;
	}
    // plan in SE3
    app::SE3RigidBodyPlanning setup;

    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Puzzle-Part-1.0-32-centralized.obj";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Puzzle-CounterPart-1.0-32-centralized.obj";
#if 0
    constexpr double sx = -14.84;
    constexpr double sy = -3.92;
    constexpr double sz = -0.75;
    constexpr double srx = 0.0;
    constexpr double sry = 0.0;
    constexpr double srz = 1.0;
    constexpr double srt = 1.20427718388;
    constexpr double gx = -50.0;
    constexpr double gy = -3.92;
    constexpr double gz = -0.75;
    constexpr double grx = 0.0;
    constexpr double gry = 0.0;
    constexpr double grz = 1.0;
    constexpr double grt = 1.20427718388;
#endif
    constexpr double sx = 1;
    constexpr double sy = 7.0;
    constexpr double sz = 9.0;
    constexpr double srx = 0.528411106326 ;
    constexpr double sry = -0.405061752404;
    constexpr double srz = 0.746127790295 ;
    constexpr double srt = 1.35052791781  ;
    constexpr double gx = -5.0;
    constexpr double gy = 8;
    constexpr double gz = 7;
    constexpr double grt = 0.977384381117;
    constexpr double grx = 0;
    constexpr double gry = 0;
    constexpr double grz = 1;

    constexpr double cdres = 0.0001;

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
    b.setLow(-70.0);
    b.setHigh(70.0);
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
    if (setup.solve(3600 * 24 * days))
    {
        // simplify & print the solution
        setup.simplifySolution();
        setup.getSolutionPath().printAsMatrix(std::cout);
    }
    if (dump_plan_fn) {
	    base::PlannerData pdata(setup.getSpaceInformation());
	    setup.getPlanner()->getPlannerData(pdata);
	    std::ofstream fout(dump_plan_fn);
	    fout.precision(17);
	    printPlan(pdata, fout);
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
