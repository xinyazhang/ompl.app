/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Xinya Zhang */
/* Original Author: Ioan Sucan */

#include "config_planner.h"
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>
#include <fstream>

using namespace ompl;

void usage()
{
	std::cout << R"xxx(Usage: Dual <Planner ID> <Sampler ID> <Running time in day> [Output] [Sample injection] [K in K-Nearest]
)xxx";
	usage_planner_and_sampler();
}

int main(int argc, char* argv[])
{
	std::cout << "-----BEGIN-----" << std::endl;
	if (argc > 1 && argv[1] == std::string("-h")) {
		usage();
		return 0;
	}
	if (argc < 4) {
		std::cout << "INVALID NUMBER OF ARGUMENTS, EXITING" << std::endl;
		usage();
		return -1;
	}
	int planner_id = atoi(argv[1]);
	int sampler_id = atoi(argv[2]);
	double days = atof(argv[3]);
	const char* dump_plan_fn = nullptr;
	const char* sample_inj_fn = nullptr;
	int KNearest = 1;
	if (argc >= 5) {
		dump_plan_fn = argv[4];
		std::cout << "Planner data will be dumped into file: " << dump_plan_fn << std::endl;
	}
	if (argc >= 6) {
		sample_inj_fn = argv[5];
		std::cout << "Samples from file " << sample_inj_fn << " will be injected " << std::endl;
	}
	if (argc >= 7) {
		KNearest = atoi(argv[6]);
		std::cout << "K-Nearest set to be " << KNearest << std::endl;
	}
	// plan in SE3
	app::SE3RigidBodyPlanning setup;

	std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/knotted_ring.dt.tcp.obj";
	std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/dual-g4.dt.tcp.obj";
	constexpr double sx = 7.85;
	constexpr double sy = 39.91;
	constexpr double sz = -0.82;
	constexpr double srx = 0.0;
	constexpr double sry = 0.0;
	constexpr double srz = 1.0;
	constexpr double srt = 1.57079632679;
	constexpr double gx = -0.15;
	constexpr double gy = -6.09;
	constexpr double gz = -0.82;
	constexpr double grx = 1.0;
	constexpr double gry = 0.0;
	constexpr double grz = 0.0;
	constexpr double grt = 0.0;
	constexpr double cdres = 0.005;

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
	config_planner(setup, planner_id, sampler_id, sample_inj_fn, KNearest);

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
	b.setHigh(0, 50); // x
	b.setHigh(1, 50); // y
	b.setHigh(2, 15); // z
	b.setLow(0, -15); // x
	b.setLow(1, -15); // y
	b.setLow(2, -15); // z
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
		// setup.simplifySolution(); // Well we need the precise solution for injection
		std::cout.precision(17);
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
