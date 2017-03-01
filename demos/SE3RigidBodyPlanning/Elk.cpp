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

#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/pdst/PDST.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>

using namespace ompl;

int main(int argc, char* argv[])
{
	std::cout << "-----BEGIN-----" << std::endl;
	if (argc < 3) {
		std::cout << "INVALID NUMBER OF ARGUMENTS, EXITING" << std::endl;
		return -1;
	}
	int planner_id = atoi(argv[1]);
	int sampler_id = atoi(argv[2]);
    // plan in SE3
    app::SE3RigidBodyPlanning setup;

    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Elk-Part-Possion(12,8,4,1.05).ply";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Elk-CounterPart-Possion(12,8,4,1.05).ply";
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
    constexpr double cdres = 0.0001;

    switch (planner_id) {
	    case 0:
		    setup.setPlanner(std::make_shared<geometric::RRTConnect>(setup.getSpaceInformation()));
		    break;
	    case 1:
		    setup.setPlanner(std::make_shared<geometric::RRT>(setup.getSpaceInformation()));
		    break;
	    case 2:
		    setup.setPlanner(std::make_shared<geometric::BKPIECE1>(setup.getSpaceInformation()));
		    break;
	    case 3:
		    setup.setPlanner(std::make_shared<geometric::LBKPIECE1>(setup.getSpaceInformation()));
		    break;
	    case 4:
		    setup.setPlanner(std::make_shared<geometric::KPIECE1>(setup.getSpaceInformation()));
		    break;
	    case 5:
		    setup.setPlanner(std::make_shared<geometric::SBL>(setup.getSpaceInformation()));
		    break;
	    case 6:
		    setup.setPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));
		    break;
	    case 7:
		    setup.setPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));
		    break;
	    case 8:
		    setup.setPlanner(std::make_shared<geometric::BITstar>(setup.getSpaceInformation()));
		    break;
	    case 9:
		    setup.setPlanner(std::make_shared<geometric::PDST>(setup.getSpaceInformation()));
		    break;
	    default:
		    break;
    };
    switch (sampler_id) {
	    case 0:
		    setup.getSpaceInformation()->setValidStateSamplerAllocator(
				    [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr
				    {
				    return std::make_shared<base::UniformValidStateSampler>(si);
				    });
		    break;
	    case 1:
		    setup.getSpaceInformation()->setValidStateSamplerAllocator(
				    [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr
				    {
				    return std::make_shared<base::GaussianValidStateSampler>(si);
				    });
		    break;
	    case 2:
		    setup.getSpaceInformation()->setValidStateSamplerAllocator(
				    [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr
				    {
				    return std::make_shared<base::ObstacleBasedValidStateSampler>(si);
				    });
		    break;
	    case 3:
		    setup.getSpaceInformation()->setValidStateSamplerAllocator(
				    [](const base::SpaceInformation *si) -> base::ValidStateSamplerPtr
				    {
				    auto vss = std::make_shared<base::MaximizeClearanceValidStateSampler>(si);
				    vss->setNrImproveAttempts(5);
				    return vss;
				    });
		    break;
	    default:
		    break;
    }

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
    if (setup.solve(3600 * 48))
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
