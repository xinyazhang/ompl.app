/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Ioan Sucan */

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include "ompl/kinematic/PathSimplifierKinematic.h"
#include "ompl/base/OrthogonalProjectionEvaluator.h"
#include "ompl/base/GoalState.h"

#include "ompl/kinematic/planners/kpiece/LBKPIECE1.h"
#include "ompl/kinematic/planners/kpiece/KPIECE1.h"
#include "ompl/kinematic/planners/sbl/SBL.h"
#include "ompl/kinematic/planners/rrt/RRT.h"
#include "ompl/kinematic/planners/rrt/RRTConnect.h"
#include "ompl/kinematic/planners/sbl/pSBL.h"
#include "ompl/kinematic/planners/rrt/pRRT.h"
#include "ompl/kinematic/planners/rrt/LazyRRT.h"
#include "ompl/kinematic/planners/est/EST.h"

#include "../../resources/config.h"
#include "environment2D.h"
#include <iostream>
#include <libgen.h>


using namespace ompl;

static const double SOLUTION_TIME = 1.0;


/** Declare a class used in validating states. Such a class definition is needed for any use
 * of a kinematic planner */
class myStateValidityChecker : public base::StateValidityChecker
{
public:

    myStateValidityChecker(base::SpaceInformation *si) : base::StateValidityChecker(si)
    {
    }
    
    virtual bool operator()(const base::State *state) const
    {
	/* planning is done in a continuous space, but our collision space representation is discrete */
	int x = (int)(state->values[0]);
	int y = (int)(state->values[1]);
	return m_grid[x][y] == 0; // 0 means valid state
    }
    
    void setGrid(const std::vector< std::vector<int> > &grid)
    {
	m_grid = grid;
    }
    
protected:
    
    std::vector< std::vector<int> > m_grid;

};

/** Declare a class used in evaluating distance between states (Manhattan distance) */
class myStateDistanceEvaluator : public base::StateDistanceEvaluator
{
public:

    myStateDistanceEvaluator(base::SpaceInformation *si) : base::StateDistanceEvaluator(si)
    {
    }
    
    virtual double operator()(const base::State *state1, const base::State *state2) const
    {
	/* planning is done in a continuous space, but our collision space representation is discrete */
	int x1 = (int)(state1->values[0]);
	int y1 = (int)(state1->values[1]);
	
	int x2 = (int)(state2->values[0]);
	int y2 = (int)(state2->values[1]);

	return abs(x1 - x2) + abs(y1 - y2);
    }
    
};

    
/** Space information */
class mySpaceInformation : public kinematic::SpaceInformationKinematic
{
public:
    mySpaceInformation(int width, int height) : kinematic::SpaceInformationKinematic()
    {
	// we have 2 dimensions : x, y
	m_stateDimension = 2;
	m_stateComponent.resize(2);

	// dimension 0 (x) spans between [0, width) 
	// dimension 1 (y) spans between [0, height) 
	// since sampling is continuous and we round down, we allow values until just under the max limit
	// the resolution is 1.0 since we check cells only

	m_stateComponent[0].minValue = 0.0;
	m_stateComponent[0].maxValue = (double)width - 0.000000001;
	m_stateComponent[0].resolution = 0.5;
	m_stateComponent[0].type = base::StateComponent::LINEAR;
	
	m_stateComponent[1].minValue = 0.0;
	m_stateComponent[1].maxValue = (double)height - 0.000000001;
	m_stateComponent[1].resolution = 0.5;
	m_stateComponent[1].type = base::StateComponent::LINEAR;
    }
};

/** A base class for testing planners */
class TestPlanner
{
public:
    TestPlanner(void)
    {
    }
    
    virtual ~TestPlanner(void)
    {
    }
    
    virtual bool execute(Environment2D &env, bool show = false, double *time = NULL, double *pathLength = NULL)
    {	 
	bool result = true;
	
	/* instantiate space information */
	mySpaceInformation *si = new mySpaceInformation(env.width, env.height);
	
	/* instantiate state validity checker */
	myStateValidityChecker *svc = new myStateValidityChecker(si);
	svc->setGrid(env.grid);
	
	/* instantiate state distance evaluator  */
	myStateDistanceEvaluator *sde = new myStateDistanceEvaluator(si);
	
	/* instantiate problem definition */
	base::ProblemDefinition *pdef = new base::ProblemDefinition(si);
	
	si->setStateValidityChecker(svc);
	si->setStateDistanceEvaluator(sde);
	si->setup();

	/* instantiate motion planner */
	base::Planner *planner = newPlanner(si);
	planner->setProblemDefinition(pdef);
	planner->setup();
	
	/* set the initial state; the memory for this is automatically cleaned by SpaceInformation */
	base::State *state = new base::State(2);
	state->values[0] = env.start.first;
	state->values[1] = env.start.second;
	pdef->addStartState(state);
	
	/* set the goal state; the memory for this is automatically cleaned by SpaceInformation */
	base::GoalState *goal = new base::GoalState(si);
	goal->state = new base::State(2);
	goal->state->values[0] = env.goal.first;
	goal->state->values[1] = env.goal.second;
	goal->threshold = 1e-3; // this is basically 0, but we want to account for numerical instabilities 
	pdef->setGoal(goal);
	
	/* start counting time */
	ompl::time::point startTime = ompl::time::now();	
	
	/* call the planner to solve the problem */
	if (planner->solve(SOLUTION_TIME))
	{
	    ompl::time::duration elapsed = ompl::time::now() - startTime;
	    if (time)
		*time += ompl::time::seconds(elapsed);
	    if (show)
		printf("Found solution in %f seconds!\n", ompl::time::seconds(elapsed));
	    
	    kinematic::PathKinematic *path = static_cast<kinematic::PathKinematic*>(goal->getSolutionPath());
	    
	    
	    /* make the solution more smooth */
	    kinematic::PathSimplifierKinematic *smoother = new kinematic::PathSimplifierKinematic(si);
	    smoother->setMaxSteps(50);
	    smoother->setMaxEmptySteps(10);

	    startTime = ompl::time::now();
	    smoother->reduceVertices(path);
	    elapsed = ompl::time::now() - startTime;
	    delete smoother;
	    if (time)
		*time += ompl::time::seconds(elapsed);
	    
	    if (show)
		printf("Simplified solution in %f seconds!\n", ompl::time::seconds(elapsed));

	    /* fill in values that were linearly interpolated */
	    si->interpolatePath(path);
	    
	    if (pathLength)
		*pathLength += path->states.size();

	    if (show)
	    {
		printEnvironment(std::cout, env);
		std::cout << std::endl;	    
	    }
	    
	    Environment2D temp = env;
	    /* display the solution */	    
	    for (unsigned int i = 0 ; i < path->states.size() ; ++i)
	    {
		int x = (int)(path->states[i]->values[0]);
		int y = (int)(path->states[i]->values[1]);
		if (temp.grid[x][y] == T_FREE || temp.grid[x][y] == T_PATH)
		    temp.grid[x][y] = T_PATH;
		else
		{
		    temp.grid[x][y] = T_ERROR;
		    result = false;
		}		
	    }
	    
	    if (show)
		printEnvironment(std::cout, temp);
	}
	else
	    result = false;
	
	// free memory for start states
	pdef->clearStartStates();
	
	// free memory for goal
	pdef->clearGoal();
	
	delete planner;
	delete pdef;
	delete svc;
	delete sde;
	delete si;
	return result;
    }
    
protected:
    
    virtual base::Planner* newPlanner(kinematic::SpaceInformationKinematic *si) = 0;
    
};

class RRTTest : public TestPlanner 
{
protected:

    base::Planner* newPlanner(kinematic::SpaceInformationKinematic *si)
    {
	kinematic::RRT *rrt = new kinematic::RRT(si);
	rrt->setRange(0.95);
	return rrt;
    }    
};

class RRTConnectTest : public TestPlanner 
{
protected:

    base::Planner* newPlanner(kinematic::SpaceInformationKinematic *si)
    {
	kinematic::RRTConnect *rrt = new kinematic::RRTConnect(si);
	rrt->setRange(0.95);
	return rrt;
    }    
};

class pRRTTest : public TestPlanner 
{
protected:

    base::Planner* newPlanner(kinematic::SpaceInformationKinematic *si)
    {
	kinematic::pRRT *rrt = new kinematic::pRRT(si);
	rrt->setRange(0.95);
	rrt->setThreadCount(4);
	return rrt;
    }    
};

class LazyRRTTest : public TestPlanner 
{
protected:

    base::Planner* newPlanner(kinematic::SpaceInformationKinematic *si)
    {
	kinematic::LazyRRT *rrt = new kinematic::LazyRRT(si);
	rrt->setRange(0.95);
	return rrt;
    }    
};

class SBLTest : public TestPlanner 
{
public:
    SBLTest(void)
    {
	ope = NULL;
    }

    virtual bool execute(Environment2D &env, bool show = false, double *time = NULL, double *pathLength = NULL)
    {
	bool result = TestPlanner::execute(env, show, time, pathLength);	
	if (ope)
	{
	    delete ope;	
	    ope = NULL;
	}
	return result;
    }
    
protected:
    
    base::Planner* newPlanner(kinematic::SpaceInformationKinematic *si)
    {
	kinematic::SBL *sbl = new kinematic::SBL(si);
	sbl->setRange(0.95);
	
	std::vector<unsigned int> projection;
	projection.push_back(0);
	projection.push_back(1);
	ope = new base::OrthogonalProjectionEvaluator(si, projection);
	
	std::vector<double> cdim;
	cdim.push_back(1);
	cdim.push_back(1);
	ope->setCellDimensions(cdim);
	
	sbl->setProjectionEvaluator(ope);

	return sbl;
    }
    
    base::OrthogonalProjectionEvaluator *ope;
    
};

class pSBLTest : public TestPlanner 
{
public:
    pSBLTest(void)
    {
	ope = NULL;
    }

    virtual bool execute(Environment2D &env, bool show = false, double *time = NULL, double *pathLength = NULL)
    {
	bool result = TestPlanner::execute(env, show, time, pathLength);	
	if (ope)
	{
	    delete ope;	
	    ope = NULL;
	}
	return result;
    }
    
protected:
    
    base::Planner* newPlanner(kinematic::SpaceInformationKinematic *si)
    {
	kinematic::pSBL *sbl = new kinematic::pSBL(si);
	sbl->setRange(0.95);
	sbl->setThreadCount(4);
	
	std::vector<unsigned int> projection;
	projection.push_back(0);
	projection.push_back(1);
	ope = new base::OrthogonalProjectionEvaluator(si, projection);
	
	std::vector<double> cdim;
	cdim.push_back(1);
	cdim.push_back(1);
	ope->setCellDimensions(cdim);
	
	sbl->setProjectionEvaluator(ope);

	return sbl;
    }
    
    base::OrthogonalProjectionEvaluator *ope;
    
};

class ESTTest : public TestPlanner 
{
public:
    ESTTest(void)
    {
	ope = NULL;
    }

    virtual bool execute(Environment2D &env, bool show = false, double *time = NULL, double *pathLength = NULL)
    {
	bool result = TestPlanner::execute(env, show, time, pathLength);	
	if (ope)
	{
	    delete ope;	
	    ope = NULL;
	}
	return result;
    }
    
protected:
    
    base::Planner* newPlanner(kinematic::SpaceInformationKinematic *si)
    {
	kinematic::EST *est = new kinematic::EST(si);
	est->setRange(0.75);
	
	std::vector<unsigned int> projection;
	projection.push_back(0);
	projection.push_back(1);
	ope = new base::OrthogonalProjectionEvaluator(si, projection);
		
	std::vector<double> cdim;
	cdim.push_back(1);
	cdim.push_back(1);
	ope->setCellDimensions(cdim);

	est->setProjectionEvaluator(ope);

	return est;
    }
    
    base::OrthogonalProjectionEvaluator *ope;
    
};

class KPIECE1Test : public TestPlanner 
{
public:
    KPIECE1Test(void)
    {
	ope = NULL;
    }

    virtual bool execute(Environment2D &env, bool show = false, double *time = NULL, double *pathLength = NULL)
    {
	bool result = TestPlanner::execute(env, show, time, pathLength);	
	if (ope)
	{
	    delete ope;	
	    ope = NULL;
	}
	return result;
    }
    
protected:
    
    base::Planner* newPlanner(kinematic::SpaceInformationKinematic *si)
    {
	kinematic::KPIECE1 *kpiece = new kinematic::KPIECE1(si);
	kpiece->setRange(0.95);
	
	std::vector<unsigned int> projection;
	projection.push_back(0);
	projection.push_back(1);
	ope = new base::OrthogonalProjectionEvaluator(si, projection);

	std::vector<double> cdim;
	cdim.push_back(1);
	cdim.push_back(1);
	ope->setCellDimensions(cdim);	

	kpiece->setProjectionEvaluator(ope);

	return kpiece;
    }
    
    base::OrthogonalProjectionEvaluator *ope;
    
};

class LBKPIECE1Test : public TestPlanner 
{
public:
    LBKPIECE1Test(void)
    {
	ope = NULL;
    }

    virtual bool execute(Environment2D &env, bool show = false, double *time = NULL, double *pathLength = NULL)
    {
	bool result = TestPlanner::execute(env, show, time, pathLength);	
	if (ope)
	{
	    delete ope;	
	    ope = NULL;
	}
	return result;
    }
    
protected:
    
    base::Planner* newPlanner(kinematic::SpaceInformationKinematic *si)
    {
	kinematic::LBKPIECE1 *kpiece = new kinematic::LBKPIECE1(si);
	kpiece->setRange(0.95);
	
	std::vector<unsigned int> projection;
	projection.push_back(0);
	projection.push_back(1);
	ope = new base::OrthogonalProjectionEvaluator(si, projection);

	std::vector<double> cdim;
	cdim.push_back(1);
	cdim.push_back(1);
	ope->setCellDimensions(cdim);	

	kpiece->setProjectionEvaluator(ope);

	return kpiece;
    }
    
    base::OrthogonalProjectionEvaluator *ope;
    
};

class PlanTest : public testing::Test
{
public:
    
    void runPlanTest(TestPlanner *p, double *success, double *avgruntime, double *avglength)
    {    
	double time   = 0.0;
	double length = 0.0;
	int    good   = 0;
	int    N      = 100;

	for (int i = 0 ; i < N ; ++i)
	    if (p->execute(env, false, &time, &length))
		good++;
	
	*success    = 100.0 * (double)good / (double)N;
	*avgruntime = time / (double)N;
	*avglength  = length / (double)N;

	if (verbose)
	{
	    printf("    Success rate: %f%%\n", *success);
	    printf("    Average runtime: %f\n", *avgruntime);
	    printf("    Average path length: %f\n", *avglength);
	}
    }
    
protected:
    
    PlanTest(void) 
    {
	verbose = true;
    }
    
    void SetUp(void)
    {
	/* load environment */
	boost::filesystem::path path(TEST_RESOURCES_DIR);
	path = path / "env1.txt";
	loadEnvironment(path.string().c_str(), env);
	
	if (env.width * env.height == 0)
	{
	    std::cerr << "The environment has a 0 dimension. Cannot continue" << std::endl;
	    FAIL();	    
	}
    }
    
    void TearDown(void)
    {
    }

    Environment2D env;
    bool          verbose;
};


TEST_F(PlanTest, kinematic_RRT)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new RRTTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.01);
    EXPECT_TRUE(avglength < 70.0);
}

TEST_F(PlanTest, kinematic_RRTConnect)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new RRTConnectTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.01);
    EXPECT_TRUE(avglength < 70.0);
}

TEST_F(PlanTest, kinematic_pRRT)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new pRRTTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.02);
    EXPECT_TRUE(avglength < 70.0);
}

TEST_F(PlanTest, kinematic_SBL)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new SBLTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.1);
    EXPECT_TRUE(avglength < 65.0);
}

TEST_F(PlanTest, kinematic_pSBL)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new pSBLTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.2);
    EXPECT_TRUE(avglength < 65.0);
}

TEST_F(PlanTest, kinematic_KPIECE1)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new KPIECE1Test();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.1);
    EXPECT_TRUE(avglength < 70.0);
}

TEST_F(PlanTest, kinematic_LBKPIECE1)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new LBKPIECE1Test();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.1);
    EXPECT_TRUE(avglength < 70.0);
}

TEST_F(PlanTest, kinematic_EST)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new ESTTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.1);
    EXPECT_TRUE(avglength < 65.0);
}

TEST_F(PlanTest, kinematic_LazyRRT)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new LazyRRTTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;
    
    EXPECT_TRUE(success >= 70.0);
    EXPECT_TRUE(avgruntime < 1.0);
    EXPECT_TRUE(avglength < 65.0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
