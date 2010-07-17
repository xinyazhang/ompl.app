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

#include "ompl/base/GoalState.h"
#include "ompl/base/manifolds/RealVectorStateManifold.h"
#include "ompl/control/manifolds/RealVectorControlManifold.h"
#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/planners/kpiece/KPIECE1.h"

#include "../../resources/config.h"
#include "environment2D.h"
#include <iostream>
#include <libgen.h>

using namespace ompl;

static const double SOLUTION_TIME = 1.0;
static const double MAX_VELOCITY = 3.0;

/** Declare a class used in validating states. Such a class definition is needed for any use
 * of a kinematic planner */
class myStateValidityChecker : public base::StateValidityChecker
{
public:

    myStateValidityChecker(base::SpaceInformation *si, const std::vector< std::vector<int> > &grid) : base::StateValidityChecker(si)
    {
	setGrid(grid);
    }
    
    virtual bool isValid(const base::State *state) const
    {
	/* planning is done in a continuous space, but our collision space representation is discrete */
	int x = (int)(state->as<base::RealVectorState>()->values[0]);
	int y = (int)(state->as<base::RealVectorState>()->values[1]);
	
	if (x < 0 || y < 0 || x >= m_w || y >= m_h)
	    return false;

	return m_grid[x][y] == 0; // 0 means valid state
    }
    
    void setGrid(const std::vector< std::vector<int> > &grid)
    {
	m_grid = grid;
	m_w = m_grid.size();
	m_h = m_grid[0].size();	
    }
    
protected:
    
    std::vector< std::vector<int> > m_grid;
    int m_w, m_h;
    
};

class myStateManifold : public base::RealVectorStateManifold
{
public:
    
    myStateManifold() : base::RealVectorStateManifold(4)
    {
    }
    
    virtual double distance(const base::State *state1, const base::State *state2) const
    {
	/* planning is done in a continuous space, but our collision space representation is discrete */
	int x1 = (int)(state1->as<base::RealVectorState>()->values[0]);
	int y1 = (int)(state1->as<base::RealVectorState>()->values[1]);

	int x2 = (int)(state2->as<base::RealVectorState>()->values[0]);
	int y2 = (int)(state2->as<base::RealVectorState>()->values[1]);

	return abs(x1 - x2) + abs(y1 - y2);
    }
};

class myControlManifold : public control::RealVectorControlManifold
{
public:
    
    myControlManifold(const base::StateManifoldPtr &m) : control::RealVectorControlManifold(m, 2)
    {
    }
    
    virtual control::PropagationResult propagate(const base::State *state, const control::Control* control, const double duration, base::State *result) const
    {
       	result->as<base::RealVectorState>()->values[0] = state->as<base::RealVectorState>()->values[0] + duration * control->as<control::RealVectorControl>()->values[0];
	result->as<base::RealVectorState>()->values[1] = state->as<base::RealVectorState>()->values[1] + duration * control->as<control::RealVectorControl>()->values[1];
	result->as<base::RealVectorState>()->values[2] = control->as<control::RealVectorControl>()->values[0];
	result->as<base::RealVectorState>()->values[3] = control->as<control::RealVectorControl>()->values[1];
	m_stateManifold->enforceBounds(result);
	//	return SVC->isValid(state) ? control::PROPAGATION_START_VALID : control::PROPAGATION_START_INVALID;
	return control::PROPAGATION_START_UNKNOWN;
    }
    
    base::StateValidityChecker *SVC;
    
};
	
    
/** Space information */
control::SpaceInformationPtr mySpaceInformation(Environment2D &env)
{
    base::RealVectorStateManifold *sMan = new myStateManifold();
    
    base::RealVectorBounds sbounds(4);
    
    // dimension 0 (x) spans between [0, width) 
    // dimension 1 (y) spans between [0, height) 
    // since sampling is continuous and we round down, we allow values until just under the max limit
    // the resolution is 1.0 since we check cells only
    
    sbounds.low[0] = 0.0;
    sbounds.high[0] = (double)env.width - 0.000000001;
    
    sbounds.low[1] = 0.0;
    sbounds.high[1] = (double)env.height - 0.000000001;
    
    sbounds.low[2] = -MAX_VELOCITY;
    sbounds.high[2] = MAX_VELOCITY;
    
    sbounds.low[3] = -MAX_VELOCITY;
    sbounds.high[3] = MAX_VELOCITY;
    sMan->setBounds(sbounds);
    
    base::StateManifoldPtr sManPtr(sMan);
    
    myControlManifold *cMan = new myControlManifold(sManPtr);
    control::RealVectorBounds cbounds(2);
    
    cbounds.low[0] = -MAX_VELOCITY;
    cbounds.high[0] = MAX_VELOCITY;
    cbounds.low[1] = -MAX_VELOCITY;
    cbounds.high[1] = MAX_VELOCITY;
    cMan->setBounds(cbounds);
    
    control::SpaceInformationPtr si(new control::SpaceInformation(sManPtr, control::ControlManifoldPtr(cMan)));
    si->setMinMaxControlDuration(2, 25);
    si->setPropagationStepSize(0.25);
    
    si->setStateValidityChecker(base::StateValidityCheckerPtr(new myStateValidityChecker(si.get(), env.grid)));
    cMan->SVC = si->getStateValidityChecker().get();
    
    si->setup();
    
    return si;
}


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
	control::SpaceInformationPtr si = mySpaceInformation(env);
	base::ProblemDefinitionPtr pdef(new base::ProblemDefinition(si));

	/* instantiate motion planner */
	base::PlannerPtr planner = newPlanner(si);
	planner->setProblemDefinition(pdef);
	planner->setup();
	
	/* set the initial state; the memory for this is automatically cleaned by SpaceInformation */
	base::ScopedState<base::RealVectorState> state(si);
	state->values[0] = env.start.first;
	state->values[1] = env.start.second;
	state->values[2] = 0.0;
	state->values[3] = 0.0;
	pdef->addStartState(state);
		
	/* set the goal state; the memory for this is automatically cleaned by SpaceInformation */
	base::GoalState *goal = new base::GoalState(si);
	base::ScopedState<base::RealVectorState> gstate(si);
	gstate->values[0] = env.goal.first;
	gstate->values[1] = env.goal.second;
	gstate->values[2] = 0.0;
	gstate->values[3] = 0.0;
	goal->setState(gstate);
	goal->threshold = 1e-3; // this is basically 0, but we want to account for numerical instabilities 
	pdef->setGoal(base::GoalPtr(goal));

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
	    
	    control::PathControl *path = static_cast<control::PathControl*>(goal->getSolutionPath().get());
	    path->interpolate();
	    
	    if (!path->check())
		exit(1);
	    
	    elapsed = ompl::time::now() - startTime;
	    
	    if (time)
		*time += ompl::time::seconds(elapsed);
	    
	    if (pathLength)
		*pathLength += path->length();

	    if (show)
	    {
		printEnvironment(std::cout, env);
		std::cout << std::endl;	    
	    }
	    
	    Environment2D temp = env;
	    /* display the solution */	    
	    for (unsigned int i = 0 ; i < path->states.size() ; ++i)
	    {
		int x = (int)(path->states[i]->as<base::RealVectorState>()->values[0]);
		int y = (int)(path->states[i]->as<base::RealVectorState>()->values[1]);
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
	
	return result;
    }
    
protected:
    
    virtual base::PlannerPtr newPlanner(const control::SpaceInformationPtr &si) = 0;
    
};

class RRTTest : public TestPlanner 
{
protected:

    base::PlannerPtr newPlanner(const control::SpaceInformationPtr &si)
    {
	control::RRT *rrt = new control::RRT(si);
	return base::PlannerPtr(rrt);
    }    
};

class myProjectionEvaluator : public base::ProjectionEvaluator
{
public:
    myProjectionEvaluator(const base::StateManifoldPtr &manifold, const std::vector<double> &cellDimensions) : base::ProjectionEvaluator(manifold, cellDimensions)
    {
    }

    virtual unsigned int getDimension(void) const
    {
	return 2;
    }
        
    virtual void project(const base::State *state, base::EuclideanProjection *projection) const
    {
	projection[0] = state->as<base::RealVectorState>()->values[0];
	projection[1] = state->as<base::RealVectorState>()->values[1];
    }
};
    
class KPIECETest : public TestPlanner 
{
protected:

    base::PlannerPtr newPlanner(const control::SpaceInformationPtr &si)
    {
	control::KPIECE1 *kpiece = new control::KPIECE1(si);
	
	std::vector<double> cdim;
	cdim.push_back(1);
	cdim.push_back(1);
	base::ProjectionEvaluatorPtr ope(new myProjectionEvaluator(si->getStateManifold(), cdim));
	
	kpiece->setProjectionEvaluator(ope);
	
	return base::PlannerPtr(kpiece);
    }
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

TEST_F(PlanTest, controlRRT)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new RRTTest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.05);
    EXPECT_TRUE(avglength < 100.0);
}

TEST_F(PlanTest, controlKPIECE)
{
    double success    = 0.0;
    double avgruntime = 0.0;
    double avglength  = 0.0;
    
    TestPlanner *p = new KPIECETest();
    runPlanTest(p, &success, &avgruntime, &avglength);
    delete p;

    EXPECT_TRUE(success >= 99.0);
    EXPECT_TRUE(avgruntime < 0.05);
    EXPECT_TRUE(avglength < 5.0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
