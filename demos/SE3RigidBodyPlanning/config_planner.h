#ifndef DEOM_ALPHA_CONFIG_PLANNER_H
#define DEOM_ALPHA_CONFIG_PLANNER_H

#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <fstream>
#include <ompl/base/PlannerData.h>

void config_planner(ompl::app::SE3RigidBodyPlanning& setup, int planner_id, int sampler_id, const char* = nullptr, int K = 1);
void printPlan(const ompl::base::PlannerData& pdata, std::ostream& fout);
void usage_planner_and_sampler();

enum {
	PLANNER_RRT_CONNECT = 0,
	PLANNER_RRT         = 1,
	PLANNER_BKPIECE1    = 2,
	PLANNER_LBKPIECE1   = 3,
	PLANNER_KPIECE1     = 4,
	PLANNER_SBL         = 5,
	PLANNER_EST         = 6,
	PLANNER_PRM         = 7,
	PLANNER_BITstar     = 8,
	PLANNER_PDST        = 9,
	PLANNER_TRRT        = 10,
	PLANNER_BiTRRT      = 11,
	PLANNER_LazyRRT     = 12,
	PLANNER_LazyLBTRRT  = 13,
	PLANNER_SPARS       = 14,
	PLANNER_ReRRT       = 15,
	PLANNER_RRTForest   = 16,
};

#endif
