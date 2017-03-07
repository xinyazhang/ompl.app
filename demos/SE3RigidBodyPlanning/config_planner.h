#ifndef DEOM_ALPHA_CONFIG_PLANNER_H
#define DEOM_ALPHA_CONFIG_PLANNER_H

#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <fstream>
#include <ompl/base/PlannerData.h>

void config_planner(ompl::app::SE3RigidBodyPlanning& setup, int planner_id, int sampler_id);
void printPlan(const ompl::base::PlannerData& pdata, std::ostream& fout);

#endif
