#ifndef CONFIG_PLANNER_H
#define CONFIG_PLANNER_H

#include <omplapp/apps/SE3RigidBodyPlanning.h>

void config_planner(ompl::app::SE3RigidBodyPlanning& setup, int planner_id, int sampler_id);

#endif
