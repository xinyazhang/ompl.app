#include "config_planner.h"

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
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/LazyLBTRRT.h>
#include <ompl/geometric/planners/prm/SPARS.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>

using namespace ompl;

namespace {
template<typename Planner>
void set_planner(app::SE3RigidBodyPlanning& setup)
{
	setup.setPlanner(std::make_shared<Planner>(setup.getSpaceInformation()));
}

}

void config_planner(app::SE3RigidBodyPlanning& setup, int planner_id, int sampler_id)
{
	switch (planner_id) {
		case 0:
			set_planner<geometric::RRTConnect>(setup);
			break;
		case 1:
			set_planner<geometric::RRT>(setup);
			break;
		case 2:
			set_planner<geometric::BKPIECE1>(setup);
			break;
		case 3:
			set_planner<geometric::LBKPIECE1>(setup);
			break;
		case 4:
			set_planner<geometric::KPIECE1>(setup);
			break;
		case 5:
			set_planner<geometric::SBL>(setup);
			break;
		case 6:
			set_planner<geometric::EST>(setup);
			break;
		case 7:
			set_planner<geometric::PRM>(setup);
			break;
		case 8:
			set_planner<geometric::BITstar>(setup);
			break;
		case 9:
			set_planner<geometric::PDST>(setup);
			break;
		case 10:
			set_planner<geometric::TRRT>(setup);
			break;
		case 11:
			set_planner<geometric::BiTRRT>(setup);
			break;
		case 12:
			set_planner<geometric::LazyRRT>(setup);
			break;
		case 13:
			set_planner<geometric::LazyLBTRRT>(setup);
			break;
		case 14:
			set_planner<geometric::SPARS>(setup);
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
}