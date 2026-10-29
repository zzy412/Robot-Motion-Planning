#pragma once
#include "GridMap.h"
#include <vector>
namespace robot {
struct PlanResult {bool success=false;std::vector<State> path;int expanded=0;double timeMs=0,length=0,maxCurvature=0,minCenterClearance=0;};
PlanResult planAStar(const GridMap& map,State start,State goal,double resolution=0.5);
void calculateMetrics(PlanResult& out,const GridMap& map);
}
