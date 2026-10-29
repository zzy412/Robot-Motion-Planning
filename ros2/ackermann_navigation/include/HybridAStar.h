#pragma once
#include "AStar.h"
namespace robot {
struct HybridConfig{double xyResolution=0.5,headingResolution=PI/12,primitiveDuration=0.8,dt=0.05,speed=1,maxSteer=PI/6,goalPositionTolerance=0.8,goalHeadingTolerance=PI/12;int maxExpanded=130000;bool allowReverse=true;};
PlanResult planHybridAStar(const GridMap& map,const VehicleModel& vehicle,State start,State goal,const HybridConfig& cfg={});
}
