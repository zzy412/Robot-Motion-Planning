#pragma once
#include "VehicleModel.h"
#include <vector>
namespace robot {
struct Rectangle{double xmin,ymin,xmax,ymax;};
class GridMap {
public:
 double width=20,height=20;
 std::vector<Rectangle> obstacles;
 GridMap();
 bool pointFree(double x,double y,double radius=0) const;
 bool poseFree(const State& s,double length=1.8,double bodyWidth=0.9,double margin=0.08) const;
 double clearance(double x,double y) const;
};
}
