#include "GridMap.h"
#include <algorithm>
#include <cmath>
#include <limits>
namespace robot {
GridMap::GridMap(){obstacles={{5,9,8,17},{11,3,14.5,11},{16,14,18.5,18}};}
bool GridMap::pointFree(double x,double y,double r) const {
 if(x-r<0||y-r<0||x+r>width||y+r>height)return false;
 for(const auto& b:obstacles){double px=std::clamp(x,b.xmin,b.xmax),py=std::clamp(y,b.ymin,b.ymax);if(std::hypot(x-px,y-py)<=r)return false;}
 return true;
}
// OBB vs AABB separating-axis test, including safety margin.
bool GridMap::poseFree(const State& s,double length,double bodyWidth,double margin) const {
 const double hx=length/2+margin,hy=bodyWidth/2+margin;
 double c=std::cos(s.theta),sn=std::sin(s.theta);
 double extentX=hx*std::abs(c)+hy*std::abs(sn);
 double extentY=hx*std::abs(sn)+hy*std::abs(c);
 if(s.x-extentX<0||s.x+extentX>width||s.y-extentY<0||s.y+extentY>height)return false;
 for(const auto& b:obstacles){
  const double bx=(b.xmin+b.xmax)/2,by=(b.ymin+b.ymax)/2;
  const double ex=(b.xmax-b.xmin)/2,ey=(b.ymax-b.ymin)/2;
  const double dx=bx-s.x,dy=by-s.y;
  if(std::abs(dx)>extentX+ex||std::abs(dy)>extentY+ey)continue;
  const double along=dx*c+dy*sn,side=-dx*sn+dy*c;
  if(std::abs(along)>hx+ex*std::abs(c)+ey*std::abs(sn))continue;
  if(std::abs(side)>hy+ex*std::abs(sn)+ey*std::abs(c))continue;
  return false;
 }
 return true;
}
double GridMap::clearance(double x,double y) const {
 double d=std::min({x,y,width-x,height-y});
 for(const auto& b:obstacles){double px=std::clamp(x,b.xmin,b.xmax),py=std::clamp(y,b.ymin,b.ymax);d=std::min(d,std::hypot(x-px,y-py));}
 return d;
}
}
