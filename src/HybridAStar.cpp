#include "HybridAStar.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>
namespace robot {
namespace {
struct Node {State state;double g;int parent;int direction;double steer;std::vector<State> segment;};
struct Open{double f,g;int id;bool operator<(const Open& o)const{return f>o.f;}};
int headingBin(double theta,double step){double a=std::fmod(theta+2*PI,2*PI);return (int)std::floor(a/step);}
long long key(const State& s,const HybridConfig& cfg,const GridMap& map){int nx=(int)std::ceil(map.width/cfg.xyResolution),ny=(int)std::ceil(map.height/cfg.xyResolution);int bins=(int)std::ceil(2*PI/cfg.headingResolution);int ix=(int)std::floor(s.x/cfg.xyResolution),iy=(int)std::floor(s.y/cfg.xyResolution),it=headingBin(s.theta,cfg.headingResolution)%bins;return (static_cast<long long>(it)*ny+iy)*nx+ix;}
}
PlanResult planHybridAStar(const GridMap& map,const VehicleModel& vehicle,State start,State goal,const HybridConfig& cfg){
 auto begin=std::chrono::steady_clock::now();PlanResult out;
 if(!map.poseFree(start)||!map.poseFree(goal))return out;
 std::vector<Node> nodes;nodes.push_back({start,0,-1,0,0,{start}});
 std::priority_queue<Open> open;open.push({std::hypot(start.x-goal.x,start.y-goal.y),0,0});
 std::unordered_map<long long,double> best;best[key(start,cfg,map)]=0;
 int found=-1;
 const double steering[5]={-1,-0.5,0,0.5,1};
 while(!open.empty()&&out.expanded<cfg.maxExpanded){
  Open item=open.top();open.pop();const Node n=nodes[item.id];
  auto it=best.find(key(n.state,cfg,map));if(it==best.end()||item.g>it->second+1e-8)continue;
  ++out.expanded;
  if(std::hypot(n.state.x-goal.x,n.state.y-goal.y)<cfg.goalPositionTolerance&&std::abs(wrapAngle(n.state.theta-goal.theta))<cfg.goalHeadingTolerance){found=item.id;break;}
  for(int dir : {1,-1}){if(dir==-1&&!cfg.allowReverse)continue;
   for(double angle:steering){double delta=angle*cfg.maxSteer;
    auto segment=vehicle.propagate(n.state,dir*cfg.speed,delta,cfg.primitiveDuration,cfg.dt);
    bool free=true;for(const auto& s:segment){if(!map.poseFree(s)){free=false;break;}}if(!free)continue;
    auto next=segment.back();
    double step=cfg.speed*cfg.primitiveDuration;
    double cost=step*(dir<0?1.5:1.0)+step*0.05*std::abs(angle);
    if(n.direction!=0&&dir!=n.direction)cost+=1.5;
    cost+=step*0.07*std::abs(angle*cfg.maxSteer-n.steer)/cfg.maxSteer;
    double ng=n.g+cost;long long k=key(next,cfg,map);
    if(best.count(k)&&ng>=best[k]-1e-8)continue;
    best[k]=ng;int id=(int)nodes.size();nodes.push_back({next,ng,item.id,dir,delta,std::move(segment)});
    double h=std::hypot(next.x-goal.x,next.y-goal.y);
    // Heading bias for search speed: not an admissible optimality guarantee.
    h+=0.35*std::abs(wrapAngle(goal.theta-next.theta));
    open.push({ng+h,ng,id});
   }
  }
 }
 if(found!=-1){out.success=true;std::vector<int> chain;for(int i=found;i!=-1;i=nodes[i].parent)chain.push_back(i);std::reverse(chain.begin(),chain.end());
  out.path.push_back(nodes[chain.front()].state);
  for(size_t j=1;j<chain.size();++j){const auto& s=nodes[chain[j]].segment;out.path.insert(out.path.end(),s.begin()+1,s.end());}
  calculateMetrics(out,map);
  // Curvature of motion primitives, rather than finite differences of polyline.
  out.maxCurvature=0;for(size_t j=1;j<chain.size();++j)out.maxCurvature=std::max(out.maxCurvature,std::abs(std::tan(nodes[chain[j]].steer)/vehicle.wheelbase()));
 }
 out.timeMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();return out;
}
}
