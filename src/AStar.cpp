#include "AStar.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
namespace robot {
void calculateMetrics(PlanResult& r,const GridMap& map){
 r.length=0;r.maxCurvature=0;r.minCenterClearance=std::numeric_limits<double>::infinity();
 for(size_t i=0;i<r.path.size();++i){auto& s=r.path[i];r.minCenterClearance=std::min(r.minCenterClearance,map.clearance(s.x,s.y));if(i){auto& p=r.path[i-1];r.length+=std::hypot(s.x-p.x,s.y-p.y);}}
 for(size_t i=1;i+1<r.path.size();++i){auto& a=r.path[i-1];auto& b=r.path[i];auto& c=r.path[i+1];double ux=b.x-a.x,uy=b.y-a.y,vx=c.x-b.x,vy=c.y-b.y;double la=std::hypot(ux,uy),lb=std::hypot(vx,vy),lc=std::hypot(c.x-a.x,c.y-a.y);if(la*lb*lc>1e-10)r.maxCurvature=std::max(r.maxCurvature,std::abs(2*(ux*vy-uy*vx)/(la*lb*lc)));}
 if(r.path.empty())r.minCenterClearance=0;
}
PlanResult planAStar(const GridMap& map,State start,State goal,double res){
 auto begin=std::chrono::steady_clock::now();PlanResult out;
 int nx=(int)std::ceil(map.width/res),ny=(int)std::ceil(map.height/res);
 auto index=[nx](int x,int y){return y*nx+x;};
 auto cell=[res](int x,int y){return State{(x+0.5)*res,(y+0.5)*res,0};};
 auto valid=[&](int x,int y){return x>=0&&y>=0&&x<nx&&y<ny&&map.pointFree(cell(x,y).x,cell(x,y).y,0.48);};
 int sx=(int)(start.x/res),sy=(int)(start.y/res),gx=(int)(goal.x/res),gy=(int)(goal.y/res);
 if(!valid(sx,sy)||!valid(gx,gy))return out;
 struct Q{double f,g;int id;bool operator<(const Q& o)const{return f>o.f;}};
 std::priority_queue<Q> pq;std::vector<double> g(nx*ny,std::numeric_limits<double>::infinity());std::vector<int> parent(nx*ny,-1);
 int s=index(sx,sy),target=index(gx,gy);g[s]=0;pq.push({std::hypot(goal.x-start.x,goal.y-start.y),0,s});
 const int dirs[8][2]={{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
 while(!pq.empty()){auto q=pq.top();pq.pop();if(q.g>g[q.id]+1e-9)continue;++out.expanded;if(q.id==target){out.success=true;break;}
 int x=q.id%nx,y=q.id/nx;
 for(auto& d:dirs){int xx=x+d[0],yy=y+d[1];if(!valid(xx,yy))continue;
 // Check diagonal corner cutting and segment samples.
 if(d[0]&&d[1]&&(!valid(x+d[0],y)||!valid(x,y+d[1])))continue;
 int ni=index(xx,yy);double step=res*std::hypot(d[0],d[1]);double ng=q.g+step;
 if(ng<g[ni]-1e-9){g[ni]=ng;parent[ni]=q.id;auto p=cell(xx,yy);pq.push({ng+std::hypot(goal.x-p.x,goal.y-p.y),ng,ni});}
 }
 }
 if(out.success){for(int i=target;i!=-1;i=parent[i])out.path.push_back(cell(i%nx,i/nx));std::reverse(out.path.begin(),out.path.end());calculateMetrics(out,map);}
 out.timeMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();return out;
}
}
