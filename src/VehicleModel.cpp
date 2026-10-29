#include "VehicleModel.h"
#include <algorithm>
#include <stdexcept>
namespace robot {
double wrapAngle(double a){return std::atan2(std::sin(a),std::cos(a));}
State VehicleModel::step(State s,double v,double delta,double dt) const {
 if(wheelbase_<=0||dt<=0) throw std::invalid_argument("wheelbase and dt must be positive");
 const double w=v*std::tan(delta)/wheelbase_;
 if(std::abs(w)<1e-12){s.x+=v*dt*std::cos(s.theta);s.y+=v*dt*std::sin(s.theta);}
 else {const double th=s.theta+w*dt;s.x+=v/w*(std::sin(th)-std::sin(s.theta));s.y+=v/w*(std::cos(s.theta)-std::cos(th));}
 s.theta=wrapAngle(s.theta+w*dt);
 return s;
}
std::vector<State> VehicleModel::propagate(State s,double v,double delta,double duration,double dt) const {
 std::vector<State> path{s};
 const int n=std::max(1,(int)std::ceil(duration/dt));
 for(int i=0;i<n;++i){s=step(s,v,delta,duration/n);path.push_back(s);} return path;
}
}
