#pragma once
#include <cmath>
#include <vector>
namespace robot {
constexpr double PI=3.14159265358979323846;
struct State {double x=0,y=0,theta=0;};
double wrapAngle(double angle);
class VehicleModel {
public:
 explicit VehicleModel(double wheelbase=1.2): wheelbase_(wheelbase) {}
 State step(State state,double velocity,double steering,double dt) const;
 std::vector<State> propagate(State initial,double velocity,double steering,double duration,double dt) const;
 double wheelbase() const {return wheelbase_;}
private: double wheelbase_;
};
}
