#pragma once
#include <cmath>
#include <algorithm>

namespace mobility {
struct Risk {int level;double predicted_tilt_rad;bool prepare_buffer;};
// Model-specific rule baseline. Thresholds are not clinical fall classifiers.
class RiskMonitor {
 public:
  Risk step(double roll,double pitch,double wx,double wy,double hip_height,double vz,double dt=.002) {
    if(!std::isfinite(roll)||!std::isfinite(pitch)||!std::isfinite(wx)||!std::isfinite(wy)||
       !std::isfinite(hip_height)||!std::isfinite(vz)||!std::isfinite(dt)||dt<=0||dt>.02) {
      latched_=3;return {3,0,true};
    }
    const double predicted=std::hypot(roll+.25*wx,pitch+.25*wy);
    if(latched_)return {latched_,predicted,true};
    if(std::hypot(roll,pitch)>.45||hip_height<.65)latched_=2;
    if(latched_)return {latched_,predicted,true};
    warning_duration_=predicted>.22?std::min(.2,warning_duration_+dt):0;
    const bool warning=warning_duration_>=.08;
    const bool prepare=(warning&&predicted>.28)||vz<-.7;
    return {warning?1:0,predicted,prepare};
  }
 private:
  int latched_=0;
  double warning_duration_=0;
};
}
