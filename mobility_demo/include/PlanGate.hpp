#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace mobility {
struct Plan {
  std::uint64_t sequence;
  double issued_s,ttl_s; // TTL bounds delivery age, not an executing task's duration.
  int phase,steps;
  double requested_torque_nm;
};
enum class Decision {accepted,invalid,expired,sequence,phase,capacity};
inline const char* reason(Decision d) {
  switch(d) {
    case Decision::accepted:return "accepted";
    case Decision::invalid:return "invalid_time_or_fields";
    case Decision::expired:return "expired";
    case Decision::sequence:return "old_sequence";
    case Decision::phase:return "contact_phase_changed";
    case Decision::capacity:return "outside_local_capability";
  }
  return "invalid";
}
// A deliberately narrow virtual-cloud contract; not a physical feasibility proof.
class PlanGate {
 public:
  explicit PlanGate(double capability_nm=35):capability_nm_(capability_nm) {
    if(!std::isfinite(capability_nm)||capability_nm<=0||capability_nm>80)
      throw std::invalid_argument("Unsupported local capability");
  }
  Decision accept(const Plan& p,double now,int phase) {
    if(!std::isfinite(now)||!std::isfinite(p.issued_s)||!std::isfinite(p.ttl_s)||
       !std::isfinite(p.requested_torque_nm)||now<0||p.issued_s<0||p.issued_s>now||
       p.ttl_s<=0||p.ttl_s>1||!p.sequence)return Decision::invalid;
    if(p.sequence<=active_.sequence)return Decision::sequence;
    if(now-p.issued_s>p.ttl_s)return Decision::expired;
    if(p.phase!=phase)return Decision::phase;
    if(p.steps<0||p.steps>6||p.requested_torque_nm<0||p.requested_torque_nm>capability_nm_)return Decision::capacity;
    active_=p;
    return Decision::accepted;
  }
  const Plan& active() const {return active_;}
 private:
  const double capability_nm_;
  Plan active_{};
};
}
