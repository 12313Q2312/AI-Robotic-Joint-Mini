#include "AssistPolicy.hpp"
#include "PlanGate.hpp"
#include "RiskMonitor.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

int main() {
  mobility::AssistPolicy assist(.35,35,100);
  double previous=0;
  for(int i=0;i<1000;++i) {double y=assist.step(500,.002,true);assert(y<=35+1e-12);assert(std::abs(y-previous)<=.2+1e-12);previous=y;}
  for(int i=0;i<1000;++i)previous=assist.step(500,.002,false);
  assert(std::abs(previous)<1e-12);
  assert(assist.step(std::numeric_limits<double>::quiet_NaN(),.002,true)==0);
  std::cout<<"PASS assistance magnitude, slew, disable and invalid feedback\n";
  mobility::PlanGate gate;
  mobility::Plan p{1,1,.4,0,6,35};
  assert(gate.accept(p,1.1,0)==mobility::Decision::accepted);
  assert(gate.accept(p,1.2,0)==mobility::Decision::sequence);
  p.sequence=2;
  assert(gate.accept(p,1.5,0)==mobility::Decision::expired);
  p.issued_s=2;
  assert(gate.accept(p,1.5,0)==mobility::Decision::invalid);
  p.issued_s=1.5;p.phase=1;
  assert(gate.accept(p,1.6,0)==mobility::Decision::phase);
  p.phase=0;p.requested_torque_nm=500;
  assert(gate.accept(p,1.6,0)==mobility::Decision::capacity);
  p.requested_torque_nm=35;
  assert(gate.accept(p,1.6,0)==mobility::Decision::accepted);
  assert(gate.active().sequence==2);
  mobility::PlanGate stairs(80);
  p.sequence=1;p.requested_torque_nm=80;
  assert(stairs.accept(p,1.6,0)==mobility::Decision::accepted);
  p.sequence=2;p.requested_torque_nm=81;
  assert(stairs.accept(p,1.6,0)==mobility::Decision::capacity);
  bool invalid_capacity=false;
  try {mobility::PlanGate invalid(NAN);}catch(const std::invalid_argument&) {invalid_capacity=true;}
  assert(invalid_capacity);
  std::cout<<"PASS cloud age, sequence, phase and capability admission\n";
  mobility::RiskMonitor risk;
  auto normal=risk.step(0,0,0,0,.88,0);
  assert(normal.level==0&&!normal.prepare_buffer);
  auto predicted=risk.step(0,.10,0,.8,.88,0);
  assert(predicted.level==0&&!predicted.prepare_buffer);
  for(int i=0;i<50;++i)predicted=risk.step(0,.10,0,.8,.88,0);
  assert(predicted.level>=1&&predicted.prepare_buffer);
  auto fallen=risk.step(0,.55,0,0,.60,-1);
  assert(fallen.level==2&&fallen.prepare_buffer);
  auto latched=risk.step(0,0,0,0,.88,0);
  assert(latched.level==2);
  mobility::RiskMonitor fault;
  assert(fault.step(0,NAN,0,0,.88,0).level==3);
  assert(fault.step(0,.55,0,0,.60,-1).level==3);
  std::cout<<"PASS predictive warning, protection preparation, latch and sensor fault\n";
}
