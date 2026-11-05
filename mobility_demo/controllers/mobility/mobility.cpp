#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/supervisor.h>
#include "AssistPolicy.hpp"
#include "PlanGate.hpp"
#include "RiskMonitor.hpp"
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using Vec = std::array<double, 3>;
using Joints = std::array<double, 10>;
constexpr double PI = 3.14159265358979323846;
double smooth(double x) { x = std::clamp(x, 0.0, 1.0); return x*x*x*(10+x*(-15+6*x)); }
double lerp(double a, double b, double x) { return a+(b-a)*smooth(x); }
Joints pose(Vec root, const std::array<Vec,2>& feet) {
  Joints out{};
  for (int leg=0; leg<2; ++leg) {
    const double side = leg==0 ? -1.0 : 1.0;
    const double dx=feet[leg][0]-root[0];
    const double dy=feet[leg][1]-(root[1]+side*.115);
    const double down=root[2]-feet[leg][2];
    const double vertical=std::hypot(down,dy);
    const double distance=std::hypot(dx,vertical);
    const double knee=std::acos(std::clamp((distance*distance-2*.42*.42)/(2*.42*.42),-1.0,1.0));
    const double hip=-std::atan2(dx,vertical)-knee/2;
    const double roll=std::atan2(dy,down);
    out[leg*5+0]=roll;
    out[leg*5+1]=hip;
    out[leg*5+2]=knee;
    out[leg*5+3]=-hip-knee;
    out[leg*5+4]=-roll;
  }
  return out;
}
double forceMoment(const double* joint, const double* foot, Vec f, Vec axis) {
  const Vec r{foot[0]-joint[0],foot[1]-joint[1],foot[2]-joint[2]};
  const Vec cross{r[1]*f[2]-r[2]*f[1],r[2]*f[0]-r[0]*f[2],r[0]*f[1]-r[1]*f[0]};
  return -(cross[0]*axis[0]+cross[1]*axis[1]+cross[2]*axis[2]);
}
int main(int argc, char** argv) {
  wb_robot_init();
  const std::string scenario=argc>1?argv[1]:"stand";
  const bool assisted=argc>2 && std::string(argv[2])=="assist";
  const bool capture=argc>4&&std::string(argv[4])=="capture";
  const std::string output=argc>3?argv[3]:"../../runs/smoke.csv";
  const int time_step=(int)wb_robot_get_basic_time_step();
  const double dt=time_step/1000.;
  const double phase_duration=scenario=="stairs"?6.0:4.0;
  const double transfer_duration=scenario=="stairs"?2.6:2.0;
  const double swing_duration=scenario=="stairs"?3.2:1.9;
  const double duration=scenario=="stand"?5:(scenario=="stop"?19:2+6*phase_duration+9);
  double mass=0;
  try {mass=std::stod(wb_robot_get_custom_data());}catch(...) {wb_supervisor_simulation_quit(2);return 2;}
  if(!std::isfinite(mass)||mass<=0||mass>200){wb_supervisor_simulation_quit(2);return 2;}
  const char* names[10]={"r_roll","r_hip","r_knee","r_ankle","r_foot","l_roll","l_hip","l_knee","l_ankle","l_foot"};
  std::array<WbDeviceTag,10> motors{}, sensors{};
  for(int j=0;j<10;++j) {
    motors[j]=wb_robot_get_device(names[j]);
    sensors[j]=wb_motor_get_position_sensor(motors[j]);
    wb_position_sensor_enable(sensors[j],time_step);
    wb_motor_set_torque(motors[j],0);
  }
  const WbNodeRef body=wb_supervisor_node_get_self();
  const WbNodeRef foot_nodes[2]={wb_supervisor_node_get_from_def("R_FOOT"),wb_supervisor_node_get_from_def("L_FOOT")};
  const WbNodeRef hip_nodes[2]={wb_supervisor_node_get_from_def("R_HIP"),wb_supervisor_node_get_from_def("L_HIP")};
  const WbNodeRef knee_nodes[2]={wb_supervisor_node_get_from_def("R_KNEE"),wb_supervisor_node_get_from_def("L_KNEE")};
  std::ofstream csv(std::filesystem::u8path(output));
  if(!csv) { std::cerr<<"Cannot create telemetry "<<output<<"\n"; wb_supervisor_simulation_quit(2); return 2; }
  auto event_path=std::filesystem::u8path(output);event_path.replace_extension(".events.jsonl");
  std::ofstream events(event_path);
  auto event=[&](double t,const char* type,const char* detail){events<<"{\"time_s\":"<<t<<",\"type\":\""<<type<<"\",\"detail\":\""<<detail<<"\"}\n";events.flush();};
  const bool stairs=scenario=="stairs";
  const double assist_gain=stairs?.65:.35,assist_limit=stairs?80:35,assist_slew=stairs?350:250;
  mobility::PlanGate gate(assist_limit);mobility::RiskMonitor monitor;
  if(gate.accept({1,0,.5,0,6,assist_limit},0,0)!=mobility::Decision::accepted) {wb_supervisor_simulation_quit(2);return 2;}
  event(0,"virtual_cloud_plan","accepted_finite_six_step_task");
  csv<<std::setprecision(10)<<"time_s,root_x_m,root_y_m,root_z_m,roll_rad,pitch_rad,vx_m_s,vy_m_s,phase,r_contact,l_contact,yaw_rad,r_foot_x_m,r_foot_y_m,r_foot_z_m,l_foot_x_m,l_foot_y_m,l_foot_z_m,risk_level,predicted_tilt_rad,buffer_prepare_signal,cloud_online,plan_sequence,plan_rejections,test_force_x_n";
  for(int j=0;j<10;++j) csv<<",q"<<j<<"_rad,ref"<<j<<"_rad,human"<<j<<"_nm,exo"<<j<<"_nm,velocity"<<j<<"_rad_s";
  csv<<"\n";
  std::vector<mobility::AssistPolicy> policies;
  for(int j=0;j<10;++j)policies.emplace_back(assist_gain,assist_limit,assist_slew);
  Joints previous{}, previous_ref{};
  std::vector<mobility::AssistPolicy> guards;for(int j=0;j<10;++j)guards.emplace_back(1,25,150);
  Vec previous_root{0,0,.92};
  Vec previous_com{},com_speed{};
  std::array<Vec,2> feet={Vec{0,-.115,.08},Vec{0,.115,.08}};
  Vec root_target{0,0,.88}, start_root=root_target;
  std::array<Vec,2> start_feet=feet;
  int last_phase=-1;
  bool fallen=false,sensor_fault=false;
  bool stop_requested=false,prepared=false;
  int step_limit=6,previous_risk=0,rejections=0;
  std::array<bool,4> injected{};
  bool completed=false;
  double last_time=0;
  double next_capture=3;
  for(int sample=0;wb_robot_step(time_step)!=-1;++sample) {
    const double t=wb_robot_get_time();
    last_time=t;
    if(capture&&t>=next_capture) {auto path=std::filesystem::u8path(output);path.replace_extension();const auto filename=path.string()+"_frame_"+std::to_string((int)std::round(t*1000))+".jpg";wb_supervisor_export_image(filename.c_str(),90);next_capture+=2;}
    const double* p=wb_supervisor_node_get_position(body);
    const double* rotation=wb_supervisor_node_get_orientation(body);
    const double* velocity=wb_supervisor_node_get_velocity(body);
    bool feedback_valid=true;
    for(int i=0;i<3;++i)feedback_valid&=std::isfinite(p[i]);
    for(int i=0;i<9;++i)feedback_valid&=std::isfinite(rotation[i]);
    for(int i=0;i<6;++i)feedback_valid&=std::isfinite(velocity[i]);
    for(auto sensor:sensors)feedback_valid&=std::isfinite(wb_position_sensor_get_value(sensor));
    if(!feedback_valid) {sensor_fault=true;for(auto motor:motors)wb_motor_set_torque(motor,0);event(t,"invalid_observation","zero_motor_commands_and_end_test");break;}
    const double roll=std::atan2(rotation[7],rotation[8]);
    const double pitch=std::atan2(-rotation[6],std::hypot(rotation[7],rotation[8]));
    const double gait_t=std::max(0.0,t-2);
    if(scenario=="stop"&&t>=9&&!stop_requested) {step_limit=std::min(6,1+(int)(gait_t/phase_duration));stop_requested=true;event(t,"local_stop","finish_current_step_then_transfer_to_hold");}
    const int phase=scenario=="stand" ? -1 : std::min(step_limit,(int)(gait_t/phase_duration));
    double toe_pitch=0;
    const double pulse_end=scenario=="push-large"?13.15:12.95;
    const double push=(scenario=="push-small"||scenario=="push-large")&&t>=12.8&&t<pulse_end?(scenario=="push-large"?400:60):0;
    if(push) {const double disturbance[3]={push,0,0};wb_supervisor_node_add_force(body,disturbance,false);}
    if(push&&!injected[3]) {event(t,"external_test_disturbance",scenario=="push-large"?"400N_for_0.35s":"60N_for_0.15s");injected[3]=true;}
    const double times[3]={5,12,14};
    for(int k=0;k<3;++k)if(t>=times[k]&&!injected[k]) {
      mobility::Plan candidate{(std::uint64_t)k+2,t,.4,phase,6,35};
      if(k==0)candidate.issued_s=t-2;
      if(k==1)candidate.phase=phase-1;
      if(k==2)candidate.requested_torque_nm=500;
      const auto decision=gate.accept(candidate,t,phase);if(decision!=mobility::Decision::accepted)++rejections;
      event(t,"virtual_cloud_candidate",mobility::reason(decision));injected[k]=true;
    }
    if(phase>=0 && phase<step_limit) {
      if(phase!=last_phase) {
        // Reconnect each stair phase to observed horizontal state; never move the body directly.
        if(stairs&&t>2) {
          for(int leg=0;leg<2;++leg) {const double* observed=wb_supervisor_node_get_position(foot_nodes[leg]);feet[leg][0]=observed[0];feet[leg][1]=observed[1];}
          root_target[0]=p[0];root_target[1]=p[1];
        }
        start_feet=feet;start_root=root_target;last_phase=phase;
      }
      const int moving=phase%2, support=1-moving;
      const double u=(gait_t-phase*phase_duration);
      const double target_x=scenario=="stairs"?.37+.40*(phase/2):.20*(phase+1);
      const double ground=scenario=="stairs"?.1*(phase/2+1):0;
      root_target[0]=lerp(start_root[0],start_feet[support][0]+.025,u/(scenario=="stairs"?4.0:transfer_duration));
      const double support_y=stairs?start_feet[support][1]+(support==0?-.00575:.00575):start_feet[support][1]*1.05;
      root_target[1]=lerp(start_root[1],support_y,u/(scenario=="stairs"?4.0:transfer_duration));
      root_target[2]=lerp(start_root[2],.88+start_feet[support][2]-.08,scenario=="stairs"?(u-transfer_duration)/2.0:u/transfer_duration);
      const double swing=std::clamp((u-transfer_duration)/swing_duration,0.0,1.0);
      feet[moving][0]=lerp(start_feet[moving][0],target_x,swing);
      if(stairs)feet[moving][1]=lerp(start_feet[moving][1],moving==0?-.115:.115,swing);
      feet[moving][2]=lerp(start_feet[moving][2],.08+ground,swing)+(scenario=="stairs"?.13:.07)*std::sin(PI*swing)*std::sin(PI*swing);
      if(scenario=="stairs") {
        Vec launch=start_feet[moving];
        const double peak=std::max(launch[2],.08+ground)+.10;
        feet[moving][0]=lerp(launch[0],target_x,(swing-.25)/.50);
        feet[moving][2]=swing<.25 ? lerp(launch[2],peak,swing/.25) : (swing>.75 ? lerp(peak,.08+ground,(swing-.75)/.25) : peak);
      }
    } else if(phase==step_limit) {
      if(last_phase!=step_limit) {start_root=root_target;last_phase=step_limit;event(t,"task_transition","transfer_to_double_support_hold");}
      const double u=(gait_t-step_limit*phase_duration)/3.0;
      root_target[0]=lerp(start_root[0],(feet[0][0]+feet[1][0])*.5+.02,u);
      root_target[1]=lerp(start_root[1],0,u);
    }
    const double initial=smooth(t/1.5);
    if(t<2 || scenario=="stand")root_target[2]=lerp(.92,.88,initial);
    Joints refs=pose(root_target,feet);
    if(phase>=0&&phase<step_limit)refs[(phase%2)*5+3]+=toe_pitch;
    if(phase>=0 && phase<step_limit && gait_t-phase*phase_duration>transfer_duration) {
      const int moving=phase%2;
      Vec local{};
      for(int i=0;i<3;++i)for(int k=0;k<3;++k)local[i]+=rotation[k*3+i]*(feet[moving][k]-p[k]);
      const double side=moving==0?-1.0:1.0;
      local[1]-=side*.115;
      const double down=-local[2],vertical=std::hypot(down,local[1]);
      const double d2=local[0]*local[0]+vertical*vertical;
      const double knee=std::acos(std::clamp((d2-2*.42*.42)/(2*.42*.42),-1.0,1.0));
      const double hip=-std::atan2(local[0],vertical)-knee/2;
      const double legroll=std::atan2(local[1],down);
      const double progress=std::clamp((gait_t-phase*phase_duration-transfer_duration)/swing_duration,0.0,1.0);
      const double world_feedback=scenario=="stairs"?1-smooth((progress-.75)/.20):1.0;
      const double swing_refs[5]={legroll,hip,knee,-pitch-hip-knee+toe_pitch,-roll-legroll};
      for(int k=0;k<5;++k)refs[moving*5+k]=world_feedback*swing_refs[k]+(1-world_feedback)*refs[moving*5+k];
    }
    Vec root_speed{};
    for(int i=0;i<3;++i)root_speed[i]=sample?(root_target[i]-previous_root[i])/dt:0;
    const auto risk=monitor.step(roll,pitch,velocity[3],velocity[4],p[2]-std::max(0.0,root_target[2]-.88),velocity[2],dt);
    if(risk.level!=previous_risk) {event(t,"local_risk_level",risk.level==0?"normal":(risk.level==1?"predictive_warning":"outside_recovery_envelope"));previous_risk=risk.level;}
    if(risk.prepare_buffer&&!prepared) {prepared=true;event(t,"buffer_prepare_signal","signal_only_no_whole_body_pad_deployment");}
    csv<<t<<","<<p[0]<<","<<p[1]<<","<<p[2]<<","<<roll<<","<<pitch<<","<<velocity[0]<<","<<velocity[1]<<","<<phase;
    int contacts[2]={0,0};
    for(int leg=0;leg<2;++leg) {wb_supervisor_node_get_contact_points(foot_nodes[leg],false,&contacts[leg]);csv<<","<<contacts[leg];}
    csv<<","<<std::atan2(rotation[3],rotation[0]);
    for(auto node:foot_nodes) {const double* fp=wb_supervisor_node_get_position(node);for(int i=0;i<3;++i)csv<<","<<fp[i];}
    csv<<","<<risk.level<<","<<risk.predicted_tilt_rad<<","<<prepared<<","<<(t<15)<<","<<gate.active().sequence<<","<<rejections<<","<<push;
    const double* com=wb_supervisor_node_get_center_of_mass(body);
    for(int i=0;i<3;++i) {const double raw=sample?(com[i]-previous_com[i])/dt:0;com_speed[i]+=(raw-com_speed[i])*dt/(.02+dt);previous_com[i]=com[i];}
    const double* rf=wb_supervisor_node_get_position(foot_nodes[0]);
    const double* lf=wb_supervisor_node_get_position(foot_nodes[1]);
    const double span_x=lf[0]-rf[0],span_y=lf[1]-rf[1];
    const double load_projection=scenario=="stairs"?((com[0]-rf[0])*span_x+(com[1]-rf[1])*span_y)/std::max(.01,span_x*span_x+span_y*span_y):(com[1]-rf[1])/std::max(.05,lf[1]-rf[1]);
    const double left_load=contacts[0]&&contacts[1]?std::clamp(load_projection,0.0,1.0):(contacts[1]?1.0:0.0);
    const Vec force{std::clamp(1200*(root_target[0]+.012-com[0])+500*(root_speed[0]-com_speed[0]),-200.0,200.0),
                    std::clamp(1200*(root_target[1]*.78-com[1])+500*(root_speed[1]*.78-com_speed[1]),-200.0,200.0),
                    std::clamp(mass*9.81+2000*(root_target[2]-p[2])+300*(root_speed[2]-velocity[2]),0.0,1200.0)};
    for(int j=0;j<10;++j) {
      const int kind=j%5;
      const double q=wb_position_sensor_get_value(sensors[j]);
      const double v=sample? (q-previous[j])/dt:0;
      const double ref_v=sample?(refs[j]-previous_ref[j])/dt:0;
      const double kp_flat[5]={900,900,1100,400,300},kp_stairs[5]={5000,5000,6000,2400,1000};
      const double kd_flat[5]={35,40,35,6,2},kd_stairs[5]={80,90,90,8,1.2};
      const double* kp=scenario=="stairs"?kp_stairs:kp_flat;
      const double* kd=scenario=="stairs"?kd_stairs:kd_flat;
      const double limit[5]={120,120,140,110,90};
      const int leg=j/5;
      const double load=leg?left_load:1-left_load;
      const double leg_roll=wb_position_sensor_get_value(sensors[leg*5]);
      const Vec axis_roll{rotation[0],rotation[3],rotation[6]};
      Vec axis_pitch{};
      for(int i=0;i<3;++i)axis_pitch[i]=rotation[3*i+1]*std::cos(leg_roll)+rotation[3*i+2]*std::sin(leg_roll);
      Vec f{};for(int i=0;i<3;++i)f[i]=load*force[i];
      const double* fp=leg?lf:rf;
      double feedforward=contacts[leg]&&kind<3?forceMoment(wb_supervisor_node_get_position(kind==2?knee_nodes[leg]:hip_nodes[leg]),fp,f,kind==0?axis_roll:axis_pitch):0;
      if(contacts[leg]&&kind==3)feedforward=(com[0]-fp[0])*f[2]-(com[2]-fp[2])*f[0];
      if(contacts[leg]&&kind==4)feedforward=(com[2]-fp[2])*f[1]-(com[1]-fp[1])*f[2];
      const double balance=contacts[leg]>0 ? load*(kind==3 ? 400*pitch+100*velocity[4] : (kind==4 ? 400*roll+100*velocity[3] : 0)) : 0;
      const double human=std::clamp(kp[kind]*(refs[j]-q)+kd[kind]*(ref_v-v)+balance+feedforward,-limit[kind],limit[kind]);
      const double protective=load*(kind==3?70*pitch+15*velocity[4]:(kind==4?70*roll+15*velocity[3]:0));
      const double exo=policies[j].step(human,dt,assisted && (kind==1||kind==2||(stairs&&kind==0)))+
                       guards[j].step(protective,dt,assisted&&risk.level>0&&(kind==3||kind==4));
      wb_motor_set_torque(motors[j],human+exo);
      csv<<","<<q<<","<<refs[j]<<","<<human<<","<<exo<<","<<v;
      previous[j]=q;previous_ref[j]=refs[j];
    }
    previous_root=root_target;
    csv<<"\n";
    if(t>1 && (p[2]<.55 || std::abs(roll)>.8 || std::abs(pitch)>.8)) {fallen=true;break;}
    if(t>=duration) {completed=true;break;}
  }
  csv.close();
  event(last_time,completed&&!fallen?"simulation_complete":"simulation_not_accepted",fallen?"fall_threshold_reached":"duration_reached");
  events.close();
  auto done_path=std::filesystem::u8path(output);done_path.replace_extension(".completed.json");
  std::ofstream done(done_path);done<<"{\"completed\":"<<(completed&&!fallen&&!sensor_fault?"true":"false")<<",\"fallen\":"<<(fallen?"true":"false")<<",\"sensor_fault\":"<<(sensor_fault?"true":"false")<<",\"sim_time_s\":"<<last_time<<",\"mass_kg\":"<<mass<<"}";done.close();
  std::cout<<"MOBILITY_RESULT scenario="<<scenario<<" assisted="<<assisted<<" fallen="<<fallen<<std::endl;
  wb_supervisor_simulation_quit(completed&&!fallen&&!sensor_fault?0:1);
  wb_robot_cleanup();
  return completed&&!fallen&&!sensor_fault?0:1;
}
