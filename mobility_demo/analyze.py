"""Derive acceptance evidence from simulator telemetry, never from exit code alone."""
from pathlib import Path
import csv
import json
import math
import hashlib
import re
import numpy as np

ROOT=Path(__file__).resolve().parent
FRAME_KEYS=['time_s','root_x_m','root_y_m','root_z_m','roll_rad','pitch_rad','yaw_rad']+[f'q{i}_rad' for i in range(10)]+['r_contact','l_contact','risk_level','buffer_prepare_signal','cloud_online','predicted_tilt_rad']

def evidence_path(relative):
    for base in (ROOT,ROOT/'evidence'):
        path=base/relative
        if path.is_file():return path
    raise FileNotFoundError(relative)

def alternating(touchdowns,count=6):
    events=sorted(touchdowns,key=lambda e:e['time_s'])
    return len(events)>=count and all(events[i]['side']!=events[i-1]['side'] for i in range(1,len(events)))

def command_positive_work(time,angle,command):
    t,q,tau=map(lambda x:np.asarray(x,dtype=float),(time,angle,command))
    if len(t)<2 or t.shape!=q.shape or t.shape!=tau.shape or not all(np.isfinite(x).all() for x in (t,q,tau)) or np.any(np.diff(t)<=0):
        raise ValueError('Nonfinite, mismatched or nonmonotonic work evidence')
    # Command at t[i] is held while Webots advances to t[i+1].
    return float(np.maximum(0,tau[:-1]*np.diff(q)).sum())

def analyze(name):
    path=evidence_path(f'runs/{name}.csv')
    with path.open(encoding='utf-8') as f:rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(f)]
    if len(rows)<2:raise ValueError('Insufficient simulator evidence')
    a={k:np.array([r[k] for r in rows]) for k in rows[0]}
    t=a['time_s'];dt=np.diff(t,prepend=t[0]);tail=t>=t[-1]-2
    if not all(np.isfinite(x).all() for x in a.values()) or np.any(dt[1:]<=0):raise ValueError('Invalid simulator evidence')
    work=sum(command_positive_work(t,a[f'q{j}_rad'],a[f'human{j}_nm']) for j in range(10))
    exo_work=sum(command_positive_work(t,a[f'q{j}_rad'],a[f'exo{j}_nm']) for j in range(10))
    steps=0;touchdowns=[]
    for side in ('r','l'):
        airborne=0.;stable=0.;candidate=False;last_x=a[f'{side}_foot_x_m'][0]
        for i,r in enumerate(rows):
            contact=r[f'{side}_contact']>=2
            if not contact:
                airborne+=dt[i];stable=0
                if airborne>.1:candidate=True
            else:
                stable+=dt[i]
                if stable>.08:
                    if candidate and r[f'{side}_foot_x_m']-last_x>.08:
                        steps+=1;last_x=r[f'{side}_foot_x_m'];candidate=False
                        touchdowns.append({'time_s':r['time_s'],'side':side,'x_m':r[f'{side}_foot_x_m'],'ankle_height_m':r[f'{side}_foot_z_m']})
                    airborne=0
    marker=path.with_suffix('.completed.json')
    completion=json.loads(marker.read_text()) if marker.exists() else {}
    raw_report=path.with_suffix('.json');run_report=json.loads(raw_report.read_text(encoding='utf-8')) if raw_report.exists() else {}
    tilt=np.degrees(np.arccos(np.clip(np.cos(a['roll_rad'])*np.cos(a['pitch_rad']),-1,1)))
    stationary_range=float(np.hypot(np.ptp(a['root_x_m'][tail]),np.ptp(a['root_y_m'][tail])))
    window=max(1,round(.1/float(np.median(dt[1:]))))
    speed=np.hypot(a['root_x_m'][window:]-a['root_x_m'][:-window],a['root_y_m'][window:]-a['root_y_m'][:-window])/(t[window:]-t[:-window])
    speed_tail=speed[t[window:]>=t[-1]-2]
    summary={'run':name,'scenario':run_report.get('scenario'),'mode':run_report.get('mode'),'duration_s':float(t[-1]),'completed':bool(completion.get('completed',False)),
        'engine_exit_code':run_report.get('exit_code'),'forward_distance_m':float(a['root_x_m'][-1]-a['root_x_m'][0]),'stable_touchdown_count':steps,'touchdowns':sorted(touchdowns,key=lambda e:e['time_s']),'alternating_touchdowns':alternating(touchdowns),'mass_kg':completion.get('mass_kg'),
        'min_hip_height_m':float(a['root_z_m'].min()),'final_hip_height_m':float(a['root_z_m'][-1]),'max_trunk_tilt_deg':float(tilt.max()),'final_ankle_heights_m':[float(a['r_foot_z_m'][-1]),float(a['l_foot_z_m'][-1])],
        'human_equivalent_positive_work_j':work,'device_positive_mechanical_work_j':exo_work,'last_2s_pelvis_range_m':stationary_range,
        'last_2s_filtered_speed_p95_m_s':float(np.percentile(speed_tail,95)),'last_2s_both_feet_contact_fraction':float(np.mean((a['r_contact'][tail]>=2)&(a['l_contact'][tail]>=2))),
        'finite_telemetry':bool(all(np.isfinite(x).all() for x in a.values())),
        'simulation_source_sha256':run_report.get('source_sha256',{}),'evidence':'actual_Webots_telemetry_synthetic_human_not_clinical'}
    if 'risk_level' in a:
        warning=np.flatnonzero(a['risk_level']>=1);prepare=np.flatnonzero(a['buffer_prepare_signal']>0)
        summary.update(first_warning_s=float(t[warning[0]]) if len(warning) else None,first_buffer_prepare_signal_s=float(t[prepare[0]]) if len(prepare) else None,
            rejected_cloud_candidates=int(a['plan_rejections'][-1]),local_samples_after_virtual_cloud_offline=int(np.sum(a['cloud_online']==0)))
    stride=max(1,round(.04/float(np.median(dt[1:]))))
    frames=[[round(rows[i].get(k,0),6) for k in FRAME_KEYS] for i in range(0,len(rows),stride)]
    if frames[-1][0]!=rows[-1]['time_s']:frames.append([round(rows[-1].get(k,0),6) for k in FRAME_KEYS])
    return {'type':'walking','id':name,'summary':summary,'columns':FRAME_KEYS,'frames':frames}

def pair_metadata_valid(a,b,model_same,root_rms_m,joint_rms_rad):
    required={'controllers/mobility/mobility.cpp','include/AssistPolicy.hpp','include/PlanGate.hpp','include/RiskMonitor.hpp','prepare.py'}
    x,y=a.get('simulation_source_sha256',{}),b.get('simulation_source_sha256',{})
    sources=required.issubset(x) and required.issubset(y) and all(re.fullmatch('[0-9a-f]{64}',x[k]) and x[k]==y[k] for k in required)
    masses=a.get('mass_kg'),b.get('mass_kg')
    return bool(sources and model_same and a.get('scenario')==b.get('scenario')=='flat' and a.get('mode')=='off' and b.get('mode')=='assist' and
        all(isinstance(v,(int,float)) and math.isfinite(v) for v in masses) and abs(masses[0]-masses[1])<1e-6 and
        abs(a.get('duration_s',0)-b.get('duration_s',-1))<1e-6 and a.get('completed') and b.get('completed') and
        a.get('alternating_touchdowns') and b.get('alternating_touchdowns') and math.isfinite(root_rms_m) and math.isfinite(joint_rms_rad) and root_rms_m<=.05 and joint_rms_rad<=.10)

def compare_runs(off_name,on_name):
    off,on=analyze(off_name),analyze(on_name)
    def read(name):
        with evidence_path(f'runs/{name}.csv').open() as f:return np.array([[float(r[k]) for k in ['time_s','root_x_m','root_y_m','root_z_m']+[f'q{j}_rad' for j in range(10)]] for r in csv.DictReader(f)])
    a,b=read(off_name),read(on_name);sample_t=a[:,0]
    aligned=np.column_stack([np.interp(sample_t,b[:,0],b[:,k]) for k in range(1,b.shape[1])])
    errors=a[:,1:]-aligned;root_rms=float(np.sqrt(np.mean(np.sum(errors[:,:3]**2,axis=1))));joint_rms=float(np.sqrt(np.mean(errors[:,3:]**2)))
    def model(name):return re.sub(r'controllerArgs\s*\[.*?\]','controllerArgs []',evidence_path(f'worlds/{name}.wbt').read_text(encoding='utf-8'),flags=re.S)
    model_same=model(off_name)==model(on_name)
    valid=pair_metadata_valid(off['summary'],on['summary'],model_same,root_rms,joint_rms)
    return {'valid':valid,'same_physics_world':model_same,'root_position_rms_m':root_rms,'joint_angle_rms_rad':joint_rms}

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('run');a=p.parse_args();result=analyze(a.run)
    (ROOT/'results').mkdir(exist_ok=True)
    (ROOT/'results'/f'{a.run}.summary.json').write_text(json.dumps(result['summary'],indent=2),encoding='utf-8')
    print(json.dumps(result['summary']))
