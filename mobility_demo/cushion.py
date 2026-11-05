"""Independent 1-DoF component model with a finite stroke and energy accounting.

The effective mass and material constants are hypotheses, not human injury data.
Compression-only damping is intentional; the spring can return stored energy.
"""
from __future__ import annotations
from dataclasses import asdict,dataclass,replace
from pathlib import Path
import csv
import json
import math
import hashlib
import re
from datetime import datetime,timezone

ROOT=Path(__file__).resolve().parent

@dataclass(frozen=True)
class Parameters:
    mass_kg:float=20.0
    drop_height_m:float=.30
    gravity_m_s2:float=9.81
    pad_stiffness_n_m:float=12000.0
    pad_damping_ns_m:float=350.0
    stroke_m:float=.08
    stop_stiffness_n_m:float=1_000_000.0
    stop_damping_ns_m:float=1800.0

def contact(y:float,v:float,p:Parameters):
    x=max(0.,-y)
    compression=min(x,p.stroke_m)
    bottom=max(0.,x-p.stroke_m)
    if x<=0:return 0.,0.,0.,False
    spring=p.pad_stiffness_n_m*compression+p.stop_stiffness_n_m*bottom
    damping=p.pad_damping_ns_m+(p.stop_damping_ns_m if bottom>0 else 0.)
    force=spring+damping*max(0.,-v)
    stored=.5*p.pad_stiffness_n_m*compression**2+p.pad_stiffness_n_m*p.stroke_m*bottom+.5*p.stop_stiffness_n_m*bottom**2
    return force,stored,damping*min(0.,v)**2,bottom>0

def simulate(p:Parameters,dt:float=2.5e-5,duration_s:float=1.2):
    if any(not math.isfinite(x) for x in asdict(p).values()) or p.mass_kg<=0 or p.drop_height_m<0 or p.stroke_m<0 or p.pad_stiffness_n_m<0 or p.pad_damping_ns_m<0 or p.stop_stiffness_n_m<=0 or p.stop_damping_ns_m<0 or p.gravity_m_s2<=0 or not 0<dt<=.001:
        raise ValueError('Invalid component-model parameters')
    y,v,diss=p.drop_height_m,0.,0.
    e0=p.mass_kg*p.gravity_m_s2*y
    rows=[];peak=0.;max_compression=0.;max_residual=0.;bottomed=False
    def derivative(s):
        force,_,power,_=contact(s[0],s[1],p)
        return s[1],-p.gravity_m_s2+force/p.mass_kg,power
    def add(s,k,scale):return tuple(a+scale*b for a,b in zip(s,k))
    for i in range(round(duration_s/dt)+1):
        force,stored,_,bottom=contact(y,v,p)
        energy=.5*p.mass_kg*v*v+p.mass_kg*p.gravity_m_s2*y+stored+diss
        residual=energy-e0
        peak=max(peak,force);max_compression=max(max_compression,-y);max_residual=max(max_residual,abs(residual));bottomed|=bottom
        if i%4==0:rows.append((i*dt,y,v,force,max(0.,-y),stored,diss,residual))
        state=(y,v,diss);a=derivative(state);b=derivative(add(state,a,dt/2));c=derivative(add(state,b,dt/2));d=derivative(add(state,c,dt))
        y,v,diss=tuple(x+dt*(ka+2*kb+2*kc+kd)/6 for x,ka,kb,kc,kd in zip(state,a,b,c,d))
    summary={'parameters':asdict(p),'time_step_s':dt,'duration_s':duration_s,'initial_energy_j':e0,'peak_contact_force_n':peak,'max_compression_m':max_compression,'bottomed_out':bottomed,'max_energy_residual_j':max_residual,'max_energy_residual_fraction':max_residual/max(e0,1.),'dissipated_energy_final_j':rows[-1][6],
             'evidence':'synthetic_1d_component_model_not_whole_body_or_injury_validation'}
    return rows,summary

def selected_results():
    selected=json.loads((ROOT/'cushion_delivery.json').read_text(encoding='utf-8'))['run_id']
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,80}',selected):raise ValueError('Invalid component evidence ID')
    for path in (ROOT/'results/cushion/experiments'/selected,ROOT/'evidence/cushion'/selected):
        if path.is_dir():return path
    raise FileNotFoundError('Selected component evidence is not available')

def prepare(run_id=None):
    run_id=run_id or datetime.now(timezone.utc).strftime('cushion_%Y%m%dT%H%M%S_%fZ')
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,80}',run_id):raise ValueError('Invalid component run ID')
    if (ROOT/'evidence/cushion'/run_id).exists():raise FileExistsError('Component ID belongs to preserved delivery evidence')
    import zipfile
    for archive in ROOT.glob('evidence_bundle*.zip'):
        with zipfile.ZipFile(archive) as z:
            if any(name.startswith('cushion/'+run_id+'/') for name in z.namelist()):
                raise FileExistsError('Component ID belongs to immutable archived evidence')
    target=ROOT/'results/cushion/experiments'/run_id
    target.mkdir(parents=True,exist_ok=False)
    base=Parameters()
    cases={'hard_contact':replace(base,stroke_m=0.,pad_stiffness_n_m=0.,pad_damping_ns_m=0.),'cushioned':base,'insufficient_stroke':replace(base,stroke_m=.02)}
    summaries={}
    for name,p in cases.items():
        rows,summary=simulate(p)
        with (target/(name+'.csv')).open('w',encoding='utf-8',newline='') as f:
            writer=csv.writer(f);writer.writerow(['time_s','height_m','velocity_m_s','contact_force_n','compression_m','spring_energy_j','dissipated_energy_j','energy_residual_j']);writer.writerows(rows)
        summaries[name]=summary
    hard=summaries['hard_contact']['peak_contact_force_n'];soft=summaries['cushioned']['peak_contact_force_n']
    summaries['comparison']={'peak_force_reduction_fraction':1-soft/hard,'same_effective_mass_and_drop':True,'protected_body_part':'unspecified_component_effective_mass','deployment_model':'prepositioned_passive_pad; trigger signal is not physical deployment','whole_body_coupling':'not implemented'}
    (target/'summary.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2),encoding='utf-8')
    source_hash=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (target/'provenance.json').write_text(json.dumps({'run_id':run_id,'source_sha256':source_hash,'model':'finite_stroke_1d_component','parameters':asdict(base),'created_utc':datetime.now(timezone.utc).isoformat()},indent=2),encoding='utf-8')
    print(json.dumps({k:{n:v for n,v in s.items() if n in ('peak_contact_force_n','max_compression_m','bottomed_out','max_energy_residual_fraction')} for k,s in summaries.items()}))
    return summaries

if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--run');args=parser.parse_args();prepare(args.run)
