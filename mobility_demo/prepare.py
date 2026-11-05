"""Generate a self-contained primitive adult biped; no external PROTO downloads."""
from pathlib import Path
import argparse
import math

ROOT=Path(__file__).resolve().parent
HUMAN_MASS_SCALE=1.0

def camera_rotation(eye,target):
    def norm(v):return [x/math.sqrt(sum(t*t for t in v)) for x in v]
    def cross(a,b):return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
    forward=norm([b-a for a,b in zip(eye,target)])
    right=norm(cross(forward,[0,0,1]));up=cross(right,forward)
    r=[[forward[i],-right[i],up[i]] for i in range(3)]
    angle=math.acos(max(-1,min(1,(sum(r[i][i] for i in range(3))-1)/2)))
    axis=[r[2][1]-r[1][2],r[0][2]-r[2][0],r[1][0]-r[0][1]]
    axis=norm(axis)
    return ' '.join(str(x) for x in (*axis,angle))

def shape(geometry, color):
    return f'Shape {{ appearance PBRAppearance {{ baseColor {color} roughness .8 }} geometry {geometry} }}'

def box(size, color):
    return shape(f'Box {{ size {size} }}',color)

def hinge(name, axis, children, mass, bounds, translation='0 0 0',com='0 0 0',lo=-1.3,hi=1.3,definition=''):
    return f'''HingeJoint {{
      jointParameters HingeJointParameters {{ axis {axis} anchor {translation} minStop {lo} maxStop {hi} dampingConstant .2 }}
      device [ RotationalMotor {{ name "{name}" maxTorque 240 maxVelocity 12 }} PositionSensor {{ name "{name}_sensor" }} ]
      endPoint {definition} Solid {{ translation {translation} children [ {children} ]
        name "{name}_link" boundingObject {bounds}
        physics Physics {{ density -1 mass {mass} centerOfMass [ {com} ] }}
      }}
    }}'''

def leg(side,device=True):
    s=-1 if side=='r' else 1
    foot_geometry='Box { size .28 .14 .09 }'
    foot=hinge(side+'_foot','1 0 0',f'Transform {{ translation .045 0 -.035 children [ {box(".28 .14 .09",".12 .18 .24")} ] }}',1.5*HUMAN_MASS_SCALE+(.2 if device else 0),
        f'Transform {{ translation .045 0 -.035 children [ {foot_geometry} ] }}',com='.045 0 -.035',definition=f'DEF {side.upper()}_FOOT',lo=-.8,hi=.8)
    ankle=hinge(side+'_ankle','0 1 0',foot,.06*HUMAN_MASS_SCALE,'Sphere { radius .015 }',translation='0 0 -.42',lo=-.9,hi=.9)
    shin_geo='Transform { translation 0 0 -.21 children [ Box { size .085 .09 .42 } ] }'
    shin_visual=shape('Capsule { radius .044 height .332 }','.22 .31 .42')
    exo_shin=f'Transform {{ translation .025 {s*.065} -.21 children [ {box(".028 .024 .34",".08 .65 .65")} ] }}' if device else ''
    gear=f'Transform {{ translation 0 {s*.065} 0 children [ {shape("Sphere { radius .043 }",".10 .24 .30")} ] }}' if device else ''
    shin=f'Transform {{ translation 0 0 -.21 children [ {shin_visual} ] }} {exo_shin} {gear} {ankle}'
    knee=hinge(side+'_knee','0 1 0',shin,3.5*HUMAN_MASS_SCALE+(.4 if device else 0),shin_geo,translation='0 0 -.42',com='0 0 -.21',lo=-.12,hi=2.3,definition=f'DEF {side.upper()}_KNEE')
    exo_thigh=f'Transform {{ translation .025 {s*.080} -.21 children [ {box(".032 .026 .32",".08 .65 .65")} ] }}' if device else ''
    thigh=f'Transform {{ translation 0 0 -.21 children [ {shape("Capsule { radius .065 height .29 }",".22 .31 .42")} ] }} {exo_thigh} {knee}'
    thigh_bounds='Transform { translation 0 0 -.21 children [ Box { size .12 .13 .42 } ] }'
    hip=hinge(side+'_hip','0 1 0',thigh,7.0*HUMAN_MASS_SCALE+(.6 if device else 0),thigh_bounds,com='0 0 -.21',lo=-1.5,hi=.9,definition=f'DEF {side.upper()}_HIP')
    return hinge(side+'_roll','1 0 0',hip,.06*HUMAN_MASS_SCALE,'Sphere { radius .015 }',translation=f'0 {s*.115} 0',lo=-.65,hi=.65)

def generate(scenario='stand',assisted=True,run='smoke',device=True,capture=False):
    for directory in ('worlds','runs'): (ROOT/directory).mkdir(exist_ok=True)
    body=f'''{box('.24 .29 .16','.15 .25 .35')}
      Transform {{ translation 0 0 .30 scale .90 1.20 1 children [ {shape('Capsule { radius .13 height .20 }','.80 .84 .88')} ] }}
      Transform {{ translation 0 0 .64 children [ {shape('Sphere { radius .105 }','.78 .63 .49')} ] }}
      Transform {{ translation .104 0 .64 children [ {shape('Sphere { radius .022 }','.78 .63 .49')} ] }}
      Transform {{ translation .09 -.035 .67 children [ {shape('Sphere { radius .009 }','.08 .10 .12')} ] }}
      Transform {{ translation .09 .035 .67 children [ {shape('Sphere { radius .009 }','.08 .10 .12')} ] }}
      Transform {{ translation -.018 0 .702 scale .94 1 .48 children [ {shape('Sphere { radius .102 }','.69 .71 .73')} ] }}
      Transform {{ translation 0 -.235 .30 children [ {shape('Capsule { radius .042 height .35 }','.8 .84 .88')} ] }}
      Transform {{ translation 0 .235 .30 children [ {shape('Capsule { radius .042 height .35 }','.8 .84 .88')} ] }}
      Transform {{ translation 0 -.235 .09 children [ {shape('Sphere { radius .047 }','.78 .63 .49')} ] }}
      Transform {{ translation 0 .235 .09 children [ {shape('Sphere { radius .047 }','.78 .63 .49')} ] }}
      {leg('r',device)} {leg('l',device)}'''
    stairs=''
    if scenario=='stairs':
        for i in range(3):
            width=.40 if i<2 else 1.6
            height=.1*(i+1);x=.20+.40*i+width/2
            stairs+=f'Solid {{ name "stair_{i+1}" translation {x} 0 {height/2} children [ {box(f"{width} 1.5 {height}",".61 .69 .78")} ] boundingObject Box {{ size {width} 1.5 {height} }} }}\n'
    world=f'''#VRML_SIM R2025a utf8
# Synthetic adult and ideal state observations; not a validated elderly model.
WorldInfo {{ basicTimeStep 2 gravity 9.81 coordinateSystem "ENU" }}
Viewpoint {{ orientation {camera_rotation([3.0,-4.8,2.2],[.6,0,.95])} position 3.0 -4.8 2.2 }}
Background {{ skyColor [ .86 .91 .96 ] }}
DirectionalLight {{ direction -.5 .5 -1 intensity 1.5 ambientIntensity .6 }}
Solid {{ name "floor" translation 2 0 -.05 children [ {box('8 4 .1','.91 .92 .94')} ] boundingObject Box {{ size 8 4 .1 }} }}
{stairs}
DEF HUMAN Robot {{ translation 0 0 .923 children [ {body} ]
  name "synthetic_human_exoskeleton" controller "mobility"
  controllerArgs [ "{scenario}" "{'assist' if assisted else 'off'}" "../../runs/{run}.csv" {('"capture"' if capture else '')} ]
  supervisor TRUE selfCollision FALSE
  boundingObject Transform {{ translation 0 0 .26 children [ Box {{ size .24 .34 .64 }} ] }}
  customData "{70.24*HUMAN_MASS_SCALE+(4 if device else 0)}"
  physics Physics {{ density -1 mass {46*HUMAN_MASS_SCALE+(1.6 if device else 0)} centerOfMass [ 0 0 .28 ] }}
}}
'''
    path=ROOT/'worlds'/f'{run}.wbt'
    path.write_text(world,encoding='utf-8')
    return path

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scenario',default='stand');p.add_argument('--mode',choices=['assist','off'],default='assist');p.add_argument('--run',default='smoke')
    a=p.parse_args();print(generate(a.scenario,a.mode=='assist',a.run))
