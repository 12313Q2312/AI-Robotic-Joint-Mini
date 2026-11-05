import copy
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze import alternating,pair_metadata_valid

class AcceptanceRuleTests(unittest.TestCase):
    def pair(self):
        required=['controllers/mobility/mobility.cpp','include/AssistPolicy.hpp','include/PlanGate.hpp','include/RiskMonitor.hpp','prepare.py']
        a={'scenario':'flat','mode':'off','mass_kg':74.24,'duration_s':35.,'completed':True,'alternating_touchdowns':True,'simulation_source_sha256':{k:'a'*64 for k in required}}
        b=copy.deepcopy(a);b['mode']='assist';return a,b
    def test_requires_chronological_alternation(self):
        events=[{'time_s':i,'side':'r' if i%2==0 else 'l'} for i in range(6)]
        self.assertTrue(alternating(events))
        for i in range(3):events[i]['side']='r'
        for i in range(3,6):events[i]['side']='l'
        self.assertFalse(alternating(events))
    def test_valid_pair_can_be_compared(self):
        a,b=self.pair();self.assertTrue(pair_metadata_valid(a,b,True,.01,.03))
    def test_missing_provenance_never_passes_vacuously(self):
        a,b=self.pair();a['simulation_source_sha256']={};b['simulation_source_sha256']={}
        self.assertFalse(pair_metadata_valid(a,b,True,.01,.03))
    def test_mass_time_task_and_trajectory_mismatch_are_rejected(self):
        for key,value in [('mass_kg',70.24),('duration_s',30),('scenario','stairs')]:
            a,b=self.pair();b[key]=value;self.assertFalse(pair_metadata_valid(a,b,True,.01,.03))
        a,b=self.pair();self.assertFalse(pair_metadata_valid(a,b,False,.01,.03))
        self.assertFalse(pair_metadata_valid(a,b,True,.2,.03))
        self.assertFalse(pair_metadata_valid(a,b,True,.01,.2))

if __name__=='__main__':unittest.main()
