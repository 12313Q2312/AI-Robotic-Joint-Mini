import sys
from pathlib import Path
import unittest
import tempfile
import zipfile
from unittest.mock import patch
import cushion
from dataclasses import replace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from cushion import Parameters,contact,simulate

class ComponentPhysicsTests(unittest.TestCase):
    def test_unilateral_contact_has_no_force_before_touch(self):
        self.assertEqual(contact(.01,-2.,Parameters())[0],0.)
        self.assertGreaterEqual(contact(-.02,5.,Parameters())[0],0.)
    def test_energy_and_timestep_convergence(self):
        p=Parameters();_,a=simulate(p,dt=5e-5,duration_s=.7);_,b=simulate(p,dt=2.5e-5,duration_s=.7)
        self.assertLess(a['max_energy_residual_fraction'],.01)
        self.assertLess(b['max_energy_residual_fraction'],.01)
        self.assertLess(abs(a['peak_contact_force_n']-b['peak_contact_force_n'])/b['peak_contact_force_n'],.02)
        self.assertFalse(b['bottomed_out'])
    def test_insufficient_stroke_exposes_bottom_out(self):
        _,result=simulate(replace(Parameters(),stroke_m=.02),duration_s=.7)
        self.assertTrue(result['bottomed_out'])
    def test_invalid_mass_is_rejected(self):
        with self.assertRaises(ValueError):simulate(replace(Parameters(),mass_kg=0))
    def test_existing_experiment_is_never_overwritten(self):
        scratch=Path(__file__).resolve().parents[1]/'results/test-temporaries';scratch.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(dir=scratch,prefix='immutable-test-') as tmp,patch.object(cushion,'ROOT',Path(tmp)):
            self.assertTrue(Path(tmp).resolve().is_relative_to(scratch.resolve()))
            cushion.prepare('fixed_run')
            path=Path(tmp)/'results/cushion/experiments/fixed_run/cushioned.csv'
            before=path.read_bytes()
            with self.assertRaises(FileExistsError):cushion.prepare('fixed_run')
            self.assertEqual(path.read_bytes(),before)
    def test_unpacked_and_archived_experiment_ids_are_reserved(self):
        scratch=Path(__file__).resolve().parents[1]/'results/test-temporaries';scratch.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(dir=scratch,prefix='frozen-id-') as tmp,patch.object(cushion,'ROOT',Path(tmp)):
            root=Path(tmp)
            (root/'evidence/cushion/unpacked').mkdir(parents=True)
            with zipfile.ZipFile(root/'evidence_bundle_v02.zip','x') as archive:
                archive.writestr('cushion/archived/summary.json','{}')
            for name in ('unpacked','archived'):
                with self.assertRaises(FileExistsError):cushion.prepare(name)
                self.assertFalse((root/'results/cushion/experiments'/name).exists())

if __name__=='__main__':unittest.main()
