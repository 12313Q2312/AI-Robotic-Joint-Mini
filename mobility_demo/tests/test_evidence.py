import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from analyze import command_positive_work

class WorkEvidenceTests(unittest.TestCase):
    def test_constant_torque_work_has_correct_si_units(self):
        self.assertAlmostEqual(command_positive_work([0,.25,1],[0,.75,3],[2,2,2]),6)
    def test_uses_command_held_during_actual_angle_increment(self):
        self.assertEqual(command_positive_work([0,1,2],[0,1,2],[2,0,100]),2)
    def test_negative_work_is_not_counted_as_positive(self):
        self.assertEqual(command_positive_work([0,1,2],[0,1,0],[2,2,2]),2)
    def test_corrupt_evidence_is_rejected(self):
        for t,q,tau in [([0,0],[0,1],[1,1]),([0,1],[0,float('nan')],[1,1]),([0,1],[0,1],[1])]:
            with self.assertRaises(ValueError):command_positive_work(t,q,tau)

if __name__=='__main__':unittest.main()
