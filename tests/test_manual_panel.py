import pathlib
import sys
import unittest
import math

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] /
                      'ros2_ws/src/ai_robotic_joint_ros2/scripts'))
from manual_panel_core import ControlPanel


class PanelTests(unittest.TestCase):
    def setUp(self):
        self.clock = [10.0]
        self.panel = ControlPanel(lambda: self.clock[0])
        self.session = self.panel.handle({'action': 'claim'})['session']
        self.panel.feedback({'sim_time_s': 1.0, 'actual_angle_rad': 0,
                             'reference_angle_rad': 0, 'velocity_rad_s': 0})

    def send(self, action, **values):
        return self.panel.handle(dict(action=action, session=self.session, **values))

    def test_motion_requires_live_feedback_and_angles_are_converted(self):
        clock = [10.0]
        panel = ControlPanel(lambda: clock[0])
        session = panel.handle({'action': 'claim'})['session']
        panel.handle({'action': 'mode', 'session': session, 'mode': 'angle'})
        with self.assertRaises(ValueError):
            panel.handle({'action': 'angle', 'session': session, 'angle_deg': 30})
        panel.feedback({'sim_time_s': 1, 'actual_angle_rad': 0,
                        'reference_angle_rad': 0, 'velocity_rad_s': 0})
        panel.handle({'action': 'angle', 'session': session, 'angle_deg': 30})
        command = panel.command()
        self.assertEqual(command['action'], 1)
        self.assertAlmostEqual(command['target_angle_rad'], 0.5235987755982988)

    def test_release_opposite_keys_and_repeat(self):
        self.send('keys', keys=['ArrowUp'])
        first = self.panel.command()
        self.send('keys', keys=['ArrowUp'])
        self.assertEqual(first, self.panel.command())
        self.send('keys', keys=['ArrowUp', 'ArrowDown'])
        self.assertEqual(self.panel.command()['action'], 0)
        self.send('keys', keys=['ArrowDown'])
        self.assertLess(self.panel.command()['target_angle_rad'], 0)
        self.send('keys', keys=[])
        self.assertEqual(self.panel.command()['action'], 0)

    def test_fine_steps_are_single_commands_and_accumulate(self):
        self.send('nudge', direction=1)
        self.assertAlmostEqual(self.panel.command()['target_angle_rad'], math.radians(.25))
        self.send('nudge', direction=1)
        self.assertAlmostEqual(self.panel.command()['target_angle_rad'], math.radians(.5))
        self.send('nudge', direction=-1)
        self.assertAlmostEqual(self.panel.command()['target_angle_rad'], math.radians(.25))

    def test_ai_switch_cancels_motion_and_never_executes(self):
        self.send('keys', keys=['ArrowUp'])
        self.send('mode', mode='ai')
        self.assertEqual(self.panel.command()['action'], 0)
        with self.assertRaises(ValueError): self.send('angle', angle_deg=30)
        self.send('mode', mode='angle')
        self.assertEqual(self.panel.command()['action'], 0)

    def test_invalid_angles_leave_existing_command_unchanged(self):
        self.send('mode', mode='angle')
        for value in ('', None, True, 'NaN', 'inf', 45.001, -46, 'hello'):
            previous = self.panel.command()
            with self.assertRaises(ValueError): self.send('angle', angle_deg=value)
            self.assertEqual(previous, self.panel.command())
        self.send('angle', angle_deg=-45)
        self.assertAlmostEqual(self.panel.command()['target_angle_rad'], math.radians(-45))

    def test_heartbeat_loss_cancels_and_releases_ownership(self):
        self.send('keys', keys=['ArrowUp'])
        self.clock[0] += .51
        self.panel.feedback({'sim_time_s': 1.51, 'actual_angle_rad': .01,
                             'reference_angle_rad': .02, 'velocity_rad_s': .01})
        command = self.panel.command()
        self.assertEqual(command['action'], 0)
        self.assertAlmostEqual(command['target_angle_rad'], .02)
        with self.assertRaises(ValueError): self.send('keys', keys=['ArrowUp'])

    def test_feedback_loss_and_repeated_timestamps_cannot_arm(self):
        self.send('keys', keys=['ArrowUp'])
        self.clock[0] += .4
        self.send('heartbeat')
        self.clock[0] += .11
        self.panel.feedback({'sim_time_s': 1, 'actual_angle_rad': 0,
                             'reference_angle_rad': 0, 'velocity_rad_s': 0})
        self.assertEqual(self.panel.command()['action'], 0)
        self.assertFalse(self.panel.state()['connected'])

    def test_reset_cancels_and_uses_new_reference(self):
        self.send('keys', keys=['ArrowUp'])
        self.panel.feedback({'sim_time_s': 0, 'actual_angle_rad': .1,
                             'reference_angle_rad': .1, 'velocity_rad_s': 0})
        self.assertEqual(self.panel.command()['action'], 0)
        self.assertAlmostEqual(self.panel.command()['target_angle_rad'], .1)

    def test_second_page_cannot_take_control_or_stop_first(self):
        with self.assertRaises(ValueError): self.panel.handle({'action': 'claim'})
        self.send('keys', keys=['ArrowUp'])
        with self.assertRaises(ValueError): self.panel.handle({'action': 'stop', 'session': 'other'})
        self.assertEqual(self.panel.command()['action'], 1)

    def test_foreign_mode_and_fault_feedback_reject_motion(self):
        with self.assertRaises(ValueError): self.send('angle', angle_deg=5)
        self.panel.feedback({'sim_time_s': 2, 'actual_angle_rad': 0,
                             'reference_angle_rad': 0, 'velocity_rad_s': 0, 'status': 'fault'})
        with self.assertRaises(ValueError): self.send('keys', keys=['ArrowUp'])


if __name__ == '__main__':
    unittest.main()
