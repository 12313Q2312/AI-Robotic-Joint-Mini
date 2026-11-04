"""Paired supervision tests. Synthetic input here is not Webots evidence."""
import math
import pathlib
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] /
                      'ros2_ws/src/ai_robotic_joint_ros2/scripts'))
from study_panel_core import StudyPanel


def feedback(time=1.0, **changes):
    side = dict(sim_time_s=time, actual_angle_rad=0.0, reference_angle_rad=0.0,
                requested_angle_rad=0.0, reference_velocity_rad_s=0.0,
                velocity_rad_s=0.0, commanded_torque_nm=0.1, status='ready')
    result = dict(baseline=dict(side), candidate=dict(side), payload_mass_kg=.25,
                  spring_compression_m=.01, compensation_torque_nm=.1,
                  loss_torque_nm=0.0, candidate_label='F-N fixture',
                  model_status='fixture_geometry_unoptimized', can_reset=True,
                  reset_sequence=0)
    result.update(changes)
    return result


class StudyPanelTests(unittest.TestCase):
    def test_server_restart_uses_a_newer_ros_command_sequence(self):
        with patch('study_panel_core.time.monotonic_ns', side_effect=(100, 200)):
            first, second = StudyPanel(lambda: 10.0), StudyPanel(lambda: 10.0)
        first.handle({'action': 'claim'})
        second.handle({'action': 'claim'})
        self.assertGreater(second.command()['control']['sequence'], first.command()['control']['sequence'])

    def test_pending_reset_is_cancelled_on_world_rewind_or_feedback_timeout(self):
        for rewind in (True, False):
            clock = [10.0]
            panel = StudyPanel(lambda: clock[0])
            session = panel.handle({'action': 'claim'})['session']
            panel.feedback(feedback())
            panel.handle(dict(action='reset', session=session, payload_index=0))
            self.assertTrue(panel.state()['reset_pending'])
            if rewind:
                panel.feedback(feedback(time=0.0))
            else:
                clock[0] += .49
                panel.handle(dict(action='heartbeat', session=session))
                clock[0] += .03
            command = panel.command()
            self.assertFalse(panel.state()['reset_pending'])
            self.assertEqual(command['reset_sequence'], 0)
            self.assertEqual(command['control']['action'], 0)

    def test_production_requires_order_id_and_foreign_owner_cannot_consume_it(self):
        panel = StudyPanel(lambda: 10.0, require_action_ids=True)
        session = panel.handle({'action': 'claim'})['session']
        panel.feedback(feedback())
        for identity in (None, True, 0, '1'):
            with self.assertRaises(ValueError):
                panel.handle(dict(action='keys', keys=['ArrowUp'], session=session, action_id=identity))
        with self.assertRaises(ValueError):
            panel.handle(dict(action='stop', session='foreign', action_id=100))
        panel.handle(dict(action='keys', keys=['ArrowUp'], session=session, action_id=1))
        self.assertEqual(panel.command()['control']['action'], 1)

    def test_late_move_cannot_undo_a_newer_stop(self):
        panel = StudyPanel(lambda: 10.0)
        session = panel.handle({'action': 'claim'})['session']
        panel.feedback(feedback())
        panel.handle(dict(action='stop', session=session, action_id=2))
        with self.assertRaises(ValueError):
            panel.handle(dict(action='keys', keys=['ArrowUp'], session=session, action_id=1))
        self.assertEqual(panel.command()['control']['action'], 0)

    def test_payload_only_changes_via_explicit_stopped_reset_and_ack(self):
        panel = StudyPanel(lambda: 10.0)
        session = panel.handle({'action': 'claim'})['session']
        panel.feedback(feedback())
        panel.handle(dict(action='keys', session=session, keys=['ArrowUp']))
        with self.assertRaises(ValueError):
            panel.handle(dict(action='reset', session=session, payload_index=0))
        self.assertEqual(panel.command()['payload_index'], 2)
        panel.handle(dict(action='stop', session=session))
        panel.handle(dict(action='reset', session=session, payload_index=0))
        command = panel.command()
        self.assertEqual(command['payload_index'], 0)
        self.assertGreater(command['reset_sequence'], 0)
        self.assertEqual(command['control']['action'], 0)
        self.assertTrue(panel.state()['reset_pending'])
        with self.assertRaises(ValueError):
            panel.handle(dict(action='keys', session=session, keys=['ArrowUp']))
        panel.feedback(feedback(time=1.1, reset_sequence=command['reset_sequence'], payload_mass_kg=0))
        self.assertFalse(panel.state()['reset_pending'])
        self.assertEqual(panel.command()['control']['action'], 0)

    def test_both_sides_must_be_fresh_and_synchronized_before_motion(self):
        clock = [10.0]
        panel = StudyPanel(lambda: clock[0])
        session = panel.handle({'action': 'claim'})['session']
        command = dict(action='keys', keys=['ArrowUp'], session=session)
        with self.assertRaises(ValueError): panel.handle(command)
        panel.feedback(feedback(candidate=dict(feedback()['candidate'], sim_time_s=1.01)))
        with self.assertRaises(ValueError): panel.handle(command)
        panel.feedback(feedback())
        panel.handle(command)
        self.assertEqual(panel.command()['control']['action'], 1)
        self.assertAlmostEqual(panel.command()['control']['target_angle_rad'], math.pi / 4)
        clock[0] += .51
        self.assertFalse(panel.state()['connected'])
        self.assertEqual(panel.command()['control']['action'], 0)

    def test_automatic_experiment_releases_human_control_and_never_rearms(self):
        panel = StudyPanel(lambda: 10.0)
        session = panel.handle({'action': 'claim'})['session']
        panel.feedback(feedback())
        panel.handle(dict(action='keys', keys=['ArrowUp'], session=session))
        panel.set_experiment_active(True)
        self.assertEqual(panel.command()['control']['action'], 0)
        with self.assertRaises(ValueError): panel.handle({'action': 'claim'})
        panel.set_experiment_active(False)
        session = panel.handle({'action': 'claim'})['session']
        self.assertEqual(panel.command()['control']['action'], 0)

    def test_one_side_fault_and_duplicate_timestamp_cannot_keep_motion_alive(self):
        clock = [10.0]
        panel = StudyPanel(lambda: clock[0])
        session = panel.handle({'action': 'claim'})['session']
        panel.feedback(feedback())
        panel.handle(dict(action='keys', keys=['ArrowUp'], session=session))
        bad = feedback(time=1.1)
        bad['candidate']['status'] = 'fault_pause_webots'
        panel.feedback(bad)
        self.assertFalse(panel.state()['connected'])
        self.assertEqual(panel.command()['control']['action'], 0)
        panel.feedback(feedback(time=1.2))
        panel.handle(dict(action='keys', keys=['ArrowUp'], session=session))
        clock[0] += .4
        panel.handle(dict(action='heartbeat', session=session))
        clock[0] += .11
        panel.feedback(feedback(time=1.2))
        self.assertFalse(panel.state()['connected'])
        self.assertEqual(panel.command()['control']['action'], 0)

    def test_reset_and_heartbeat_never_restart_old_motion_and_foreign_owner_rejected(self):
        panel = StudyPanel(lambda: 10.0)
        session = panel.handle({'action': 'claim'})['session']
        panel.feedback(feedback())
        panel.handle(dict(action='keys', keys=['ArrowUp'], session=session))
        with self.assertRaises(ValueError):
            panel.handle(dict(action='stop', session='foreign'))
        panel.feedback(feedback(time=0.0))
        self.assertEqual(panel.command()['control']['action'], 0)
        panel.handle(dict(action='heartbeat', session=session))
        self.assertEqual(panel.command()['control']['action'], 0)
        panel.handle(dict(action='mode', mode='ai', session=session))
        with self.assertRaises(ValueError):
            panel.handle(dict(action='angle', angle_deg=30, session=session))
        for index in (-1, 3, True, '1'):
            with self.assertRaises(ValueError):
                panel.handle(dict(action='reset', payload_index=index, session=session))


if __name__ == '__main__': unittest.main()
