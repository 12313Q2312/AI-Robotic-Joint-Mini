"""Actual loopback HTTP with synthetic feedback: NOT a Webots experiment."""
import json
import pathlib
import re
import sys
import threading
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] /
                      'ros2_ws/src/ai_robotic_joint_ros2/scripts'))
from manual_panel_server import create_server
from study_panel_core import StudyPanel
from study_panel_server import engineering_status
from test_study_panel import feedback


class StudyHttpTests(unittest.TestCase):
    def setUp(self):
        self.panel = StudyPanel(require_action_ids=True)
        web = pathlib.Path(__file__).resolve().parents[1] / 'ros2_ws/src/ai_robotic_joint_ros2/study_web'
        self.server = create_server(self.panel, web, port=0,
                                    report_provider=lambda: engineering_status(self.panel))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = 'http://127.0.0.1:' + str(self.server.server_port)
        with urllib.request.urlopen(self.url) as response:
            self.page = response.read().decode()
        self.token = re.search(r'name="panel-token" content="([^"]+)"', self.page)[1]

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def post(self, payload, **headers):
        fields = {'Content-Type': 'application/json', 'X-Panel-Token': self.token}
        fields.update(headers)
        request = urllib.request.Request(self.url + '/api/control', json.dumps(payload).encode(), fields)
        with urllib.request.urlopen(request) as response:
            return json.load(response)

    def test_new_assets_load_and_preview_never_fabricates_robot_state(self):
        with urllib.request.urlopen(self.url + '/api/state') as response:
            state = json.load(response)
        self.assertEqual(state['app'], 'joint-mini-study')
        self.assertFalse(state['connected'])
        self.assertIsNone(state['telemetry'])
        self.assertFalse(state['can_reset'])
        self.assertNotIn('__PANEL_TOKEN__', self.page)
        for name in ('/panel.js', '/panel.css'):
            with urllib.request.urlopen(self.url + name) as response:
                self.assertEqual(response.status, 200)
        with self.assertRaises(urllib.error.HTTPError):
            urllib.request.urlopen(self.url + '/../../AGENTS.md')

    def test_engineering_report_exposes_status_not_private_files_or_invented_acceptance(self):
        with urllib.request.urlopen(self.url + '/api/report') as response:
            report = json.load(response)
        self.assertFalse(report['paired_feedback_fresh'])
        self.assertEqual(report['webots_motion_acceptance'], 'pending')
        self.assertEqual(report['native_cad_acceptance'], 'pending')
        self.assertEqual(report['public_trial_acceptance'], 'pending')
        self.assertNotIn('session', report)
        self.assertNotIn('savings_percent', report)
        self.assertTrue(report['project_document_index'])

    def test_token_origin_and_host_boundaries_are_preserved(self):
        for headers in ({'X-Panel-Token': 'wrong'}, {'Origin': 'https://foreign.invalid'},
                        {'Host': 'foreign.invalid:8766'}):
            with self.assertRaises(urllib.error.HTTPError) as failure:
                self.post({'action': 'claim'}, **headers)
            self.assertEqual(failure.exception.code, 403)

    def test_http_rejects_missing_or_reordered_actions_and_no_feedback_motion(self):
        session = self.post({'action': 'claim'})['session']
        command = dict(action='keys', keys=['ArrowUp'], session=session)
        with self.assertRaises(urllib.error.HTTPError): self.post(command)
        with self.assertRaises(urllib.error.HTTPError): self.post(dict(command, action_id=1))
        self.panel.feedback(feedback())
        self.post(dict(command, action_id=2))
        self.post(dict(action='stop', session=session, action_id=4))
        with self.assertRaises(urllib.error.HTTPError): self.post(dict(command, action_id=3))
        self.assertEqual(self.panel.command()['control']['action'], 0)

    def test_http_single_owner_reset_requires_real_ack_and_ai_stays_placeholder(self):
        session = self.post({'action': 'claim'})['session']
        with self.assertRaises(urllib.error.HTTPError): self.post({'action': 'claim'})
        self.panel.feedback(feedback())
        self.post(dict(action='reset', session=session, action_id=1, payload_index=0))
        with self.assertRaises(urllib.error.HTTPError):
            self.post(dict(action='keys', keys=['ArrowUp'], session=session, action_id=2))
        self.panel.feedback(feedback(time=1.1, reset_sequence=1, payload_mass_kg=0))
        self.post(dict(action='mode', session=session, action_id=3, mode='ai'))
        with self.assertRaises(urllib.error.HTTPError):
            self.post(dict(action='angle', session=session, action_id=4, angle_deg=30))
        self.assertEqual(self.panel.command()['control']['action'], 0)


if __name__ == '__main__': unittest.main()
