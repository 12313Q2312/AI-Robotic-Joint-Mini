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
from manual_panel_core import ControlPanel
from manual_panel_server import create_server


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.panel = ControlPanel()
        web = pathlib.Path(__file__).resolve().parents[1] / 'ros2_ws/src/ai_robotic_joint_ros2/web'
        self.server = create_server(self.panel, web, port=0)
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

    def post(self, values, token=None, origin=None):
        headers = {'Content-Type': 'application/json', 'X-Panel-Token': token or self.token}
        if origin: headers['Origin'] = origin
        request = urllib.request.Request(self.url + '/api/control', json.dumps(values).encode(), headers)
        with urllib.request.urlopen(request) as response:
            return json.load(response)

    def test_no_fake_feedback_and_real_control_requires_session(self):
        with urllib.request.urlopen(self.url + '/api/state') as response:
            self.assertFalse(json.load(response)['connected'])
        session = self.post({'action': 'claim'})['session']
        self.post({'action': 'mode', 'session': session, 'mode': 'angle'})
        with self.assertRaises(urllib.error.HTTPError) as failure:
            self.post({'action': 'angle', 'session': session, 'angle_deg': 30})
        self.assertEqual(failure.exception.code, 400)

    def test_foreign_origin_and_wrong_token_are_rejected(self):
        for kwargs in ({'token': 'wrong'}, {'origin': 'https://foreign.invalid'}):
            with self.assertRaises(urllib.error.HTTPError) as failure:
                self.post({'action': 'claim'}, **kwargs)
            self.assertEqual(failure.exception.code, 403)

    def test_no_directory_traversal_and_all_assets_load(self):
        for path in ('/panel.js', '/panel.css'):
            with urllib.request.urlopen(self.url + path) as response:
                self.assertEqual(response.status, 200)
        with self.assertRaises(urllib.error.HTTPError):
            urllib.request.urlopen(self.url + '/../../AGENTS.md')

    def test_foreign_host_cannot_read_session_token(self):
        request = urllib.request.Request(self.url, headers={'Host': 'foreign.invalid:8765'})
        with self.assertRaises(urllib.error.HTTPError) as failure:
            urllib.request.urlopen(request)
        self.assertEqual(failure.exception.code, 403)


if __name__ == '__main__': unittest.main()
