#!/usr/bin/env python3
"""Loopback-only panel, ROS2 adapter and CSV recording. No third-party web stack."""
import argparse
import csv
import json
import pathlib
import secrets
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

from manual_panel_core import ControlPanel


def create_server(panel, web_dir, host='127.0.0.1', port=8765, report_provider=None):
    web_dir = pathlib.Path(web_dir)
    token = secrets.token_urlsafe(32)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def local_host(self):
            # Reject DNS rebinding: loopback binding alone is not sufficient.
            authority = self.headers.get('Host', '').lower()
            return authority in ('localhost:' + str(self.server.server_port),
                                 '127.0.0.1:' + str(self.server.server_port))

        def respond(self, status, body, content_type='application/json; charset=utf-8'):
            if isinstance(body, dict):
                body = json.dumps(body, ensure_ascii=False, allow_nan=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; frame-ancestors 'none'; object-src 'none'")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if not self.local_host():
                return self.respond(403, {'error': 'Local hostname required'})
            path = urlsplit(self.path).path
            if path == '/api/state':
                return self.respond(200, panel.state())
            if path == '/api/report' and report_provider is not None:
                # Opt-in, fixed engineering status only; never serve arbitrary
                # workspace paths, personal source files or control credentials.
                return self.respond(200, report_provider())
            files = {'/': ('index.html', 'text/html; charset=utf-8'),
                     '/panel.js': ('panel.js', 'text/javascript; charset=utf-8'),
                     '/panel.css': ('panel.css', 'text/css; charset=utf-8')}
            if path not in files:
                return self.respond(404, {'error': 'Not found'})
            name, mime = files[path]
            data = (web_dir / name).read_bytes()
            if path == '/':
                data = data.replace(b'__PANEL_TOKEN__', token.encode('ascii'))
            self.respond(200, data, mime)

        def do_POST(self):
            origin = self.headers.get('Origin')
            expected_origin = 'http://' + self.headers.get('Host', '')
            if (not self.local_host() or self.path != '/api/control' or self.headers.get('X-Panel-Token') != token or
                    (origin is not None and origin != expected_origin) or
                    self.headers.get('Content-Type', '').split(';')[0] != 'application/json'):
                return self.respond(403, {'error': 'Only this local panel may send commands'})
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096:
                    raise ValueError('Invalid request size')
                self.connection.settimeout(2)
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ValueError('Invalid command')
                self.respond(200, panel.handle(payload))
            except (ValueError, TypeError, TimeoutError) as error:
                self.respond(400, {'error': str(error)})

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--prefix', default='/manual_demo/joint')
    parser.add_argument('--web-dir')
    parser.add_argument('--preview', action='store_true', help='UI only; no ROS or fake feedback')
    parser.add_argument('--record', help='New CSV path; existing files are never overwritten')
    args, ros_args = parser.parse_known_args()
    panel = ControlPanel()
    node = None
    log_file = None
    if args.preview:
        default_web = pathlib.Path(__file__).resolve().parents[1] / 'web'
    else:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Float64
        from ai_robotic_joint_ros2.msg import ManualCommand, ControlTelemetry
        from ament_index_python.packages import get_package_share_directory
        rclpy.init(args=ros_args)
        default_web = pathlib.Path(get_package_share_directory('ai_robotic_joint_ros2')) / 'web'
        if args.record:
            target = pathlib.Path(args.record)
            target.parent.mkdir(parents=True, exist_ok=True)
            log_file = target.open('x', newline='', encoding='utf-8')
            names = list(ControlTelemetry.get_fields_and_field_types())
            writer = csv.DictWriter(log_file, fieldnames=names)
            writer.writeheader()

        class Bridge(Node):
            def __init__(self):
                super().__init__('manual_panel')
                self.motion_publisher = self.create_publisher(ManualCommand, args.prefix + '/manual/safe_command', 1)
                self.request_publisher = self.create_publisher(Float64, args.prefix + '/target_angle', 1)
                self.safe_publisher = self.create_publisher(Float64, args.prefix + '/safe_target_angle', 1)
                self.subscription = self.create_subscription(
                    ControlTelemetry, args.prefix + '/control_diagnostics', self.feedback, qos_profile_sensor_data)
                self.timer = self.create_timer(0.05, self.publish_command)

            def feedback(self, message):
                values = {name: getattr(message, name) for name in message.get_fields_and_field_types()}
                panel.feedback(values)
                if log_file:
                    writer.writerow(values)
                    log_file.flush()

            def publish_command(self):
                values = panel.command()
                # Refuse a competing controller/panel in the same namespace.
                if self.count_publishers(args.prefix + '/manual/safe_command') > 1:
                    values = dict(values, action=0)
                message = ManualCommand(**values)
                self.motion_publisher.publish(message)
                target = Float64(data=values['target_angle_rad'])
                self.request_publisher.publish(target)
                self.safe_publisher.publish(target)

        node = Bridge()
    server = create_server(panel, args.web_dir or default_web, port=args.port)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print('Manual panel: http://localhost:' + str(args.port) +
          (' [UI PREVIEW ONLY — no robot feedback]' if args.preview else ' [awaiting dedicated controller]'), flush=True)
    try:
        if args.preview:
            thread.join()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        if node:
            node.destroy_node()
            rclpy.shutdown()
        if log_file:
            log_file.close()


if __name__ == '__main__':
    main()
