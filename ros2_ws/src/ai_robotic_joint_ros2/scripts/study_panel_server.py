#!/usr/bin/env python3
"""Local-only paired study panel. Preview never fabricates robot feedback."""
import argparse
import csv
import pathlib
import threading

from manual_panel_server import create_server
from study_panel_core import StudyPanel


def message_values(message):
    return {name: getattr(message, name) for name in message.get_fields_and_field_types()}


def engineering_status(panel):
    """Live status is not an attestation of mechanical or motion acceptance."""
    state = panel.state()
    return {
        'scope': 'R0 vs F-N unoptimized geometry-equivalent fixture; simulation only',
        'paired_feedback_fresh': state['connected'],
        'source_modules': ['one shared trajectory / two C++ torque PIDs',
                           'keyboard and angle ownership/watchdog',
                           'geometry-derived spring energy and passive torque',
                           'separate motor/passive/loss diagnostics'],
        'software_test_evidence': 'See data/verification/software_baseline_20261104.json; this live endpoint does not run or certify tests.',
        'webots_motion_acceptance': 'pending',
        'native_cad_acceptance': 'pending',
        'five_group_optimization': 'pending',
        'public_trial_acceptance': 'pending',
        'limitations': ['No physical haptic experience or measured energy saving',
                        'No explicit cam/roller contact verification',
                        'STOP means controlled reference braking, not a hardware emergency stop'],
        'project_document_index': ['docs/ENGINEERING_EVIDENCE.md', 'docs/LIMITATIONS.md',
                                   'docs/REPRODUCIBILITY.md',
                                   'research/gravity_compensation/sources.csv'],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--read-only', action='store_true', help='Disable all human ownership while automated tests run')
    parser.add_argument('--record', help='Exclusive-create CSV; never overwrite existing experiments')
    args, ros_args = parser.parse_known_args()
    panel = StudyPanel(require_action_ids=True)
    panel.set_experiment_active(args.read_only)
    node = None
    log_file = None
    if args.preview:
        web = pathlib.Path(__file__).resolve().parents[1] / 'study_web'
    else:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from ament_index_python.packages import get_package_share_directory
        from ai_robotic_joint_ros2.msg import ManualCommand, ControlTelemetry, StudyCommand, StudyTelemetry
        rclpy.init(args=ros_args)
        web = pathlib.Path(get_package_share_directory('ai_robotic_joint_ros2')) / 'study_web'
        writer = None
        if args.record:
            target = pathlib.Path(args.record)
            target.parent.mkdir(parents=True, exist_ok=True)
            log_file = target.open('x', newline='', encoding='utf-8')
            names = [side + '_' + name for side in ('baseline', 'candidate')
                     for name in ControlTelemetry.get_fields_and_field_types()]
            names += [name for name in StudyTelemetry.get_fields_and_field_types()
                      if name not in ('baseline', 'candidate')]
            writer = csv.DictWriter(log_file, fieldnames=names)
            writer.writeheader()
            log_file.flush()

        class Bridge(Node):
            def __init__(self):
                super().__init__('joint_study_panel')
                self.topic = '/joint_study/manual/safe_command'
                # During explicit automated acceptance, the panel is a viewer,
                # not even a STOP publisher. Otherwise a second publisher
                # would trigger the driver's conflict hold and block the test.
                self.publisher = None if args.read_only else self.create_publisher(StudyCommand, self.topic, 1)
                self.subscription = self.create_subscription(
                    StudyTelemetry, '/joint_study/diagnostics', self.feedback, qos_profile_sensor_data)
                self.timer = self.create_timer(.05, self.publish)

            def feedback(self, message):
                values = message_values(message)
                for side in ('baseline', 'candidate'):
                    values[side] = message_values(values[side])
                panel.feedback(values)
                if writer:
                    flat = {side + '_' + key: value for side in ('baseline', 'candidate')
                            for key, value in values[side].items()}
                    flat.update({key: value for key, value in values.items()
                                 if key not in ('baseline', 'candidate')})
                    writer.writerow(flat)
                    log_file.flush()

            def publish(self):
                if self.publisher is None:
                    return
                competition = self.count_publishers(self.topic) > 1
                panel.set_experiment_active(args.read_only or competition)
                values = panel.command()
                self.publisher.publish(StudyCommand(
                    control=ManualCommand(**values['control']),
                    payload_index=values['payload_index'], reset_sequence=values['reset_sequence']))

        node = Bridge()
    server = create_server(panel, web, port=args.port,
                           report_provider=lambda: engineering_status(panel))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print('Joint study: http://localhost:' + str(args.port) +
          (' [PREVIEW: no simulated or fabricated feedback]' if args.preview else ' [awaiting paired controller]'), flush=True)
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


if __name__ == '__main__': main()
