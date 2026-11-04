"""One human owner, one shared reference, two synchronized feedback channels.

No fake samples and no ROS dependency. All physics values are SI units.
ControlPanel is reused rather than changing the historical manual Demo.
"""
import math
import time

from manual_panel_core import ControlPanel


class StudyPanel:
    def __init__(self, clock=time.monotonic, require_action_ids=False):
        self.control = ControlPanel(clock)
        # ROS command ordering is separate from each browser's action_id.
        # CLOCK_MONOTONIC continues across process restarts in this WSL boot;
        # a restarted server must not emit small counters below the driver's
        # last consumed command. A WSL reboot also ends the old driver process.
        self.control.sequence = time.monotonic_ns()
        self.pair = None
        self.payload_index = 2
        self.reset_sequence = 0
        self.reset_pending = False
        self.experiment_active = False
        self.require_action_ids = require_action_ids
        self.highest_action_id = 0

    def feedback(self, values):
        with self.control.lock:
            previous_time = self.pair['baseline']['sim_time_s'] if self.pair else None
            previous_ack = self.pair.get('reset_sequence', 0) if self.pair else 0
            try:
                sides = [values['baseline'], values['candidate']]
                for side in sides:
                    for name in ('sim_time_s', 'actual_angle_rad', 'reference_angle_rad',
                                 'requested_angle_rad', 'reference_velocity_rad_s',
                                 'velocity_rad_s', 'commanded_torque_nm'):
                        value = side[name]
                        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                            raise ValueError('non-finite feedback')
                    if side['sim_time_s'] < 0 or side.get('status', '').startswith('fault'):
                        raise ValueError('controller fault')
                for name in ('sim_time_s', 'reference_angle_rad', 'reference_velocity_rad_s',
                             'requested_angle_rad'):
                    if abs(sides[0][name] - sides[1][name]) > 1e-9:
                        raise ValueError('unsynchronized pair')
                for name in ('payload_mass_kg', 'spring_compression_m',
                             'compensation_torque_nm', 'loss_torque_nm'):
                    if not math.isfinite(values[name]):
                        raise ValueError('invalid physics record')
                if values.get('model_status', '').startswith('fault'):
                    raise ValueError('physics fault')
            except (KeyError, TypeError, ValueError):
                self.reset_pending = False
                self.reset_sequence = previous_ack
                self.pair = None
                self.control.feedback({})  # Invalidates freshness and cancels motion.
                self.control.reason = '两侧反馈缺失、故障或不同步；请暂停 Webots 并检查'
                return
            if self.pair and sides[0]['sim_time_s'] == self.pair['baseline']['sim_time_s']:
                return  # Repeated messages cannot revive a paused simulation.
            self.pair = dict(values, baseline=dict(sides[0]), candidate=dict(sides[1]))
            self.control.feedback(sides[0])
            ack = values.get('reset_sequence', 0)
            if type(ack) is not int or ack < 0:
                self.reset_pending = False
                self.reset_sequence = previous_ack
                self.control.feedback({})
                self.pair = None
                return
            if previous_time is not None and sides[0]['sim_time_s'] < previous_time:
                # A world reload invalidates unacknowledged actions, including
                # a payload reset. Do not replay its nonce into the new world.
                self.reset_pending = False
            if self.reset_pending and ack == self.reset_sequence:
                self.reset_pending = False
                if self.control.session:
                    self.control.handle({'action': 'stop', 'session': self.control.session})
                self.control.target = sides[0]['reference_angle_rad']
                self.control.reason = '两侧已重置；等待新的明确操作，不自动续跑'
            elif not self.reset_pending:
                self.reset_sequence = ack
                masses = (0.0, .125, .25)
                self.payload_index = min(range(3), key=lambda i: abs(masses[i] - values['payload_mass_kg']))

    def handle(self, payload):
        with self.control.lock:
            action = payload.get('action')
            if self.experiment_active and action not in ('heartbeat', 'stop', 'release'):
                raise ValueError('自动验收正在运行，人工入口已停用')
            if action == 'claim':
                result = self.control.handle(payload)
                self.highest_action_id = 0
                return result
            # Authenticate BEFORE consuming any ordering identifier. A late
            # request from an expired/foreign owner cannot affect this owner.
            self.control.handle({'action': 'heartbeat', 'session': payload.get('session')})
            if action != 'heartbeat':
                identity = payload.get('action_id')
                if self.require_action_ids or identity is not None:
                    if type(identity) is not int or not self.highest_action_id < identity <= 9007199254740991:
                        raise ValueError('过期或缺少操作序号；旧动作已丢弃，请重新操作')
                    self.highest_action_id = identity
            if action == 'reset':
                index = payload.get('payload_index')
                if type(index) is not int or index not in (0, 1, 2):
                    raise ValueError('请选择 0、0.125 或 0.25 kg')
                if self.reset_pending or not self.state()['can_reset']:
                    raise ValueError('先停止运动，等待参考减速和两侧反馈后再重置')
                self.control.handle({'action': 'stop', 'session': payload.get('session')})
                self.payload_index = index
                self.reset_sequence += 1
                self.reset_pending = True
                self.control.reason = '重置请求已发送；等待真实控制器确认'
                return {'ok': True}
            if self.reset_pending and action in ('keys', 'angle', 'nudge'):
                raise ValueError('重置尚未确认，不接受运动指令')
            return self.control.handle(payload)

    def command(self):
        with self.control.lock:
            command = self.control.command()
            if self.reset_pending and (not self.control.session or not self.control._fresh()):
                self.reset_pending = False
                self.reset_sequence = self.pair.get('reset_sequence', 0) if self.pair else 0
            return {'control': command, 'payload_index': self.payload_index,
                    'reset_sequence': self.reset_sequence}

    def set_experiment_active(self, active):
        with self.control.lock:
            if active and not self.experiment_active and self.control.session:
                self.control.handle({'action': 'release', 'session': self.control.session})
            self.experiment_active = bool(active)

    def state(self):
        with self.control.lock:
            result = self.control.state()
            connected = result['connected'] and not self.reset_pending and self.pair is not None
            can_reset = (connected and not self.control.motion and
                         not self.experiment_active and bool(self.pair.get('can_reset')) and
                         abs(self.pair['baseline']['reference_velocity_rad_s']) < 1e-7)
            return dict(result, app='joint-mini-study', telemetry=self.pair, connected=connected,
                        payload_index=self.payload_index, reset_pending=self.reset_pending,
                        experiment_active=self.experiment_active,
                        model_notice='几何等效力矩测试夹具；非最终 CAD，非显式接触或实测节能',
                        can_reset=can_reset)
