"""Human command ownership and watchdog. No ROS or browser dependency.

All command angles are radians. A stop requests controlled reference braking,
not an actual hardware emergency stop. Timeouts use monotonic WALL time.
"""
import math
import threading
import time
import uuid


class ControlPanel:
    MODES = {'keyboard': 1, 'angle': 2, 'ai': 3}

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.session = None
        self.last_heartbeat = -math.inf
        self.last_feedback = -math.inf
        self.telemetry = None
        self.mode = 'keyboard'
        self.keys = set()
        self.sequence = 0
        self.motion = 0
        self.target = 0.0
        self.reason = '等待专用 Demo 控制器反馈'

    def _fresh(self):
        return (self.telemetry is not None and
                self.clock() - self.last_feedback < 0.5 and
                self.telemetry.get('status', 'ready') != 'fault')

    def _stop(self, reason):
        self.keys.clear()
        self.motion = 0
        if self.telemetry:
            self.target = self.telemetry['reference_angle_rad']
        self.sequence += 1
        self.reason = reason

    def feedback(self, telemetry):
        with self.lock:
            fields = ('sim_time_s', 'actual_angle_rad', 'reference_angle_rad', 'velocity_rad_s')
            if not all(isinstance(telemetry.get(k), (int, float)) and
                       math.isfinite(telemetry[k]) for k in fields):
                self.last_feedback = -math.inf
                self._stop('反馈无效，请暂停仿真并检查')
                return
            if self.telemetry:
                old_time = self.telemetry['sim_time_s']
                if telemetry['sim_time_s'] == old_time:
                    return  # Repeated timestamps cannot keep a paused model "fresh".
                if telemetry['sim_time_s'] < old_time:
                    self._stop('检测到仿真复位：旧动作已清除')
                    self.target = telemetry['reference_angle_rad']
            else:
                self.target = telemetry['reference_angle_rad']
            self.telemetry = dict(telemetry)
            self.last_feedback = self.clock()

    def _watchdog(self):
        if self.motion and not self._fresh():
            self._stop('反馈中断或仿真暂停：动作已取消')
        if self.session and self.clock() - self.last_heartbeat >= 0.5:
            self._stop('界面心跳中断：动作已取消')
            self.session = None

    def _move(self, radians):
        if not math.isfinite(radians) or abs(radians) > math.radians(45):
            raise ValueError('目标必须在 −45° 到 +45°之间')
        self.target = radians
        self.motion = 1
        self.sequence += 1
        self.reason = '已接受人工指令；以实际反馈判断结果'

    def handle(self, payload):
        with self.lock:
            self._watchdog()
            action = payload.get('action')
            if action == 'claim':
                if self.session:
                    raise ValueError('另一个页面正在控制；关闭后等待 0.5 秒再连接')
                self.session = uuid.uuid4().hex
                self.last_heartbeat = self.clock()
                self._stop('已连接，等待人工操作')
                return {'session': self.session}
            if not self.session or payload.get('session') != self.session:
                raise ValueError('没有控制权，请重新连接')
            self.last_heartbeat = self.clock()
            if action == 'heartbeat':
                return {'ok': True}
            if action in ('stop', 'release'):
                self._stop('人工停止：参考减速后保持')
                if action == 'release':
                    self.session = None
                return {'ok': True}
            if action == 'mode':
                mode = payload.get('mode')
                if mode not in self.MODES:
                    raise ValueError('未知控制模式')
                self._stop('切换模式：旧动作已取消')
                self.mode = mode
                if mode == 'ai':
                    self.reason = 'AI 任务后续开放，本版不会执行任务'
                return {'ok': True}
            if self.mode == 'ai':
                raise ValueError('AI 任务尚未实现')
            if not self._fresh():
                raise ValueError('没有新鲜反馈，禁止启动动作')
            if action == 'angle' and self.mode == 'angle':
                value = payload.get('angle_deg')
                if isinstance(value, bool) or value is None or value == '':
                    raise ValueError('请输入有限的角度数值')
                try:
                    radians = math.radians(float(value))
                except (ValueError, TypeError, OverflowError) as error:
                    raise ValueError('请输入有限的角度数值') from error
                self._move(radians)
            elif action == 'keys' and self.mode == 'keyboard':
                values = payload.get('keys', [])
                if not isinstance(values, list) or any(k not in ('ArrowUp', 'ArrowDown') for k in values):
                    raise ValueError('无效的方向键')
                keys = set(values)
                if keys != self.keys:
                    direction = int('ArrowUp' in keys) - int('ArrowDown' in keys)
                    if direction:
                        self._move(math.radians(45 * direction))
                    else:
                        self._stop('松键或相反方向同时按下：减速保持')
                    self.keys = keys
            elif action == 'nudge' and self.mode == 'keyboard':
                if self.keys:
                    raise ValueError('微调前先松开上下键')
                direction = payload.get('direction')
                if type(direction) is not int or direction not in (-1, 1):
                    raise ValueError('无效微调方向')
                base = self.target if self.motion else self.telemetry['reference_angle_rad']
                self._move(base + math.radians(0.25 * direction))
            else:
                raise ValueError('当前模式不接受此操作')
            return {'ok': True}

    def command(self):
        with self.lock:
            self._watchdog()
            return {'sequence': self.sequence, 'mode': self.MODES[self.mode],
                    'action': self.motion, 'target_angle_rad': self.target}

    def state(self):
        with self.lock:
            self._watchdog()
            return {'app': 'joint-mini-manual', 'protocol_version': 1,
                    'mode': self.mode, 'connected': self._fresh(),
                    'has_owner': self.session is not None,
                    'requested_angle_deg': math.degrees(self.target),
                    'reason': self.reason, 'telemetry': self.telemetry,
                    'speed_limit_deg_s': 3, 'acceleration_limit_deg_s2': 5,
                    'command_step_deg': 0.25}
