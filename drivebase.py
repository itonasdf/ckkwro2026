"""
DriveBaseAPI for Pybricks MicroPython
Supports SPIKE Prime utilizing EV3 codebase
Built by itonasd
"""

from pybricks.parameters import Color
from pybricks.tools import StopWatch
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.tools import wait, read_input_byte
from micropython import const

PIVOT_LEFT = const(-1)
PIVOT_RIGHT = const(1)

def clamp(val: float, min_val: float = -1.0, max_val: float = 1.0): return max(min(val, max_val), min_val)
def nearest_hash(s: float, params) -> int: return min(params.keys(), key=lambda t: abs(t - s))
def resolve_pid(params: dict, key, kp, ki, kd):
    tkp, tki, tkd = params[nearest_hash(key, params)]
    return (tkp if kp < 0 else kp,
            tki if ki < 0 else ki,
            tkd if kd < 0 else kd)

class PIDController:
    __slots__ = (
        "kp", "ki", "kd", "dt", "error", "integral_limit", 
        "integral_deadzone", "integral", "prev_error", "derivative"
    )

    def __init__(self):
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.dt = 0.0
        self.error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.prev_error = 0.0
        self.integral_limit = 1
        self.integral_deadzone = 10

    def setPID(self, params: tuple[float, float, float]) -> None:
        self.kp, self.ki, self.kd = params

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0

    def calculate(self, setpoint: float, measurement: float) -> float:
        self.error = setpoint - measurement
        if abs(self.error) <= self.integral_deadzone:
            self.integral += self.error * self.dt
            self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)

        self.derivative = (self.error - self.prev_error) / self.dt
        self.prev_error = self.error

        return (self.kp * self.error) + (self.ki * self.integral) + (self.kd * self.derivative)

class Task:
    __slots__ = ("series", "index")
    
    def __init__(self, series):
        self.series = series
        self.index = 0

    def update(self) -> bool:
        step = self.series[self.index]
        if callable(step):
            step()
            self.index += 1
            return self.index >= len(self.series)
        
        until = step[0]
        task = step[1] if len(step) > 1 else None
        cleanup = step[2] if len(step) > 2 else None
        if until():
            if callable(cleanup): cleanup()
            self.index += 1
            return self.index >= len(self.series)
        else:
            if callable(task): task()
        return False

class MissionMotor:
    __slots__ = ("_motor",)

    def __init__(self, motor: Motor): self._motor = motor
    def move(self, speed: int): return lambda: self._motor.dc(speed)
    def coast(self): return lambda: self._motor.stop()
    def brake(self): return lambda: self._motor.brake()
    def hold(self): return lambda: self._motor.hold()
    def stalled(self): return lambda: self._motor.stalled()
    def degree(self, target: int): return lambda: abs(self._motor.angle()) >= target
    def resetEncoder(self): return lambda: self._motor.reset_angle(0)

class DriveBaseAPI:
    def __init__(
        self, left_motor: Motor, right_motor: Motor, color_sensor: ColorSensor, hub: PrimeHub,
        straight_params, tagline_params, turn_params, operate_frequency: int = 100
    ):
        self._target_heading = 0
        self._hub = hub
        self._straight_params = straight_params
        self._tagline_params = tagline_params
        self._turn_params = turn_params
        self._left_motor = left_motor
        self._right_motor = right_motor
        self._color_sensor = color_sensor
        self._dt = 1 / operate_frequency
        self._throttle = int(1000 / operate_frequency)
        self._straight_controller = PIDController()
        self._straight_controller.dt = self._dt
        self._turn_controller = PIDController()
        self._turn_controller.dt = self._dt
        self._tag_controller = PIDController()
        self._tag_controller.dt = self._dt
        self._concurrent_queue: list[Task] = []

    def runConcurrent(self, *series) -> None:
        self._concurrent_queue.append(Task(series))

    def run(self, *series):
        task = Task(series)
        timer = StopWatch()
        while True:
            timer.reset()
            for i in range(len(self._concurrent_queue)-1, -1, -1):
                if self._concurrent_queue[i].update(): self._concurrent_queue.pop(i)
            if task.update(): break
            elapsed = timer.time()
            if elapsed < self._throttle:
                wait(self._throttle - elapsed)
    
    def resetEncoder(self):
        def callback() -> None:
            self._tag_controller.reset()
            self._straight_controller.reset()
            self._left_motor.reset_angle(0)
            self._right_motor.reset_angle(0)
        return callback

    def resetImu(self):
        def callback() -> None:
            self._target_heading = 0
            self._hub.imu.reset_heading(0)
        return callback

    def brake(self):
        def callback() -> None:
            self._tag_controller.reset()
            self._straight_controller.reset()
            self._left_motor.brake()
            self._right_motor.brake()
        return callback
    
    def hold(self):
        def callback() -> None:
            self._tag_controller.reset()
            self._straight_controller.reset()
            self._left_motor.hold()
            self._right_motor.hold()
        return callback
    
    def coast(self):
        def callback() -> None:
            self._left_motor.stop()
            self._right_motor.stop()
        return callback

    def beep(self):
        return lambda: self._hub.speaker.beep()

    def moveTank(self, left_speed: int, right_speed: int):
        ls = clamp(left_speed, -100.0, 100.0)
        rs = clamp(right_speed, -100.0, 100.0)
        def callback() -> None:
            self._left_motor.dc(float(ls))
            self._right_motor.dc(float(rs))
        return callback

    def straight(
        self, speed: int, telemetry: bool = False,
        kp: float = -1.0, ki: float = -1.0, kd: float = -1.0
    ):
        n = 0
        s = clamp(speed, -100.0, 100.0)
        started = False
        kp, ki, kd = resolve_pid(self._straight_params, s, kp, ki, kd)
        def callback() -> None:
            nonlocal started, n
            if not started:
                started = True
                self._straight_controller.setPID((kp, ki, kd))
                if telemetry: print(f"moveImu_{s}")

            n += self._throttle
            rotation = self._straight_controller.calculate(self._target_heading, self._hub.imu.heading())
            self._left_motor.dc(float(s + rotation))
            self._right_motor.dc(float(s - rotation))
            if telemetry:
                print(f"  t: {n}, sp: {self._target_heading}, imu: {self._hub.imu.heading()}, p: {self._straight_controller.error}, i: {self._straight_controller.integral}, d: {self._straight_controller.derivative}")
        return callback

    def tagline(
        self, speed: int, reflection: int, pivot: int = PIVOT_RIGHT, telemetry: bool = False,
        kp: float = -1.0, ki: float = -1.0, kd: float = -1.0
    ):
        n = 0
        s = clamp(speed, -100.0, 100.0)
        started = False
        kp, ki, kd = resolve_pid(self._tagline_params, s, kp, ki, kd)
        def callback() -> None:
            nonlocal started, n
            if not started:
                started = True
                self._tag_controller.setPID((kp, ki, kd))
                if telemetry: print(f"tagline: speed {s}")

            n += self._throttle
            rotation = self._tag_controller.calculate(reflection, self._color_sensor.reflection())
            self._left_motor.dc(float(s + rotation) * pivot)
            self._right_motor.dc(float(s - rotation) * pivot)
            if telemetry:
                print(f"  t: {n}, sp: {reflection}, sen: {self._color_sensor.reflection()}, p: {self._tag_controller.error}, i: {self._tag_controller.integral}, d: {self._tag_controller.derivative}")
        return callback

    def turn(
        self, pivot: int = 0, deadzone: float = 15.0, telemetry: bool = False,
        kp: float = -1.0, ki: float = -1.0, kd: float = -1.0
    ):
        power = [0 if pivot == PIVOT_LEFT else 1, 0 if pivot == PIVOT_RIGHT else 1]
        started = False
        n = 0

        def compensate(x: float) -> float:
            if x == 0: return 0.0
            sign = 1 if x > 0 else -1
            return sign * (deadzone + abs(x) * (100 - deadzone) / 100)

        def callback() -> None:
            nonlocal started, kp, ki, kd, n
            if not started:
                started = True
                turn_angle = abs(self._target_heading - self._hub.imu.heading())
                kp, ki, kd = resolve_pid(self._turn_params, turn_angle, kp, ki, kd)
                self._turn_controller.reset()
                self._turn_controller.setPID((kp, ki, kd))
                if telemetry: print(f"turnImu: angle {turn_angle}")

            n += self._throttle
            rotation = compensate(self._turn_controller.calculate(self._target_heading, self._hub.imu.heading()))
            self._left_motor.dc(float(rotation * power[0]))
            self._right_motor.dc(float(-rotation * power[1]))
            if telemetry:
                print(f"  t: {n}, sp: {self._target_heading}, imu: {self._hub.imu.heading()}, p: {self._turn_controller.error}, i: {self._turn_controller.integral}, d: {self._turn_controller.derivative}")
        return callback
    
    def degree(self, target: int | float):
        return lambda: (abs(self._left_motor.angle()) + abs(self._right_motor.angle())) / 2 >= target

    def heading(self, target: int, tolerance: float = 1.0, stable: int = 5):
        n = 0
        started = False
        def callback() -> bool:
            nonlocal n, started
            if not started:
                self._target_heading = target
                started = True

            if abs(self._target_heading - self._hub.imu.heading()) <= tolerance: n += 1
            else: n = 0
            return n >= stable
        return callback
    
    def blackReflection(self, threshold: int):
        return lambda: self._color_sensor.reflection() <= threshold
    
    def whiteReflection(self, threshold: int):
        return lambda: self._color_sensor.reflection() >= threshold
    
    def colorReflection(self, color: Color):
        return lambda: self._color_sensor.color() == color

    def untilButton(self, button):
        return lambda: button in self._hub.buttons.pressed()
    
    @staticmethod
    def ms(target: int):
        timer = StopWatch()
        started = False
        def callback() -> bool:
            nonlocal started
            if not started:
                timer.reset()
                started = True
            return timer.time() >= target
        return callback
    
    @staticmethod
    def forever():
        return lambda: False

    @staticmethod
    def any(*conditions):
        return lambda: any(c() for c in conditions)

    @staticmethod
    def all(*conditions):
        return lambda: all(c() for c in conditions)
    
    @staticmethod
    def untilStdin(key: str):
        return lambda: read_input_byte(True, True) == key

    @staticmethod
    def clearStdio():
        return lambda: print("\033[2J\033[H", end="")
