"""
DriveBaseAPI for Pybricks MicroPython
Supports SPIKE Prime utilizing EV3 codebase
Built by itonasd
"""

from pybricks.parameters import Color, Axis
from pybricks.tools import StopWatch
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.tools import wait, read_input_byte
from micropython import const
from math import pi

PIVOT_LEFT = const(-1)
PIVOT_RIGHT = const(1)

def clamp(val: float, min_val: float = -1.0, max_val: float = 1.0): return max(min(val, max_val), min_val)
def nearest_hash(s: float, params) -> int: return min(params.keys(), key=lambda t: abs(t - s))

class PIDController:
    __slots__ = (
        "kp", "ki", "kd", "integral_limit", "integral_deadzone", "dt", "integral", "prev_error"
    )

    def __init__(self):
        self.kp = 1
        self.ki = 0
        self.kd = 0.1
        self.integral_limit = 1
        self.integral_deadzone = 10
        self.dt = 0.01
        self.integral = 0.0
        self.prev_error = 0.0

    def setPID(self, params: tuple[float, float, float]) -> None:
        self.kp, self.ki, self.kd = params

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0

    def calculate(self, setpoint: float, measurement: float) -> float:
        error = setpoint - measurement
        if abs(error) <= self.integral_deadzone:
            self.integral += error * self.dt
            self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)

        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

class Task:
    __slots__ = ("series", "index")
    
    def __init__(self, series):
        self.series = series
        self.index = 0

    def update(self) -> bool:
        if self.index >= len(self.series): return True
        step = self.series[self.index]

        if callable(step):
            step()
            self.index += 1
            return False
        
        until = step[0]
        task = step[1] if len(step) > 1 else None
        cleanup = step[2] if len(step) > 2 else None
        if until():
            if callable(cleanup): cleanup()
            self.index += 1
        else:
            if callable(task): task()
        return False

class MissionMotor:
    __slots__ = ("motor")

    def __init__(self, motor: Motor): self.motor = motor
    def move(self, speed: int): return lambda: self.motor.dc(speed)
    def stop(self): return lambda: self.motor.stop()
    def brake(self): return lambda: self.motor.brake()
    def hold(self): return lambda: self.motor.hold()
    def degree(self, target: int): return lambda: abs(self.motor.angle()) >= target
    def resetEncoder(self): return lambda: self.motor.reset_angle(0)

class DriveBaseAPI:
    def __init__(
        self, left_motor: Motor, right_motor: Motor, color_sensor: ColorSensor, hub: PrimeHub,
        forward_params, tagline_params, turn_params, wheel_diameter: float = 62.4, operate_frequency: int = 100
    ):
        self.target_heading = 0
        self.hub = hub
        self.forward_params = forward_params
        self.tagline_params = tagline_params
        self.turn_params = turn_params
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.color_sensor = color_sensor
        self.mm2deg = 360 / (pi * wheel_diameter)
        self.dt = 1 / operate_frequency
        self.throttle = int(1000 / operate_frequency)
        self.imu_integral = 0.0
        self.tag_integral = 0.0
        self.prev_error = 0.0
        self.concurrent_queue: list[Task] = []

    def runConcurrent(self, *series) -> None:
        self.concurrent_queue.append(Task(series))

    def run(self, *series):
        task = Task(series)
        timer = StopWatch()
        while True:
            timer.reset()
            for i in range(len(self.concurrent_queue)-1, -1, -1):
                if self.concurrent_queue[i].update(): self.concurrent_queue.pop(i)
            if task.update(): break
            elapsed = timer.time()
            if elapsed < self.throttle:
                wait(self.throttle - elapsed)
    
    def resetEnconder(self):
        def callback() -> None:
            self.imu_integral = 0.0
            self.tag_integral = 0.0
            self.prev_error = 0.0
            self.left_motor.reset_angle(0)
            self.right_motor.reset_angle(0)
        return callback

    def resetImu(self):
        def callback() -> None:
            self.target_heading = 0
            self.hub.imu.reset_heading(0)
        return callback

    def brake(self):
        def callback() -> None:
            self.imu_integral = 0.0
            self.tag_integral = 0.0
            self.prev_error = 0.0
            self.left_motor.brake()
            self.right_motor.brake()
        return callback
    
    def hold(self):
        def callback() -> None:
            self.imu_integral = 0.0
            self.tag_integral = 0.0
            self.prev_error = 0.0
            self.left_motor.hold()
            self.right_motor.hold()
        return callback
    
    def stop(self):
        def callback() -> None:
            self.left_motor.stop()
            self.right_motor.stop()
        return callback

    def beep(self):
        return lambda: self.hub.speaker.beep()

    def moveRaw(self, left_speed: int, right_speed: int):
        ls = clamp(left_speed, -100.0, 100.0)
        rs = clamp(right_speed, -100.0, 100.0)
        def callback() -> None:
            self.left_motor.dc(float(ls))
            self.right_motor.dc(float(rs))
        return callback

    def moveImu(self, speed: int, kp: float | None = None, ki: float | None = None, kd: float | None = None, telemetry: bool = False):
        s = clamp(speed, -100.0, 100.0)
        n = 0
        started = False
        tkp, tki, tkd = self.forward_params[nearest_hash(s, self.forward_params)]
        kp = tkp if kp is None else kp
        ki = tki if ki is None else ki
        kd = tkd if kd is None else kd
        def callback() -> None:
            nonlocal started, n
            if not started:
                started = True
                if telemetry: print(f"moveImu_{s}")

            error = self.target_heading - self.hub.imu.heading()
            if abs(error) <= 10:
                self.imu_integral += error * self.dt
                self.imu_integral = clamp(self.imu_integral)
            rotation = (error * kp + self.imu_integral * ki - self.hub.imu.angular_velocity(Axis.Z) * kd) #type: ignore
            n += 10
            self.left_motor.dc(float(s + rotation))
            self.right_motor.dc(float(s - rotation))
            if telemetry:
                print(f"  t: {n}, sp: {self.target_heading}, imu: {self.hub.imu.heading()}, p: {error}, i: {self.imu_integral}, d: {self.hub.imu.angular_velocity(Axis.Z)}") #type: ignore
        return callback

    def tagline(self, speed: int, reflection: int, pivot: int = PIVOT_RIGHT, kp: float | None = None, ki: float | None = None, kd: float | None = None, telemetry: bool | None = None):
        s = clamp(speed, -100.0, 100.0)
        n = 0
        started = False
        tkp, tki, tkd = self.tagline_params[nearest_hash(speed, self.tagline_params)]
        kp = tkp if kp is None else kp
        ki = tki if ki is None else ki
        kd = tkd if kd is None else kd
        def callback() -> None:
            nonlocal started, n
            if not started:
                started = True
                if telemetry: print(f"tagline: speed {s}")

            error = reflection - self.color_sensor.reflection()
            if abs(error) <= 10:
                self.tag_integral += error * self.dt
                self.tag_integral = clamp(self.tag_integral)

            derivative = (error - self.prev_error) / self.dt
            rotation = (kp * error) + (ki * self.tag_integral) + (kd * derivative)
            self.prev_error = error
            n += 10
            self.left_motor.dc(float((s + rotation) * pivot))
            self.right_motor.dc(float((s - rotation) * pivot))
            if telemetry:
                print(f"  t: {n}, sp: {reflection}, imu: {self.hub.imu.heading()}, p: {error}, i: {self.tag_integral}, d: {derivative}") #type: ignore
        return callback

    def turnImu(self, pivot: int = 0, deadzone: float = 15.0, kp: float | None = None, ki: float | None = None, kd: float | None = None, telemetry: bool = False):
        power = [0 if pivot == PIVOT_LEFT else 1, 0 if pivot == PIVOT_RIGHT else 1]
        started = False
        n = 0
        integral = 0.0

        def compensate(x: float) -> float:
            if x == 0: return 0.0
            sign = 1 if x > 0 else -1
            return sign * (deadzone + abs(x) * (100 - deadzone) / 100)

        def callback() -> None:
            nonlocal integral, started, kp, ki, kd, n
            if not started:
                started = True
                turn_angle = abs(self.target_heading - self.hub.imu.heading())
                tkp, tki, tkd = self.turn_params[nearest_hash(turn_angle, self.turn_params)]
                kp = tkp if kp is None else kp
                ki = tki if ki is None else ki
                kd = tkd if kd is None else kd
                if telemetry: print(f"turnImu: angle {turn_angle}")

            error = self.target_heading - self.hub.imu.heading()
            if abs(error) <= 10:
                integral += error * self.dt
                integral = clamp(integral)
            rotation = compensate(error * kp + integral * ki - self.hub.imu.angular_velocity(Axis.Z) * kd) #type: ignore
            n += 10
            self.left_motor.dc(float(rotation * power[0]))
            self.right_motor.dc(float(-rotation * power[1]))
            if telemetry:
                print(f"  t: {n}, sp: {self.target_heading}, imu: {self.hub.imu.heading()}, p: {error}, i: {integral}, d: {self.hub.imu.angular_velocity(Axis.Z)}") #type: ignore
        return callback
    
    def degree(self, target: int | float):
        return lambda: (abs(self.left_motor.angle()) + abs(self.right_motor.angle())) / 2 >= target

    def mm(self, target: int):
        return self.degree(self.mm2deg * target)

    def angle(self, target: int, tolerance: float = 1.0, stable: int = 5):
        n = 0
        started = False
        def callback() -> bool:
            nonlocal n, started
            if not started:
                self.target_heading = target
                started = True

            if abs(self.target_heading - self.hub.imu.heading()) <= tolerance: n += 1
            else: n = 0
            return n >= stable
        return callback
    
    def blackReflection(self, threshold: int):
        return lambda: self.color_sensor.reflection() <= threshold
    
    def whiteReflection(self, threshold: int):
        return lambda: self.color_sensor.reflection() >= threshold
    
    def colorReflection(self, color: Color):
        return lambda: self.color_sensor.color() == color

    def untilButton(self, button):
        return lambda: button in self.hub.buttons.pressed()
    
    @staticmethod
    def time(target: int):
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
    def untilstdin(key: str):
        return lambda: read_input_byte(True, True) == key

    @staticmethod
    def clsstdio():
        return lambda: print("\033[2J\033[H", end="")
