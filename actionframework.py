"""
RobotActionFramework for Pybricks MicroPython
Supports SPIKE Prime utilizing EV3 codebase
Built by itonasd
"""

from pybricks.parameters import Color
from pybricks.tools import StopWatch
from typing import Callable
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor

CENTER = 0
LEFT = 1 
RIGHT = 2


def clamp(val, min_val=-1.0, max_val=1.0): return max(min(val, max_val), min_val)

class PIDController:
    def __init__(self):
        self.kp = 1
        self.ki = 0
        self.kd = 0.1
        self.integral_limit = 1
        self.integral = 0.0
        self.prev_error = 0.0

    def setPID(self, params: list[int]):
        self.kp = params[0]
        self.ki = params[1]
        self.kd = params[2]

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def calculate(self, setpoint: int, measurement: int) -> int:
        error = setpoint - measurement
        self.integral += error * 0.1
        self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)

        derivative = (error - self.prev_error) / 0.1
        self.prev_error = error

        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

class ActionFramework:
    def __init__(
        self, left_motor: Motor, right_motor: Motor, color_sensor: ColorSensor, hub: PrimeHub,
        forward_params: dict[int, list[int]], linetrace_params: dict[int, list[int]], turn_params: dict[int, list[int]]
    ):
        self.target_heading = 0
        self.controller = PIDController()
        self.hub = hub
        self.forward_params = forward_params
        self.linetrace_params = linetrace_params
        self.turn_params = turn_params
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.color_sensor = color_sensor
        self.concurrent_queue = []

    def nearest_tier(self, s: int, params: dict) -> int:
        return min(params.keys(), key=lambda t: abs(t - s))

    def reset_encoder(self) -> None:
        self.controller.reset()
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)

    def reset_imu(self) -> None:
        self.target_heading = 0
        self.hub.imu.reset_heading(0)  

    def run_async(self, condition: Callable[[], bool], action: Callable[[], None] | None = None, destroying: Callable[[], None] | None = None) -> None:
        self.concurrent_queue.append([condition, action, destroying])

    def run(self, condition: Callable[[], bool], action: Callable[[], None], destroying: Callable[[], None] | None = None) -> None:
        while not condition():
            action()
            for i in range(len(self.concurrent_queue) - 1, -1, -1):
                if not self.concurrent_queue[i][0]():
                    if callable(self.concurrent_queue[i][1]): self.concurrent_queue[i][1]()
                else:
                    if callable(self.concurrent_queue[i][2]): self.concurrent_queue[i][2]()
                    self.concurrent_queue.pop(i)

        if callable(destroying):
            destroying()
        else:
            self.left_motor.stop()
            self.right_motor.stop()

    def brake(self) -> Callable[[], None]:
        def callback() -> None:
            self.left_motor.brake()
            self.right_motor.brake()
        return callback
    
    def hold(self) -> Callable[[], None]:
        def callback() -> None:
            self.left_motor.hold()
            self.right_motor.hold()
        return callback
    
    def stop(self) -> Callable[[], None]:
        def callback() -> None:
            self.left_motor.stop()
            self.right_motor.stop()
        return callback

    def accelerate(self, speed: int, ramp_degree: int) -> Callable[[], int]:
        started = False
        start_degree = 0

        def callback() -> int:
            nonlocal started, start_degree
            current = (abs(self.left_motor.angle()) + abs(self.right_motor.angle())) / 2
            if not started:
                start_degree = current
                started = True
            t = min((current - start_degree) / max(ramp_degree, 1), 1.0)
            return int(speed * t)
        return callback
    
    def decelerate(self, speed: int, total_degree: int, ramp_degree: int) -> Callable[[], int]:
        started = False
        start_degree = 0

        def callback() -> int:
            nonlocal started, start_degree
            current = (abs(self.left_motor.angle()) + abs(self.right_motor.angle())) / 2
            if not started:
                start_degree = current
                started = True
            t = max(0.0, min((total_degree - (current - start_degree)) / max(ramp_degree, 1), 1.0))
            return int(speed * t)
        return callback

    def move_tank(self, left_speed: Callable[[], int] | int, right_speed: Callable[[], int] | int) -> Callable[[], None]:
        def callback() -> None:
            ls = left_speed() if callable(left_speed) else left_speed
            rs = right_speed() if callable(right_speed) else right_speed
            self.left_motor.dc(int(clamp(ls, -100, 100)))
            self.right_motor.dc(int(clamp(rs, -100, 100)))
        return callback

    def angle_correction(self, speed: Callable[[], int] | int) -> Callable[[], None]:
        def callback() -> None:
            s = speed() if callable(speed) else speed
            self.controller.setPID(self.forward_params[self.nearest_tier(s, self.forward_params)])

            rotation = self.controller.calculate(self.target_heading, self.hub.imu.heading())
            self.left_motor.dc(int(clamp(s + rotation, -100, 100)))
            self.right_motor.dc(int(clamp(s - rotation, -100, 100)))
        return callback

    def linetrace(self, speed: Callable[[], int] | int, fixedReflection: int) -> Callable[[], None]:
        def callback() -> None:
            s = speed() if callable(speed) else speed
            self.controller.setPID(self.linetrace_params[self.nearest_tier(s, self.linetrace_params)])

            rotation = self.controller.calculate(self.color_sensor.reflection(), fixedReflection)
            self.left_motor.dc(int(clamp(s + rotation, -100, 100)))
            self.right_motor.dc(int(clamp(s - rotation, -100, 100)))
        return callback

    def turn(self, pivot: int = CENTER) -> Callable[[], None]:
        power = [0 if pivot == LEFT else 1, 0 if pivot == RIGHT else 1]
        started = False
        def callback() -> None:
            nonlocal started
            if not started:
                turn_angle = abs(self.target_heading - self.hub.imu.heading()) - 10
                self.controller.reset()
                selected = [1, 0.2, 0.1]
                for i in sorted(self.turn_params):
                    selected = self.turn_params[i]
                    if turn_angle <= i: break
                self.controller.setPID(selected)
                started = True
                
            rotation = self.controller.calculate(self.target_heading, self.hub.imu.heading())
            self.left_motor.dc(int(clamp(rotation, -100, 100) * power[0]))
            self.right_motor.dc(int(clamp(-rotation, -100, 100) * power[1]))
        return callback
    
    def degree(self, target: int) -> Callable[[], bool]:
        return lambda: (abs(self.left_motor.angle()) + abs(self.right_motor.angle())) / 2 >= target
    
    def angle(self, target: int, tolerance: float = 1.0, stable: int = 10) -> Callable[[], bool]:
        started = False
        n = 0
        def callback() -> bool:
            nonlocal n, started
            if not started:
                self.target_heading = target
                started = True

            if abs(self.target_heading - self.hub.imu.heading()) <= tolerance: n += 1
            else: n = 0
            return n >= stable
        return callback
    
    def timer(self, target: int) -> Callable[[], bool]:
        timer = StopWatch()
        started = False
        
        def callback() -> bool:
            nonlocal started
            if not started:
                timer.reset()
                started = True
            return timer.time() >= target
            
        return callback
    
    def black_reflection(self, threshold: int) -> Callable[[], bool]:
        return lambda: self.color_sensor.reflection() <= threshold
    
    def white_reflection(self, threshold: int) -> Callable[[], bool]:
        return lambda: self.color_sensor.reflection() >= threshold
    
    def color_reflection(self, color: Color) -> Callable[[], bool]:
        return lambda: self.color_sensor.color() == color

    def any(self, *conditions: Callable[[], bool]) -> Callable[[], bool]:
        return lambda: any(c() for c in conditions)

    def all(self, *conditions: Callable[[], bool]) -> Callable[[], bool]:
        return lambda: all(c() for c in conditions)
