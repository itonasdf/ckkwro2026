"""
Main Entry Point for WRO2026 Senior
"""

from pybricks.parameters import Port, Color, Direction
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.robotics import DriveBase
from pybricks.hubs import PrimeHub
from pybricks.tools import wait

from huskylens import Huskylens, Block, ALGORITHM_COLOR_RECOGNITION
from drivebase import DriveBaseAPI, MissionMotor, PIVOT_LEFT, PIVOT_RIGHT

prime_hub = PrimeHub()
#husky = Huskylens(Port.E)
#m1 = MissionMotor(Motor(Port.C))
#m2 = MissionMotor(Motor(Port.D))
w = DriveBaseAPI(
    Motor(Port.A, Direction.COUNTERCLOCKWISE), 
    Motor(Port.B, Direction.CLOCKWISE), 
    ColorSensor(Port.F), 
    hub = prime_hub,
    wheel_diameter = 62.4,
    operate_frequency = 100,
    forward_params = {
        30:  (3.0, 6.0, 0.045), -30:  (0.0, 0.0, 0.0),
        50:  (2.4, 10.0, 0.045), -50:  (0.0, 0.0, 0.0),
        75:  (2.0, 14.0, 0.045), -75:  (0.0, 0.0, 0.0),
        100: (3.0, 25.0, 0.045), -100: (0.0, 0.0, 0.0),
    },
    linetrace_params = {
        30:  (0.1, 0.0, 0.01), -30:  (0.0, 0.0, 0.0),
        50:  (0.0, 0.0, 0.0), -50:  (0.0, 0.0, 0.0),
        75:  (0.0, 0.0, 0.0), -75:  (0.0, 0.0, 0.0),
        100: (0.0, 0.0, 0.0), -100: (0.0, 0.0, 0.0),
    },
    turn_params = {
        30:  (0.0, 0.0, 0.0),
        90:  (3.0, 0.0, 0.1),
        180: (0.0, 0.0, 0.0),
    },
)


def solve_mosaic(tiles: list[Block], ratio_tolerance: int, area_tolerance: int) -> list[list[int]] | None:
    filtered = [tile for tile in tiles if abs(tile.ratio() - ratio_tolerance) <= 1.0 and tile.area() <= area_tolerance]
    if len(filtered) < 12: return None
    sort_by_row = sorted(filtered, key = lambda tile: tile.y)
    return [[v.id for v in sorted(sort_by_row[i:i+4], key = lambda tile: tile.x)] for i in range(0, 12, 4)]


def main():
    return 0

main()
