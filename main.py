"""
Program's Entry Point for WRO2026 Senior
"""

from pybricks.parameters import Port, Color, Direction
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.hubs import PrimeHub

from huskylens import Huskylens, Block, ALGORITHM_COLOR_RECOGNITION
from drivebase import DriveBaseFramework, MissionMotor, LEFT, RIGHT

prime_hub = PrimeHub()
husky = Huskylens(Port.E)
m1 = MissionMotor(Motor(Port.C))
m2 = MissionMotor(Motor(Port.D))
w = DriveBaseFramework(
    Motor(Port.A, Direction.CLOCKWISE), 
    Motor(Port.B, Direction.COUNTERCLOCKWISE), 
    ColorSensor(Port.F), 
    hub = prime_hub,
    wheel_diameter = 0,
    forward_params = {
        10:  (0.0, 0.0, 0.0), -10:  (0.0, 0.0, 0.0),
        25:  (0.0, 0.0, 0.0), -25:  (0.0, 0.0, 0.0),
        50:  (0.0, 0.0, 0.0), -50:  (0.0, 0.0, 0.0),
        75:  (0.0, 0.0, 0.0), -75:  (0.0, 0.0, 0.0),
        100: (0.0, 0.0, 0.0), -100: (0.0, 0.0, 0.0),
    },
    linetrace_params = {
        10:  (0.0, 0.0, 0.0), -10:  (0.0, 0.0, 0.0),
        25:  (0.0, 0.0, 0.0), -25:  (0.0, 0.0, 0.0),
        50:  (0.0, 0.0, 0.0), -50:  (0.0, 0.0, 0.0),
        75:  (0.0, 0.0, 0.0), -75:  (0.0, 0.0, 0.0),
        100: (0.0, 0.0, 0.0), -100: (0.0, 0.0, 0.0),
    },
    turn_params = {
        30:  (0.0, 0.0, 0.0),
        90:  (0.0, 0.0, 0.0),
        180: (0.0, 0.0, 0.0),
    },
)

def solve_mosaic(tiles: list[Block]) -> list[list[int]]:
    filtered = [tile for tile in tiles if abs(tile.ratio - 1.0) <= 0.25 and tile.area <= 2000]
    sort_by_row = sorted(filtered, key = lambda tile: tile.y)
    length = len(filtered)

    #return [[v.id for v in sorted(sort_by_row[i:i+4], key = lambda tile: tile.x)] for i in range(0, 12, 4)]

    result = []
    for i in range(3):
        temp = []
        for j in range(4):
            idx = i * 4 + j
            if idx < length:
                tile = sort_by_row[idx]
                temp.append((tile.id, tile.x))
            else: temp.append((1, 9999))
        result.append([t[0] for t in sorted(temp, key = lambda t: t[1])])

    return result



def main():
    # write your program here

    return 0



if __name__ == "__main__": main()
