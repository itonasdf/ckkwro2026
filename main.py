"""
Program's Entry Point for WRO2026 Senior
"""

from pybricks.parameters import Port, Color, Direction
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.hubs import PrimeHub

from huskylens import Huskylens, Block, ALGORITHM_COLOR_RECOGNITION
from drivebase import DriveBaseAPI, MissionMotor, PIVOT_LEFT, PIVOT_RIGHT

prime_hub = PrimeHub()
#husky = Huskylens(Port.E)
m1 = MissionMotor(Motor(Port.C))
m2 = MissionMotor(Motor(Port.D))
w = DriveBaseAPI(
    Motor(Port.A, Direction.COUNTERCLOCKWISE), 
    Motor(Port.B, Direction.CLOCKWISE), 
    ColorSensor(Port.F), 
    hub = prime_hub,
    straight_params = {
        30:  (2.0, 2.5, 0.05), -30:  (2.0, 2.5, 0.05),
        50:  (2.4, 6.0, 0.065), -50:  (2.4, 6.0, 0.065),
        75:  (3.0, 10.0, 0.1), -75:  (3.0, 10.0, 0.1),
        100: (4.0, 16.0, 0.2), -100: (4.0, 16.0, 0.2),
    },
    tagline_params = {
        30:  (0.65, 0.0, 0.065), -30:  (0.65, 0.0, 0.065),
        50:  (0.65, 0.0, 0.065), -50:  (0.65, 0.0, 0.065),
        75:  (0.9, 0.0, 0.1), -75:  (0.0, 0.0, 0.0),
        100: (0.0, 0.0, 0.0), -100: (0.0, 0.0, 0.0),
    },
    turn_params = {
        30:  (2.0, 0.0, 0.04),
        90:  (2.0, 0.0, 0.04),
    },
)

# (0,0)#1, (0,1)#2, (0,2), (0,3)
# (1,0)#5, (1,1)#6, (1,2), (1,3)
# (2,0)#3, (2,1)#4, (2,2), (2,3)
def getMosaicData(tiles: list[Block], ratio_tolerance: int, area_tolerance: int) -> list[list[int]] | None:
    filtered = [tile for tile in tiles if abs(tile.ratio() - ratio_tolerance) <= 1.0 and tile.area() <= area_tolerance]
    if len(filtered) < 12: return None
    sort_by_row = sorted(filtered, key = lambda tile: tile.y)
    return [[v.id for v in sorted(sort_by_row[i:i+4], key = lambda tile: tile.x)] for i in range(0, 12, 4)]

def main(args):
    # write your program here
    return

main()
