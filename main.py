"""
Program's Entry Point for WRO2026 Senior (BETA)
"""

from pybricks.parameters import Port, Color, Direction
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.hubs import PrimeHub

from huskylens import Huskylens, Block, ALGORITHM_COLOR_RECOGNITION
from beta_drivebase import DriveBaseExtended, MissionMotor, PIVOT_LEFT, PIVOT_RIGHT

prime_hub = PrimeHub()
#husky = Huskylens(Port.E)
#m1 = MissionMotor(Motor(Port.C))
#m2 = MissionMotor(Motor(Port.D))
w = DriveBaseExtended(
    Motor(Port.A, Direction.CLOCKWISE), 
    Motor(Port.B, Direction.COUNTERCLOCKWISE), 
    ColorSensor(Port.F), 
    hub = prime_hub,
    straight_params = {
        30:  (0.0, 0.0, 0.0), -30:  (0.0, 0.0, 0.0),
        50:  (0.0, 0.0, 0.0), -50:  (0.0, 0.0, 0.0),
        75:  (0.0, 0.0, 0.0), -75:  (0.0, 0.0, 0.0),
        100: (0.0, 0.0, 0.0), -100: (0.0, 0.0, 0.0),
    },
    tagline_params = {
        30:  (0.0, 0.0, 0.0), -30:  (0.0, 0.0, 0.0),
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

def getMosaicData(tiles: list[Block], ratio_tolerance: int, area_tolerance: int) -> list[list[int]] | None:
    filtered = [tile for tile in tiles if abs(tile.ratio() - ratio_tolerance) <= 1.0 and tile.area() <= area_tolerance]
    if len(filtered) < 12: return None
    sort_by_row = sorted(filtered, key = lambda tile: tile.y)
    return [[v.id for v in sorted(sort_by_row[i:i+4], key = lambda tile: tile.x)] for i in range(0, 12, 4)]


def main():
    return 0

main()
