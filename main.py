"""
Program's Entry Point for WRO2026 Senior
"""

from pybricks.parameters import Port, Color, Direction
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.hubs import PrimeHub
from pybricks.tools import wait

from huskylens import Huskylens, Block, ALGORITHM_COLOR_RECOGNITION
from drivebase import DriveBaseAPI, MissionMotor, PIVOT_LEFT, PIVOT_RIGHT

WHITECLR = Color(210, 30, 80)
YELLOWCLR = Color(56, 60, 65)
BLUECLR = Color(230, 85, 20)
GREENCLR = Color(190, 40, 15)

husky = Huskylens(Port.E)
mf = MissionMotor(Motor(Port.C, Direction.CLOCKWISE))
mb = MissionMotor(Motor(Port.A, Direction.COUNTERCLOCKWISE))
w = DriveBaseAPI(
    Motor(Port.B, Direction.COUNTERCLOCKWISE), 
    Motor(Port.D, Direction.CLOCKWISE), 
    ColorSensor(Port.F),
    hub = PrimeHub(),
    straight_params = {
        60:  (3.2, 6.0, 0.12),  -60:  (3.2, 6.0, 0.12),
        80:  (3.6, 10.0, 0.12), -80:  (3.6, 10.0, 0.12),
        100: (6.2, 12.0, 0.24), -100: (6.2, 12.0, 0.24),
    },
    tagline_params = {
        60:  (1.25, 0.0, 0.06),
        80:  (1.25, 0.0, 0.085),
        100:  (1.25, 0.0, 0.1),
    },
    turn_params = {
        10:  (6.0, 0.0, 0.15),
        90:  (4.2, 0.0, 0.12),
        120: (4.4, 0.0, 0.12)
    },
    pturn_params = {
        15:  (8.0, 0.0, 0.15),
        24:  (8.25, 0.0, 0.15),
        90:  (10.0, 0.0, 0.22),
    },
    color_params = [
        YELLOWCLR, GREENCLR, BLUECLR, WHITECLR, Color(230, 40, 15)
    ]
)

print(f"charging current: {expr.w._hub.charger.current()}")
print(f"battery voltage:  {expr.w._hub.battery.voltage()}")
