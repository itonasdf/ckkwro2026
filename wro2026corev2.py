"""
Program's Entry Point for WRO2026 Senior
"""

from pybricks.parameters import Port, Color, Direction
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.hubs import PrimeHub

from huskylens import Huskylens, Block, ALGORITHM_COLOR_RECOGNITION
from drivebase import DriveBaseAPI, MissionMotor, PIVOT_LEFT, PIVOT_RIGHT

# (0,0)#1, (0,1)#2, (0,2), (0,3)
# (1,0)#5, (1,1)#6, (1,2), (1,3)
# (2,0)#3, (2,1)#4, (2,2), (2,3)

class expr:
    ALGORITHM_COLOR_RECOGNITION = ALGORITHM_COLOR_RECOGNITION
    PIVOT_LEFT = PIVOT_LEFT
    PIVOT_RIGHT = PIVOT_RIGHT
    BRAKE_TIME = 20

    YELLOW = 0
    BLUE = 1
    GREEN = 2
    WHITE = 3
    COLOR = Color

    color_var = [
        [YELLOW, YELLOW, BLUE, BLUE],
        [GREEN, GREEN, WHITE, WHITE],
        [YELLOW, YELLOW, GREEN, GREEN]
    ]

    husky = Huskylens(Port.E)
    mf = MissionMotor(Motor(Port.C, Direction.CLOCKWISE), kp=700000, ki=350000, kd=5000)
    mf._motor.control.limits(speed=1150)
    mb = MissionMotor(Motor(Port.A, Direction.COUNTERCLOCKWISE), kp=10000, ki=5000, kd=300)
    w = DriveBaseAPI(
        Motor(Port.B, Direction.COUNTERCLOCKWISE), 
        Motor(Port.D, Direction.CLOCKWISE), 
        ColorSensor(Port.F),
        hub = PrimeHub(),
        straight_params = {
            60:  (5.0, 5.5, 0.3),  -60:  (5.0, 5.5, 0.3),
            80:  (5.0, 6.0, 0.425), -80:  (5.0, 6.0, 0.425),
            100: (5.0, 7.5, 0.65), -100: (5.0, 7.5, 0.65),
        },
        tagline_params = {
            -75:  (1.4, 0.0, 0.08),
            -50: (1.2, 0.0, 0.065),
            50:  (1.2, 0.0, 0.065),
            75:  (1.4, 0.0, 0.08),
        },
        turn_params = {
            90:  (3.0, 0.0, 0.035),
        },
        pturn_params = {
            90:  (9.0, 0.0, 0.24),
        }
    )

    @staticmethod
    def getMosaicData(tiles: list[Block], ratio_tolerance: int, area_tolerance: int) -> list[list[int]] | None:
        #filtered = [tile for tile in tiles if abs(tile.ratio() - ratio_tolerance) <= 1.0 and tile.area() <= area_tolerance]
        filtered = tiles
        if len(filtered) < 12: return None
        sort_by_row = sorted(filtered, key = lambda tile: tile.y)
        return [[v.id for v in sorted(sort_by_row[i:i+4], key = lambda tile: tile.x)] for i in range(0, 12, 4)]

    @staticmethod
    def mf_set0():
        w = expr.w
        mf = expr.mf

        return (
            [ w.ms(50), mf.move(50) ],
            [ w.ms(100), mf.move(75) ],
            [ w.ms(80), mf.move(100) ],
            [ w.ms(100), mf.move(75) ],
            [ w.ms(50), mf.move(50) ],
            mf.resetEncoder(),
            mf.brake()
        )

    @staticmethod
    def mf_set0d():
        w = expr.w
        mf = expr.mf
        return (
            [ mf.degreeAt(-40), mf.track(-40) ],
            [ w.ms(50), mf.move(50) ],
            [ w.ms(100), mf.move(75) ],
            [ w.ms(80), mf.move(100) ],
            [ w.ms(100), mf.move(75) ],
            [ w.ms(50), mf.move(50) ],
            mf.resetEncoder(),
            mf.brake()
        )

    @staticmethod
    def mf_keep():
        w = expr.w
        mf = expr.mf
        return (
            [ mf.degree(75), mf.move(-50) ],
            [ mf.degree(200), mf.move(-25) ],
            [ mf.degreeAt(-565), mf.track(-565) ],
        )

    @staticmethod
    def mf_keeppick():
        w = expr.w
        mf = expr.mf
        return (
            [ mf.degree(75), mf.move(-50) ],
            [ mf.degree(200), mf.move(-25) ],
            [ mf.degreeAt(-500), mf.track(-500) ],
        )

    @staticmethod
    def mf_mid():
        w = expr.w
        mf = expr.mf
        return (
            [ mf.degreeAt(-150), mf.track(-150) ],
        )

    @staticmethod
    def mf_low():
        w = expr.w
        mf = expr.mf
        return (
            [ mf.degreeAt(-185), mf.track(-185) ],
        )

    @staticmethod
    def mf_pick(tolerance = 1, stable = 5):
        w = expr.w
        mf = expr.mf
        return (
            [ mf.degreeAt(-500, tolerance, stable), mf.track(-500) ],
        )

    @staticmethod
    def mf_releasemax():
        w = expr.w
        mf = expr.mf
        return (
            [ mf.degreeAt(-600), mf.track(-600) ],
        )

    @staticmethod
    def mf_release():
        w = expr.w
        mf = expr.mf
        return (
            [ mf.degreeAt(-565), mf.track(-565) ],
        )

    @staticmethod
    def mb_set0():
        w = expr.w
        mb = expr.mb
        return (
            [ w.ms(50), mb.move(50) ],
            [ w.ms(100), mb.move(75) ],
            [ w.ms(100), mb.move(100) ],
            [ w.ms(100), mb.move(75) ],
            [ w.ms(50), mb.move(50) ],
            mb.resetEncoder(),
            mb.brake()
        )

    @staticmethod
    def mb_set0d():
        w = expr.w
        mb = expr.mb
        return (
            [ mb.degreeAt(-30), mb.track(-30) ],
            [ w.ms(50), mb.move(50) ],
            [ w.ms(100), mb.move(75) ],
            [ w.ms(100), mb.move(100) ],
            [ w.ms(100), mb.move(75) ],
            [ w.ms(50), mb.move(50) ],
            mb.resetEncoder(),
            mb.brake()
        )

    @staticmethod
    def mb_up2mid():
        w = expr.w
        mb = expr.mb
        return (
            [ mb.degreeAt(-280), mb.track(-280) ],
        )

    @staticmethod
    def mb_fixback():
        w = expr.w
        mb = expr.mb
        return (
            [ mb.degreeAt(-75), mb.track(-75) ],
        )

    @staticmethod
    def mb_down():
        w = expr.w
        mb = expr.mb
        return (
            [ mb.degreeAt(-310), mb.track(-310) ],
        )


print(f"charging current: {expr.w._hub.charger.current()}")
print(f"battery voltage:  {expr.w._hub.battery.voltage()}")
