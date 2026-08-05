"""
Program's Core for WRO2026 Senior
"""

from pybricks.parameters import Port, Color, Direction
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.hubs import PrimeHub
from pybricks.tools import wait

from huskylens import Huskylens, Block
from drivebase import DriveBaseAPI, MissionMotor, PIVOT_LEFT, PIVOT_RIGHT

# (0,0)#1, (0,1)#2, (0,2), (0,3)
# (1,0)#5, (1,1)#6, (1,2), (1,3)
# (2,0)#3, (2,1)#4, (2,2), (2,3)

class expr:
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

    homography_matrix = [
        [ 1.48125806e+00,  2.11608295e-01, -5.50181566e+01],
        [-1.24691686e-02,  1.68333776e+00, -2.48136456e+01],
        [-4.15638954e-05,  7.99071870e-04,  1.00000000e+00]
    ]

    husky = Huskylens(Port.E)
    mf = MissionMotor(Motor(Port.C, Direction.CLOCKWISE), kp=600000, ki=300000, kd=5000)
    mb = MissionMotor(Motor(Port.A, Direction.COUNTERCLOCKWISE), kp=200000, ki=100000, kd=5000)
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
            40:  (1.15, 0.0, 0.0625),
            50:  (1.3, 0.0, 0.0685),
            75:  (1.5, 0.0, 0.0825),
        },
        turn_params = {
            30: (3.5, 0.0, 0.05),
            90:  (3.0, 0.0, 0.04365),
        },
        pturn_params = {
            30:  (8.65, 0.0, 0.2065),
            90:  (10.0, 0.0, 0.25),
        }
    )

    @staticmethod
    def portView():
        while 1:
            print("\033[H\033[2J\033[3J", end="")
            print(f"heading: {expr.w._hub.imu.heading():.2f} \nl_motor: {expr.w._left_motor.angle()} \nr_motor: {expr.w._right_motor.angle()} \nmf_motor: {expr.mf._motor.angle()} \nmb_motor: {expr.mb._motor.angle()} \nsensor: {expr.w._color_sensor.reflection()} \nvoltage: {expr.w._hub.battery.voltage()} \n")
            wait(100)

    @staticmethod
    def getMosaicDataEx2(tiles: list[Block]):
        def transform(x: float, y: float):
            matrix = expr.homography_matrix
            u = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2]
            v = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2]
            w = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2]
            u /= w
            v /= w
            return [ u, v ]
        
        cx_arr = [v.x + v.width / 2 for v in tiles]
        cy_arr = [v.y + v.height / 2 for v in tiles]
        warped_arr = [transform(cx_arr[i], cy_arr[i]) for i in range(len(tiles))]

        min_x = min(x for x, _ in warped_arr)
        max_x = max(x for x, _ in warped_arr)
        min_y = min(y for _, y in warped_arr)
        max_y = max(y for _, y in warped_arr)
        cell_w = (max_x - min_x) / 3
        cell_h = (max_y - min_y) / 2

        for i in range(len(tiles)):
            x, y = warped_arr[i]
            x -= min_x
            y -= min_y

            col = round(x / cell_w)
            row = round(y / cell_h)
            col = max(0, min(col, 3))
            row = max(0, min(row, 2))

            expr.color_var[row][col] = tiles[i].id - 1

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
            [ mf.degreeAt(-575), mf.track(-575) ],
        )

    @staticmethod
    def mf_keeppick():
        w = expr.w
        mf = expr.mf
        return (
            [ mf.degree(75), mf.move(-50) ],
            [ mf.degree(200), mf.move(-25) ],
            [ mf.degreeAt(-510), mf.track(-510) ],
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
            [ mf.degreeAt(-510, tolerance, stable), mf.track(-510) ],
        )

    @staticmethod
    def mf_releasemax():
        w = expr.w
        mf = expr.mf
        return (
            [ mf.degreeAt(-650), mf.track(-650) ],
        )

    @staticmethod
    def mf_release():
        w = expr.w
        mf = expr.mf
        return (
            [ mf.degreeAt(-575), mf.track(-575) ],
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
            [ mb.degreeAt(-40), mb.track(-40) ],
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
            [ mb.degreeAt(-170), mb.track(-170) ],
        )

    @staticmethod
    def mb_fixback():
        w = expr.w
        mb = expr.mb
        return (
            [ mb.degreeAt(-90), mb.track(-90) ],
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
