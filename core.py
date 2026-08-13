"""
Program's Entry Point for WRO2026 Senior
"""

from pybricks.parameters import Port, Color, Direction
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.hubs import PrimeHub
from pybricks.tools import wait

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

    WHITECLR = Color(210, 30, 80)
    YELLOWCLR = Color(56, 60, 65)
    BLUECLR = Color(230, 85, 20)
    GREENCLR = Color(190, 40, 15)

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
    mb = MissionMotor(Motor(Port.A, Direction.COUNTERCLOCKWISE), kp=50000, ki=25000, kd=1250)
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
            40:  (1.15, 0.0, 0.06),
            50:  (1.3, 0.0, 0.065),
            75:  (1.5, 0.0, 0.08),
        },
        turn_params = {
            30: (4.0, 0.0, 0.065),
            90:  (3.0, 0.0, 0.04375),
        },
        pturn_params = {
            30:  (8.685, 0.0, 0.2),
            90:  (9.5, 0.0, 0.275),
        },
        color_params = [
            YELLOWCLR, GREENCLR, BLUECLR, WHITECLR, Color(230, 40, 15)
        ]
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
        mf = expr.mf
        return (
            [ mf.stable(stable=20), mf.move(75) ],
            mf.resetEncoder(),
            mf.brake()
        )

    @staticmethod
    def mf_set0d():
        mf = expr.mf
        return (
            [ mf.degreeAt(-50), mf.track(-50) ],
            [ mf.stable(stable=20), mf.move(75) ],
            mf.resetEncoder(),
            mf.brake()
        )

    @staticmethod
    def mb_set0():
        mb = expr.mb
        return (
            [ mb.stable(stable=15), mb.move(-65) ],
            mb.resetEncoder(),
            mb.brake()
        )

    @staticmethod
    def mb_set0d():
        mb = expr.mb
        return (
            [ mb.degreeAt(50), mb.track(50), ],
            [ mb.stable(stable=15), mb.move(-65) ],
            mb.resetEncoder(),
            mb.brake()
        )


print(f"charging current: {expr.w._hub.charger.current()}")
print(f"battery voltage:  {expr.w._hub.battery.voltage()}")
