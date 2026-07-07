"""
Program's Entry Point for WRO2026 Senior
"""

from pybricks.parameters import Port, Color, Direction
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.hubs import PrimeHub

from huskylens import Huskylens, Block, ALGORITHM_COLOR_RECOGNITION
from drivebase import DriveBaseAPI, MissionMotor, PIVOT_LEFT, PIVOT_RIGHT
from math import ceil, floor

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
        30:  (3.0, 2.5, 0.05), -30:  (2.0, 2.5, 0.05),
        50:  (3.0, 6.0, 0.075), -50:  (2.4, 6.0, 0.065),
        75:  (3.0, 10.0, 0.1), -75:  (3.0, 10.0, 0.1),
        100: (4.0, 16.0, 0.2), -100: (4.0, 16.0, 0.2),
    },
    tagline_params = {
        30:  (0.65, 0.0, 0.065), -30:  (0.65, 0.0, 0.065),
        50:  (1.0, 0.0, 0.075), -50:  (1.0, 0.0, 0.075),
        75:  (1.0, 0.0, 0.08), -75:  (1.0, 0.0, 0.08),
        100: (1.0, 0.0, 0.1), -100: (1.0, 0.0, 0.1),
    },
    turn_params = {
        90:  (1.8, 0.0, 0.02),
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

YELLOW = 0
BLUE = 1
GREEN = 2
WHITE = 3

M2_PICK = 50
M1_DOWN = 285
BRAKE_TIME = 20

def mosaicSection():
    w.runConcurrent( #SET0 M2
        [ w.ms(400), m2.move(75) ], 
        m2.resetEncoder(),
        m2.hold(),
    )

    w.runConcurrent( #SET0 M1
        [ w.ms(400), m1.move(-75) ],
        m1.resetEncoder(),
        m1.hold(),
    )

    w.run( #SET0, GOTO YELLOW
        [ w.ms(300), w.moveTank(-50, -50) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        w.resetImu(),
        w.resetEncoder(),
        [ w.heading(45, stable=1), w.turn(PIVOT_RIGHT, kp=8.2, kd=0.24) ],
        [ w.heading(0), w.turn(PIVOT_LEFT, kp=8.2, kd=0.24) ],
    )

    w.runConcurrent(
        [ m2.degree(M2_PICK), m2.move(-75) ],
        m2.brake(),
    )

    w.runConcurrent(
        [ m1.degree(M1_DOWN), m1.move(50) ],
        m1.brake(),
    )

    w.run(
        [ w.blackReflection(20), w.straight(50) ],
    )

def keep(left_or_right):
    w.run(
        [ w.heading(-115 if left_or_right == PIVOT_RIGHT else -65), w.turn(left_or_right, kp=8.2, kd=0.24) ],
        w.brake(),
    )

    w.runConcurrent(
        [ m2.degree(80), m2.move(-100) ], m2.brake()
    )

    w.runConcurrent(
        [ w.ms(50) ],
        [ w.ms(500), m1.move(-100) ],
        [ w.ms(250), m1.move(-75) ],
        m1.resetEncoder(),
        m1.hold(),
    )

    w.run( # KEEP
        [ w.ms(250) ],
        [ w.heading(-90), w.turn(left_or_right, kp=8.2, kd=0.24) ],
        [ w.ms(50), w.brake() ],
        w.resetEncoder(),
        [ w.degree(25), w.straight(50) ],
        [ w.degree(80), w.straight(75) ],
    )

    w.runConcurrent(
        [ m1.degree(100), m1.move(100) ],
    )

    w.run(
        [ w.degree(125), w.straight(75) ],
        [ w.degree(150), w.straight(50) ],
        [ w.ms(10), w.brake() ],
        w.resetEncoder(),
        [ m1.degree(160), m1.move(75) ],
        [ w.ms(10), m1.brake() ],
        m1.move(-10),
    )

    w.runConcurrent( # SET0 M2, RETURN M1, M2 TO ORIGINAL POSITION
        [ w.ms(400) ],
        [ m1.degree(M1_DOWN), m1.move(50) ],
        m1.brake(),

        [ w.ms(200), m2.move(100) ],
        [ w.ms(200), m2.move(75) ],
        m2.resetEncoder(),

        [ m2.degree(M2_PICK), m2.move(-75) ],
        m2.brake(),
    )

    w.run( # RETURN TO ORIGINAL POSITION
        [ w.degree(25), w.straight(-50) ],
        [ w.degree(115), w.straight(-75) ],
        [ w.degree(140), w.straight(-50) ],
        [ w.ms(BRAKE_TIME), w.brake() ]
    )

class moveToDestinationEx4:
    def __init__(self, pick_queue, initial_pos):
        self.picked = 0
        self.current_pos = initial_pos
        self.pick_queue = pick_queue
        self.pick_map = [ 0, 0, 0, 0 ]

    def gotoHome(self):
        distance = abs(-1 - self.current_pos)

        # INITIALLY TURN AND ACCELS
        w.run(
            [ w.heading(-180, tolerance=0.5), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            [ w.ms(100), w.straight(30) ],
            [ w.ms(50), w.straight(40) ],
            [ w.ms(50), w.straight(50) ],
        )

        # MOVE TO DESTINATION
        if distance == 4:
            w.run(
                [ w.blackReflection(20), w.straight(75) ],
                [ w.all(w.blackReflection(20), w.ms(50)), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(50)), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(50)), w.straight(75) ],
            )
        if distance == 3:
            w.run(
                [ w.blackReflection(20), w.straight(75) ],
                [ w.all(w.blackReflection(20), w.ms(50)), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(50)), w.straight(75) ],
            )
        if distance == 2:
            w.run(
                [ w.blackReflection(20), w.straight(75) ],
                [ w.all(w.blackReflection(20), w.ms(50)), w.straight(75) ],
            )
        if distance == 1:
            w.run(
                [ w.blackReflection(20), w.straight(75) ],
            )

        w.run(
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            [ w.degree(70), w.straight(50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            [ w.heading(-271), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder()
        )

    def gotoMid(self):
        middle = 1.5
        next_dist = middle - self.pick_queue[self.picked-1]

        if next_dist > 0:
            sign = 1
            angle = 0
        if next_dist < 0:
            sign = -1
            angle = -180

        w.run(
            [ w.blackReflection(20), w.straight(-50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            [ w.heading(angle), w.turn() ],
            [ w.blackReflection(20), w.straight(75) ],
            [ w.ms(10), w.brake() ],
            w.resetEncoder(),
            [ w.degree(50), w.straight(50) ],
            [ w.heading(90 if sign == 1 else -270), w.turn() ],
            [ w.ms(10), w.brake() ],
            w.resetEncoder(),
        )

        w.runConcurrent(
            [ m2.degree(M2_PICK), m2.move(-75) ],
            m2.brake(),
            [ w.ms(300) ],
            [ w.ms(50), m2.move(100) ],
            m2.move(30),
        )
        
        w.run(
            [ w.degree(250), w.tagline(50, 15, sign) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            [ m2.degree(90), m2.move(-100) ],
        )

        w.runConcurrent(
            [ m2.degree(160), m2.move(-100) ],
            m2.brake(),
        )

        w.runConcurrent(
            [ w.ms(50) ],
            [ w.ms(550), m1.move(-100) ],
            [ w.ms(200), m1.move(-75) ],
            m1.resetEncoder(),
            m1.hold(),
        )

        w.run(
            [ w.ms(550) ],
            [ w.degree(225), w.tagline(-50, 15, sign) ],
            w.brake(),
            w.resetEncoder(),
            [ m1.degree(100), m1.move(100) ],
            [ m1.degree(200), m1.move(50) ],
        )

        w.runConcurrent(
            [ m1.degree(310), m1.move(50) ],
            m1.brake(),
        )

        w.run(
            [ w.degree(325), w.tagline(50, 15, sign) ],
            [ w.degree(425), w.straight(75) ],
        )

        w.runConcurrent(
            m2.move(100)
        )

        w.run(
            [ w.degree(525), w.straight(50) ],
            [ w.ms(200), w.straight(30) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            w.resetImu(90),
            [ w.degree(15), w.straight(-50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            m1.resetEncoder(),
            w.resetEncoder(),
            [ m1.degree(90), m1.move(-100) ],
            m1.move(-10),
            [ w.degree(130), w.straight(50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            [ m2.degree(140), m2.move(-50) ],
            [ w.ms(100), m2.brake() ],
            [ w.ms(300), m2.move(100) ],
            m2.brake(),
            m1.move(-100),

            [ w.heading(-90), w.turn(kp=3.2, kd=0.05) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            m1.brake(),
            [ w.degree(25), w.straight(-50) ],
            [ w.degree(50), w.straight(-75) ],
            [ w.degree(125), w.straight(-100) ],
            [ w.ms(200), w.straight(-30) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            w.resetImu(-90)
        )

        w.runConcurrent( #SET0 M2
            [ w.ms(300), m2.move(75) ], 
            m2.resetEncoder(),
            m2.hold(),
            [ m2.degree(M2_PICK), m2.move(-75) ],
            m2.brake(),
        )

        w.runConcurrent( #SET0 M1
            [ w.ms(300), m1.move(-75) ],
            m1.resetEncoder(),
            m1.hold(),
            [ m1.degree(100), m1.move(100) ],
            [ m1.degree(M1_DOWN), m1.move(50) ],
            m1.brake(),
        )

        w.run(
            [ w.degree(50), w.straight(50) ],
            [ w.degree(100), w.straight(75) ],
            [ w.degree(825), w.straight(100) ],
            [ w.degree(875), w.straight(75) ],
            [ w.degree(925), w.straight(50) ],
            [ w.blackReflection(20), w.straight(30) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
        )

        self.current_pos = middle

    def gotoNext(self):
        next_color = self.pick_queue[self.picked]
        next_dist = next_color - self.current_pos
        sign = 0
        angle = -90

        if next_dist > 0:
            sign = 1
            angle = -0
            next_dist = ceil(next_dist)
        if next_dist < 0:
            sign = -1
            angle = -180
            next_dist = floor(next_dist)

        distance = abs(next_dist)

        # INITIALLY TURN AND ACCELS
        if distance != 0:
            w.run(
                [ w.heading(angle, tolerance=0.5), w.turn() ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                w.resetEncoder()
            )
            
            if self.current_pos != 1.5:
                w.run(
                    [ w.degree(25), w.straight(50) ],
                )

        # MOVE TO DESTINATION
        if distance == 4:
            w.run(
                [ w.blackReflection(20), w.straight(75) ],
                [ w.all(w.blackReflection(20), w.ms(150)), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(100)), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(100)), w.straight(75) ],
            )
        if distance == 3:
            w.run(
                [ w.blackReflection(20), w.straight(75) ],
                [ w.all(w.blackReflection(20), w.ms(150)), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(100)), w.straight(75) ],
            )
        if distance == 2:
            w.run(
                [ w.blackReflection(20), w.straight(75) ],
                [ w.all(w.blackReflection(20), w.ms(150)), w.straight(75) ],
            )
        if distance == 1:
            w.run(
                [ w.blackReflection(20), w.straight(75) ],
            )

        if distance != 0:
            if sign == -1:
                w.run(
                    [ w.ms(10), w.brake() ],
                    w.resetEncoder(),
                    [ w.degree(50), w.straight(50) ],
                    [ w.degree(110), w.straight(75) ],
                    [ w.degree(160), w.straight(50) ]
                )

            w.run(
                [ w.ms(80), w.brake() ],
                [ w.heading(-90), w.turn() ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                w.resetEncoder(),
            )

        w.run(
            w.resetEncoder(),
            [ w.degree(25), w.straight(-50) ],
            [ w.degree(50), w.straight(-75) ],
            [ w.blackReflection(20), w.straight(-50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
        )

        w.runConcurrent(
            [ m2.degree(M2_PICK), m2.move(-75) ],
            m2.brake()
        )

        counter = self.pick_map[next_color]
        row = counter // 2
        deg_in = [ 0, 110, 230 ]
        deg_out = [ 0, 110, 215 ]
        deg_turn = [ -77, -80, -83 ]

        if counter % 2 == 0:
            w.run(
                [ w.degree(25), w.straight(50) ],
                [ w.degree(225 + deg_in[row]), w.straight(75) ],
                [ w.degree(250 + deg_in[row]), w.straight(50) ],
            )

        if counter % 2 != 0:
            w.run(
                [ w.heading(deg_turn[row]), w.turn(PIVOT_RIGHT, kp=8.2, kd=0.24) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                w.resetEncoder(),
                [ w.degree(25), w.straight(50) ],
                [ w.degree(240 + deg_in[row]), w.straight(75) ],
                [ w.degree(265 + deg_in[row]), w.straight(50) ],
            )

        w.run(
            w.brake(),
            [ w.ms(100), m2.move(100) ],
            m2.move(30),
            w.resetEncoder(),
        )

        if counter % 2 == 0:
            w.run(
                [ w.degree(25), w.straight(-50) ],
                [ w.degree(55 + deg_out[row]), w.straight(-75) ],
                [ w.degree(80 + deg_out[row]), w.straight(-50) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
            )
        
        if counter % 2 != 0:
            w.run(
                [ w.degree(25), w.straight(-50) ],
                [ w.degree(75 + deg_out[row]), w.straight(-75) ],
                [ w.degree(100 + deg_out[row]), w.straight(-50) ],
                [ w.heading(-90), w.turn(PIVOT_RIGHT, kp=8.2, kd=0.24) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
            )

        self.pick_map[next_color] += 1
        self.current_pos = next_color
        self.picked += 1

def main():
    mosaicSection()

    test = moveToDestinationEx4([YELLOW, WHITE, YELLOW, GREEN, WHITE, BLUE, YELLOW, BLUE, WHITE, WHITE, GREEN, BLUE ], -1)
    test.gotoNext()
    test.gotoNext()
    keep(PIVOT_RIGHT)
    test.gotoNext()
    test.gotoNext()
    keep(PIVOT_LEFT)
    test.gotoNext()
    test.gotoNext()
    test.gotoMid()
    test.gotoNext()
    test.gotoNext()
    keep(PIVOT_RIGHT)
    test.gotoNext()
    test.gotoNext()
    keep(PIVOT_LEFT)
    test.gotoNext()
    test.gotoNext()
    
    return

    w.run(
        [ w.degree(100), w.straight(50) ],
        [ w.degree(200), w.straight(75) ],
        [ w.blackReflection(20), w.straight(100) ],
        [ w.all(w.blackReflection(20), w.ms(50)), w.straight(100) ],
        [ w.all(w.blackReflection(20), w.ms(50)), w.straight(100) ],
        [ w.all(w.blackReflection(20), w.ms(50)), w.straight(75) ],
        w.brake(),
        w.resetEncoder(),
        [ w.degree(50), w.straight(50) ],
        [ w.degree(75), w.straight(75) ],
        [ w.degree(125), w.straight(50) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        [ w.heading(-362), w.turn() ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        w.resetEncoder(),
        [ w.degree(100), w.straight(-50) ],
        [ w.ms(200), w.straight(-30) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        w.resetEncoder(),
        w.resetImu(),
        [ w.degree(100), w.straight(50) ],
        [ w.degree(200), w.straight(75) ],
        [ w.degree(600), w.straight(100) ],
        [ w.degree(700), w.straight(75) ],
        [ w.blackReflection(20), w.straight(50) ],
        w.brake(),
        w.resetEncoder(),
        [ w.degree(60), w.straight(50) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        [ w.heading(-90), w.turn() ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        w.resetEncoder(),
        [ m2.degree(90), m2.move(-100) ],
    )

    w.runConcurrent(
        [ m2.degree(160), m2.move(-100) ],
        m2.brake(),
    )

    w.runConcurrent(
        [ w.ms(50) ],
        [ w.ms(500), m1.move(-100) ],
        [ w.ms(250), m1.move(-75) ],
        m1.resetEncoder(),
        m1.hold(),
    )

    w.run(
        [ w.ms(500) ],
        [ w.degree(100), w.straight(-50) ],
    )

    w.runConcurrent(
        [ m1.degree(100), m1.move(100) ],
        [ m1.degree(200), m1.move(50) ],
        m1.brake(),
    )

    w.runConcurrent(
        [ m1.degree(300), m1.move(50) ],
    )

    w.run(
        [ w.degree(450), w.straight(-100) ],
        [ w.degree(550), w.straight(-50) ],
        [ w.ms(200), w.straight(-30) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        w.resetImu(),
        w.resetEncoder(),
        [ w.degree(100), w.straight(50) ],
        [ w.degree(200), w.straight(75) ],
        [ w.degree(1000), w.straight(100) ],
    )

    w.runConcurrent(
        [ w.ms(250), m2.move(75) ],
        m2.move(100)
    )

    w.run(
        [ w.degree(1100), w.straight(75) ],
        [ w.degree(1200), w.straight(50) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
    )

    w.run(
        [ w.degree(1300), w.straight(50) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        w.resetEncoder(),
        [ w.ms(250) ],
        [ w.degree(200), w.straight(50) ],
        [ w.ms(200), w.straight(30) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        w.resetEncoder(),
        w.resetImu(),
        [ w.degree(50), w.straight(-50) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        [ m2.degree(140), m2.move(-50) ],
        [ w.ms(100), m2.brake() ],
        [ w.ms(300), m2.move(100) ],
        m2.brake(),
    )

print(f"charging current: {w._hub.charger.current()}")
print(f"battery voltage:  {w._hub.battery.voltage()}")
main()
