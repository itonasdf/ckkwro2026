"""
Program's Entry Point for WRO2026 Senior
"""

from pybricks.parameters import Port, Color, Direction
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.hubs import PrimeHub

from huskylens import Huskylens, Block, ALGORITHM_COLOR_RECOGNITION
from beta_drivebase import DriveBaseAPI, MissionMotor, PIVOT_LEFT, PIVOT_RIGHT

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

YELLOW = 0
BLUE = 1
GREEN = 2
WHITE = 3

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
        [ w.ms(50), w.brake() ],
        w.resetImu(),
        w.resetEncoder(),
        [ w.heading(45, stable=1), w.turn(PIVOT_RIGHT, kp=5.0) ],
        [ w.heading(0, tolerance=0.5), w.turn(PIVOT_LEFT, kp=5.0) ],
    )

    w.runConcurrent(
        [ m2.degree(72), m2.move(-75) ],
        m2.brake(),
    )

    w.runConcurrent(
        [ m1.degree(300), m1.move(50) ],
        m1.brake(),
    )

    w.run(
        [ w.ms(50), w.brake() ],
        [ w.blackReflection(20), w.straight(50) ],
    )

def keep(left_or_right):
    w.run(
        [ w.heading(-110 if left_or_right == PIVOT_RIGHT else -70), w.turn(left_or_right, kp=5.0) ], w.brake(),
    )

    w.runConcurrent(
        [ m2.degree(90), m2.move(-100) ], m2.brake()
    )

    w.runConcurrent(
        [ w.ms(50) ],
        [ w.ms(600), m1.move(-100) ],
        m1.resetEncoder(),
        m1.hold(),
    )

    w.run( # KEEP
        [ w.ms(250) ],
        [ w.heading(-90), w.turn(left_or_right, kp=5.0) ],
        [ w.ms(50), w.brake() ],
        w.resetEncoder(),
        [ w.degree(175), w.straight(50) ],
        w.brake(),
        w.resetEncoder(),
        [ m1.degree(100), m1.move(100) ],
        [ m1.degree(175), m1.move(75) ],
        m1.brake(),
        m1.move(-10),
    )

    w.runConcurrent( # SET0 M2, RETURN M1, M2 TO ORIGINAL POSITION
        [ w.ms(500) ],

        [ m1.degree(300), m1.move(50) ],
        m1.brake(),

        [ w.ms(200), m2.move(100) ],
        m2.resetEncoder(),

        [ m2.degree(72), m2.move(-75) ],
        m2.brake(),
    )

    w.run( # RETURN TO ORIGINAL POSITION
        [ w.ms(100) ],
        [ w.degree(150), w.straight(-50) ],
        [ w.ms(50), w.brake() ]
    )

BREAK_TIME = 20
class moveToDestinationEx3:
    def __init__(self, pick_queue):
        self.picked = 0
        self.current_pos = -1
        self.pick_queue = pick_queue
        self.pick_map = [ 0, 0, 0, 0 ]

    def gotoNext(self):
        next_color = self.pick_queue[self.picked]
        next_dist = next_color - self.current_pos
        distance = abs(next_dist)
        sign = 0
        angle = -90

        if next_dist > 0:
            sign = 1
            angle = 0
        if next_dist < 0:
            sign = -1
            angle = -180

        # INITIALLY TURN AND ACCELS
        if distance != 0:
            w.run(
                [ w.heading(angle, tolerance=0.5), w.turn() ],
                [ w.ms(BREAK_TIME), w.brake() ],
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

        if distance != 0:
            w.run(
                w.brake(),
                w.resetEncoder(),
                [ w.degree(20), w.straight(50) ] 
            )

            if sign == -1:
                w.run(
                    [ w.degree(110), w.straight(50) ]
                )

            w.run(
                [ w.ms(BREAK_TIME), w.brake() ],
                [ w.heading(-90), w.turn() ],
                [ w.ms(BREAK_TIME), w.brake() ],
                w.resetEncoder()
            )

        w.run(
            [ w.blackReflection(20), w.straight(-50) ],
            [ w.ms(BREAK_TIME), w.brake() ],
            w.resetEncoder(),
            [ w.degree(10), w.straight(30) ],
            [ w.degree(115), w.straight(50) ],
            [ w.degree(125), w.straight(30) ],
            [ w.ms(BREAK_TIME), w.brake() ],
            w.resetEncoder()
        )

        w.runConcurrent(
            [ m2.degree(72), m2.move(-75) ],
            m2.brake()
        )

        counter = self.pick_map[next_color]
        turn = PIVOT_RIGHT if counter % 2 != 0 else PIVOT_LEFT
        angle = -77 if counter % 2 != 0 else -95
        
        w.run(
            [ w.degree(100 * (counter // 2)), w.straight(50) ],
            [ w.ms(BREAK_TIME), w.brake() ],
            [ w.heading(angle), w.turn(turn, kp=5.0) ],
            [ w.ms(BREAK_TIME), w.brake() ],
            w.resetEncoder(),
            [ w.degree(100), w.straight(50) ],
            w.brake(),
            [ w.ms(50), m2.move(100) ],
            m2.move(30),
            w.resetEncoder(),
            [ w.degree(100), w.straight(-50) ],
            [ w.ms(BREAK_TIME), w.brake() ],
            [ w.heading(-90), w.turn(turn, kp=5.0) ],
            [ w.ms(BREAK_TIME), w.brake() ],
            w.resetEncoder(),
            [ w.degree(90 * (counter // 2)), w.straight(-50) ],
            [ w.ms(BREAK_TIME), w.brake() ],
        )

        self.pick_map[next_color] += 1
        self.current_pos = next_color
        self.picked += 1

def main():
    mosaicSection()

    test = moveToDestinationEx3([WHITE, YELLOW, GREEN, BLUE, WHITE, WHITE])
    test.gotoNext()
    test.gotoNext()
    keep(PIVOT_RIGHT)
    test.gotoNext()
    test.gotoNext()
    keep(PIVOT_LEFT)
    test.gotoNext()
    test.gotoNext()

    print(f"charging current: {w._hub.charger.current()}")
    print(f"battery voltage:  {w._hub.battery.voltage()}")

main()
