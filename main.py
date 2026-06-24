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
        30:  (2.0, 2.5, 0.05), -30:  (2.0, 3.0, 0.05),
        50:  (2.4, 6.0, 0.065), -50:  (2.4, 6.0, 0.065),
        75:  (3.0, 10.0, 0.1), -75:  (3.0, 10.0, 0.1),
        100: (4.0, 16.0, 0.2), -100: (4.0, 16.0, 0.2),
    },
    tagline_params = {
        30:  (0.0, 0.0, 0.0), -30:  (0.0, 0.0, 0.0),
        50:  (0.65, 0.0, 0.05), -50:  (0.0, 0.0, 0.0),
        75:  (0.9, 0.0, 0.1), -75:  (0.0, 0.0, 0.0),
        100: (0.0, 0.0, 0.0), -100: (0.0, 0.0, 0.0),
    },
    turn_params = {
        30:  (2.1, 0.0, 0.04),
        90:  (2.1, 0.0, 0.04),
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
        [ w.ms(300), m2.move(100) ], 
        m2.resetEncoder(),
        m2.coast(),
        [ w.ms(50) ],
        m2.hold(),
    )

    w.runConcurrent( #SET0 M1
        [ w.ms(300), m1.move(-100) ], 
        m1.resetEncoder(),
        m1.coast(),
        [ w.ms(50) ],
        m1.hold(),
    )

    w.run( #SET0, GOTO YELLOW
        [ w.ms(300), w.moveTank(-50, -50) ],
        [ w.ms(50), w.brake() ],
        w.resetImu(),
        w.resetEncoder(),
        [ w.heading(45, stable=1), w.turn(PIVOT_RIGHT, kp=4.0) ],
        [ w.heading(0, stable=3), w.turn(PIVOT_LEFT, kp=4.0) ],
    )

    w.runConcurrent(
        [ m2.degree(65), m2.move(-75) ],
        m2.brake(),
    )

    w.runConcurrent(
        [ m1.degree(275), m1.move(50) ],
        m1.brake(),
    )

    w.run(
        [ w.ms(50), w.brake() ],
        [ w.blackReflection(20), w.straight(50) ],
    )

def keep(left_or_right):
    w.run(
        [ w.heading(-135 if left_or_right == PIVOT_RIGHT else -55), w.turn(left_or_right, kp=4.0) ], w.brake(),
    )

    w.runConcurrent(
        [ m2.degree(90), m2.move(-100) ], m2.brake()
    )

    w.run( # KEEP
        [ w.ms(100) ],
        [ w.ms(500), m1.move(-100) ],
        m1.hold(),
        [ w.heading(-90), w.turn(left_or_right, kp=4.0) ],
        [ w.ms(50), w.brake() ],
        w.resetEncoder(),
        [ w.degree(175), w.straight(50) ],
        w.brake(),
        [ w.ms(200), m1.move(75) ],
        m1.hold()
    )

    w.runConcurrent( # SET0 M2, RETURN M1, M2 TO ORIGINAL POSITION
        [ w.ms(200) ],
        [ w.ms(270), m1.move(50) ],
        m1.brake(),

        [ w.ms(200), m2.move(100) ],
        m2.resetEncoder(),

        [ m2.degree(65), m2.move(-80) ],
        m2.brake(),
    )

    w.run( # RETURN TO ORIGINAL POSITION
        [ w.ms(100), w.straight(-50) ],
        [ w.ms(200), w.straight(-75) ],
        [ w.blackReflection(20), w.straight(-50) ],
        [ w.ms(50), w.brake() ],
        w.resetEncoder(),
        [ w.degree(125), w.straight(50) ],
        [ w.ms(50), w.brake() ],
    )

class moveToDestination:
    def __init__(self, pick_queue):
        self.picked = 0
        self.current = -1
        self.pick_queue = pick_queue,
        self.picked_map: list[list[int]] = [ #typedef struct _pickmap { vector<pair<latest, counter>> }
            { latest: 0, counter: 0 }, #YELLOW
            { latest: 0, counter: 0 }, #BLUE
            { latest: 0, counter: 0 }, #GREEN
            { latest: 0, counter: 0 }, #WHITE
        ]

    def gotoNext(self):
        next_color = self.pick_queue[self.picked]
        next_dist = next_color - self.current
        step = abs(next_dist)
        positive = True if next_dist >= 0 else False
        angle = 0 if positive else -180

        # INITIALLY TURN AND ACCELS
        w.run(
            [ w.heading(angle, tolerance=0.5), w.turn() ],
            [ w.ms(50), w.brake() ],
            [ w.ms(100), w.straight(30) ],
            [ w.ms(200), w.straight(50) ],
        )

        # MOVE TO DESTINATION
        if step == 4:
            w.run(
                [ w.blackReflection(20), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(50)), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(50)), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(50)), w.straight(75) ],
            )
        if step == 3:
            w.run(
                [ w.blackReflection(20), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(50)), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(50)), w.straight(75) ],
            )
        if step == 2:
            w.run(
                [ w.blackReflection(20), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(50)), w.straight(75) ]
            )
        if step == 1:
            w.run(
                [ w.blackReflection(20), w.straight(75) ],
                [ w.ms(10), w.straight(50) ]
            )

        w.run(
            [ w.ms(50), w.brake() ],
            [ w.heading(-90), w.turn() ],
            [ w.ms(50), w.brake() ],
            w.resetEncoder(),
        )

        w.runConcurrent(
            [ m2.degree(65), m2.move(-100) ],
            m2.brake()
        )

        # PICK AND RETURN TO ORIGINAL POSITION
        w.run(
            [ w.degree(120), w.straight(50) ],
            w.brake(),
            [ w.ms(100), m2.move(100) ],
            m2.move(30),
            w.resetEncoder(),
            [ w.degree(125), w.straight(-50) ],
            [ w.ms(50), w.brake() ],
        )

        picked_map = self.picked_map[next_color]
        picked_map.latest = 1 if positive else -1
        picked_map.counter += 1

        self.current = next_color
        self.picked += 1


def main():
    mosaicSection()

    test = moveToDestination([ WHITE, GREEN, BLUE, YELLOW, WHITE, GREEN ])
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
