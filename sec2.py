from wro2026corev2 import expr
from math import ceil, floor

husky = expr.husky
w = expr.w
mf = expr.mf
mb = expr.mb

YELLOW = expr.YELLOW
BLUE = expr.BLUE
GREEN = expr.GREEN
WHITE = expr.WHITE
COLOR = expr.COLOR
color_var = expr.color_var

ALGORITHM_COLOR_RECOGNITION = expr.ALGORITHM_COLOR_RECOGNITION
PIVOT_LEFT = expr.PIVOT_LEFT
PIVOT_RIGHT = expr.PIVOT_RIGHT
BRAKE_TIME = expr.BRAKE_TIME

getMosaicData = expr.getMosaicData







def set0():
    w.runConcurrent(
        *expr.mf_set0(),
        *expr.mb_set0()
    )

    w.run(
        [ w.ms(500), w.straight(-50) ],
        [ w.ms(500), w.straight(-30) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        w.resetImu(),
        w.resetEncoder(),

        w.heading(40),
        [ w.degree(160), w.straight(80, kp=6.0) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        w.resetEncoder(),

        w.heading(0),
        [ w.degree(160), w.straight(80, kp=6.0) ],
    )

class MoveToDestinationEx5:
    def __init__(self, pick_queue: list[int]):
        self.picked = 0
        self.current = -1
        self.queue = pick_queue
        self.pickmap = [ 0, 0, 0, 0 ]

    def pickUp(self):
        middle = 1.5
        prev = self.queue[self.picked-1]
        next_dist = middle - prev

        if next_dist > 0:
            sign = 1
            angle = 0
        if next_dist < 0:
            sign = -1
            angle = -180

        w.run(
            w.resetEncoder(),
            [ w.degree(75), w.straight(-75) ],
            [ w.blackReflection(20), w.straight(-50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            [ w.degree(25), w.straight(50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            [ w.heading(angle), w.turn() ],
        )

        if prev == YELLOW or prev == BLUE:
            w.run(
                [ w.blackReflection(20), w.straight(75) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                w.resetEncoder(),
                [ w.degree(100), w.straight(75) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
            )
        if prev == GREEN or prev == WHITE:
            w.run(
                [ w.blackReflection(20), w.straight(-50 if prev == GREEN else 75) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                w.resetEncoder(),
                [ w.degree(80), w.straight(75) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
            )

        w.run(
            [ w.heading(90.5 if sign == 1 else -271.75), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
        )

        w.runConcurrent(
            *expr.mf_release(),
        )

        w.run(
            [ w.degree(100), w.straight(50) ],
            [ w.degree(200), w.straight(75) ],
            [ w.degree(300), w.straight(50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            [ w.ms(250), mf.move(100) ],
        )

        w.runConcurrent(
            [ w.ms(150), mf.move(100) ],
            [ w.ms(150), mf.move(75) ],
            *expr.mf_set0(),
            [ mf.degree(75), mf.move(-75) ],
            [ mf.degree(200), mf.move(-30) ],
            [ mf.degreeAt(-445), mf.track(-445) ],
        )

        w.run(
            [ w.degree(50), w.straight(-50) ],
            [ w.degree(425), w.straight(-75) ],
            [ w.degree(475), w.straight(-50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            
            [ w.degree(100), w.straight(50) ],
            [ w.degree(450), w.straight(75) ],
        )

        self.current = middle
        return sign

    def gotoMid(self):
        sign = self.pickUp()
        w.runConcurrent(
            [ w.degree(575), mf.move(100) ],
            [ w.degree(625), mf.move(75) ],
            mf.move(35),
        )

        w.run(
            [ w.degree(875), w.straight(75) ],
            [ w.degree(975), w.straight(50) ],
            [ w.ms(500), w.straight(40) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            w.resetImu(90),

            [ mf.degreeAt(-360), mf.track(-360) ],
            [ mf.degreeAt(-515), mf.track(-515) ],
            [ w.ms(100), w.moveTank(-50, 50) ],
            [ w.ms(180), w.moveTank(50, -50) ],
            [ w.heading(90), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(60), w.straight(-50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ mf.degreeAt(-360), mf.track(-360) ],
            [ w.degree(35), w.straight(-50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ mf.degreeAt(-360), mf.track(-360) ],
            [ mf.degreeAt(-515), mf.track(-515) ],
            [ w.ms(100), w.moveTank(-50, 50) ],
            [ w.ms(180), w.moveTank(50, -50) ],
            [ w.heading(90), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(50), w.straight(-50) ],
            [ w.degree(100), w.straight(-75) ],
            [ w.degree(250), w.straight(-100) ],
        )

        w.runConcurrent(
            *expr.mf_set0d()
        )

        w.run(
            [ w.degree(675), w.straight(-100) ],
            [ w.degree(725), w.straight(-75) ],
            [ w.degree(775), w.straight(-50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
        )

    def gotoTangrad(self):
        sign = self.pickUp()
        w.runConcurrent(
            [ w.degree(625), mf.move(100) ],
            [ w.degree(675), mf.move(75) ],
            mf.move(35),
        )

        w.runConcurrent(
            *expr.mb_set0(),
            *expr.mb_fixback(),
        )

        w.run(
            [ w.degree(875), w.straight(75) ],
            [ w.degree(975), w.straight(50) ],
            [ w.ms(500), w.straight(40) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            w.resetImu(90),

            [ w.degree(50), w.straight(-50) ],
            [ w.degree(150), w.straight(-75) ],
            [ w.degree(200), w.straight(-50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],

            [ w.heading(135), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(50), w.straight(50) ],
            [ w.degree(400), w.straight(75) ],
            [ w.degree(450), w.straight(50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],

            [ w.heading(90), w.turn(PIVOT_LEFT) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(50), w.straight(50) ],
            [ w.degree(100), w.straight(75) ],
            [ w.degree(200), w.straight(100) ],
            [ w.degree(250), w.straight(75) ],
            [ w.negate(w.colorReflection(COLOR.YELLOW)), w.straight(50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            
            [ w.heading(45), w.turn(PIVOT_LEFT) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(50), w.straight(50) ],
            [ w.degree(100), w.straight(75) ],
            [ w.degree(250), w.straight(100) ],
            [ w.degree(300), w.straight(75) ],
            [ w.blackReflection(20), w.straight(50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(75), w.straight(75) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            [ w.heading(-90.5), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(50), w.straight(50) ],
            [ w.degree(150), w.straight(75) ],
            [ w.degree(200), w.straight(50) ],
            [ w.ms(500), w.straight(40) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            w.resetImu(90),

            [ mf.degreeAt(-360), mf.track(-360) ],
            [ mf.degreeAt(-515), mf.track(-515) ],
            [ w.ms(100), w.moveTank(-50, 50) ],
            [ w.ms(180), w.moveTank(50, -50) ],
            [ w.heading(90), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(60), w.straight(-50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ mf.degreeAt(-360), mf.track(-360) ],
            [ w.degree(35), w.straight(-50) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ mf.degreeAt(-360), mf.track(-360) ],
            [ mf.degreeAt(-515), mf.track(-515) ],
            [ w.ms(100), w.moveTank(-50, 50) ],
            [ w.ms(180), w.moveTank(50, -50) ],
            [ w.heading(90), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
        )

    def keep(self, left_or_right):
        w.run(
            [ w.heading(-115 if left_or_right == PIVOT_RIGHT else -65), w.turn(left_or_right) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder()
        )

        w.runConcurrent(
            *expr.mf_release(),
            [ w.degree(100) ],
            *expr.mf_set0d()
        )

        w.run(
            [ w.degree(25), w.straight(-60) ],
            [ w.degree(50), w.straight(-80) ],
            [ w.degree(125), w.straight(-100) ],
            [ w.degree(150), w.straight(-80) ],
            [ w.degree(175), w.straight(-60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(25), w.straight(60) ],
            [ w.degree(50), w.straight(80) ],
            [ w.degree(135), w.straight(100) ],
            [ w.degree(160), w.straight(80) ],
            [ w.degree(185), w.straight(60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],

            [ w.heading(-90.5), w.turn(left_or_right) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(50), w.straight(60) ],
            [ w.degree(100), w.straight(80) ],
            [ w.degree(150), w.straight(60) ],
            [ w.ms(200), w.brake() ],
            w.resetEncoder(),

            *expr.mf_mid(),
        )

        w.run( # RETURN TO ORIGINAL POSITION
            [ w.degree(25), w.straight(-60) ],
            [ w.degree(50), w.straight(-80) ],
            [ w.degree(80), w.straight(-100) ],
            [ w.degree(105), w.straight(-80) ],
            [ w.degree(130), w.straight(-60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
        )

    def gotoNext(self):
        next_color = self.queue[self.picked]
        next_dist = next_color - self.current
        sign = 0
        angle = -90.5

        if next_dist > 0:
            sign = 1
            angle = 0
            next_dist = ceil(next_dist)
        if next_dist < 0:
            sign = -1
            angle = -181
            next_dist = floor(next_dist)

        distance = abs(next_dist)

        # INITIALLY TURN AND ACCELS
        if distance != 0:
            if self.picked != 0:
                w.run(
                    [ w.heading(angle), w.turn() ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                    w.resetEncoder(),
                )

            if self.current == 1.5:
                w.run(
                    [ w.degree(75), w.straight(-60) ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                )

            w.runConcurrent(
                [ w.ms(100) ],
                *expr.mf_low()
            )

        # MOVE TO DESTINATION
        if distance == 4:
            w.run(
                [ w.blackReflection(20), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(250)), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(100)), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(100)), w.straight(80) ],
            )
        if distance == 3:
            w.run(
                [ w.blackReflection(20), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(250)), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(100)), w.straight(80) ],
            )
        if distance == 2:
            w.run(
                [ w.blackReflection(20), w.straight(100) ],
                [ w.all(w.blackReflection(20), w.ms(250)), w.straight(80) ],
            )
        if distance == 1:
            w.run(
                [ w.blackReflection(20), w.straight(80) ],
            )

        if distance != 0:
            if sign == -1:
                w.run(
                    [ w.ms(BRAKE_TIME), w.brake() ],
                    w.resetEncoder(),
                    [ w.degree(25), w.straight(60) ],
                    [ w.degree(110), w.straight(80) ],
                    [ w.degree(135), w.straight(60) ]
                )

            w.run(
                [ w.ms(BRAKE_TIME), w.brake() ],
                [ w.heading(-90.5), w.turn() ],
                [ w.ms(BRAKE_TIME), w.brake() ],
            )

        w.runConcurrent(
            [ w.ms(100) ],
            [ mf.degreeAt(-480, stable=1), mf.track(-480) ],
            *expr.mf_release()
        )

        w.run(
            w.resetEncoder(),
            [ w.degree(25), w.straight(-60) ],
            [ w.degree(75), w.straight(-80) ],
            [ w.blackReflection(20), w.straight(-60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
        )

        counter = self.pickmap[next_color]
        row = counter // 2

        deg_in_even = [ 235, 355, 475]
        deg_in_odd = [ 110, 220, 330 ]

        if counter % 2 == 0:
            w.run(
                [ w.degree(25), w.straight(60) ],
                [ w.degree(50), w.straight(80) ],
                [ w.degree(deg_in_even[row] - 50), w.straight(100) ],
                [ w.degree(deg_in_even[row] - 25), w.straight(80) ],
                [ w.degree(deg_in_even[row]), w.straight(60) ],
            )
  
        if counter % 2 != 0:
            w.run(
                [ w.degree(25), w.straight(60) ],
                [ w.degree(50), w.straight(80) ],
                [ w.degree(deg_in_odd[row] - 50), w.straight(100) ],
                [ w.degree(deg_in_odd[row] - 25), w.straight(80) ],
                [ w.degree(deg_in_odd[row]), w.straight(60) ],
                [ w.heading(-75), w.turn(PIVOT_RIGHT) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                w.resetEncoder(),

                [ w.degree(25), w.straight(60) ],
                [ w.degree(65), w.straight(80) ],
                [ w.degree(90), w.straight(60) ],
            )

        w.run(
            w.brake(),
            *expr.mf_pick(tolerance=5, stable=1),
            w.resetEncoder(),
        )

        if counter % 2 == 0:
            if row == 0:
                w.run(
                    [ w.degree(25), w.straight(-60) ],
                    [ w.degree(50), w.straight(-80) ],
                    [ w.degree(75), w.straight(-60) ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                )

            if row == 1:
                w.run(
                    [ w.degree(25), w.straight(-60) ],
                    [ w.degree(50), w.straight(-80) ],
                    [ w.degree(125), w.straight(-100) ],
                    [ w.degree(150), w.straight(-80) ],
                    [ w.degree(175), w.straight(-60) ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                )

            if row == 2:
                w.run(
                    [ w.degree(25), w.straight(-60) ],
                    [ w.degree(50), w.straight(-80) ],
                    [ w.degree(250), w.straight(-100) ],
                    [ w.degree(275), w.straight(-80) ],
                    [ w.degree(300), w.straight(-60) ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                )

        if counter % 2 != 0:
            w.run(
                [ w.degree(25), w.straight(-60) ],
                [ w.degree(55), w.straight(-80) ],
                [ w.degree(80), w.straight(-60) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                [ w.heading(-90.5), w.turn(PIVOT_RIGHT) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                w.resetEncoder()
            )

            if row == 0:
                pass

            if row == 1:
                w.run(
                    [ w.degree(25), w.straight(-60) ],
                    [ w.degree(50), w.straight(-80) ],
                    [ w.degree(75), w.straight(-60) ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                )

            if row == 2:
                w.run(
                    [ w.degree(25), w.straight(-60) ],
                    [ w.degree(50), w.straight(-80) ],
                    [ w.degree(125), w.straight(-100) ],
                    [ w.degree(150), w.straight(-80) ],
                    [ w.degree(175), w.straight(-60) ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                )

        self.pickmap[next_color] += 1
        self.current = next_color
        self.picked += 1

def sec3():
    w.runConcurrent(
        *expr.mb_set0d(),
        w.beep(800),
        [ w.ms(150) ],
        w.beep(0),
    )

    w.run(
        [ w.degree(100), w.straight(-50) ],
        [ w.degree(200), w.straight(-75) ],
        [ w.degree(1200), w.straight(-100) ],
        [ w.degree(1300), w.straight(-75) ],
    )

    w.runConcurrent(
        *expr.mb_up2mid(),
        w.beep(800),
        [ w.ms(150) ],
        w.beep(0),
    )

    w.run(
        [ w.degree(1500), w.straight(-50) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        [ w.untilStdin("w") ]
    )

def section2_main():
    set0()
    asd = MoveToDestinationEx5([ YELLOW, BLUE, YELLOW, YELLOW, GREEN, GREEN, GREEN, GREEN, GREEN, BLUE, BLUE, WHITE ])

    asd.gotoNext()
    asd.gotoNext()
    asd.keep(PIVOT_RIGHT)
    asd.gotoNext()
    asd.gotoNext()
    asd.keep(PIVOT_LEFT)
    asd.gotoNext()
    asd.gotoNext()

    asd.gotoMid()

    asd.gotoNext()
    asd.gotoNext()
    asd.keep(PIVOT_LEFT)
    asd.gotoNext()
    asd.gotoNext()
    asd.keep(PIVOT_RIGHT)
    asd.gotoNext()
    asd.gotoNext()

    asd.gotoTangrad()
    sec3()

if __name__ == "__main__":
    section2_main()
