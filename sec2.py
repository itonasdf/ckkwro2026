from wro2026corev2 import expr
from math import ceil, floor
from random import randint

husky = expr.husky
w = expr.w
mf = expr.mf
mb = expr.mb

YELLOW = expr.YELLOW
BLUE = expr.BLUE
GREEN = expr.GREEN
WHITE = expr.WHITE

YELLOWCLR = expr.YELLOWCLR
GREENCLR = expr.GREENCLR
BLUECLR = expr.BLUECLR
WHITECLR = expr.WHITECLR

color_var = expr.color_var


PIVOT_LEFT = expr.PIVOT_LEFT
PIVOT_RIGHT = expr.PIVOT_RIGHT
BRAKE_TIME = expr.BRAKE_TIME




def set0():
    w.runConcurrent(
        *expr.mf_set0d(),
    )

    w.run(
        [ w.ms(650), w.moveTank(-40, -40) ],
        [ w.ms(100), w.brake() ],
        w.resetImu(),
        w.resetEncoder(),
        mf.move(-30),

        w.heading(40),
        [ w.degree(160), w.straight(80) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        w.resetEncoder(),
        mf.move(-20),

        w.heading(0),
        [ w.degree(200), w.straight(80) ],
        mf.move(20),
    )

    w.runConcurrent(
        *expr.mb_set0d()
    )

def gotoNext(dist, side, pickup = False, center = False): # MOSAIC = -1 CEMENT = 1
    sign = 0
    angle = 90.5 * side

    if dist > 0:
        sign = 1
        angle = 0
        dist = ceil(dist)
    if dist < 0:
        sign = -1
        angle = 181 * side
        dist = floor(dist)

    distance = abs(dist)

    # INITIALLY TURN AND ACCELS
    if distance != 0:
        w.run(
            [ w.heading(angle), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
        )

        if pickup:
            w.runConcurrent(
                [ w.ms(50) ],
                [ mf.degreeAt(-220, tolerance=1, stable=1), mf.track(-220) ],
                mf.move(20),
            )

    # MOVE TO DESTINATION
    if distance == 4:
        w.run(
            [ w.blackReflection(20), w.straight(80) ],

            [ w.whiteReflection(70), w.straight(100) ],
            [ w.blackReflection(20), w.straight(100) ],

            [ w.whiteReflection(70), w.straight(100) ],
            [ w.blackReflection(20), w.straight(100) ],

            [ w.whiteReflection(70), w.straight(100) ],
            [ w.blackReflection(20), w.straight(80) ],
        )
    if distance == 3:
        w.run(
            [ w.blackReflection(20), w.straight(80) ],

            [ w.whiteReflection(70), w.straight(100) ],
            [ w.blackReflection(20), w.straight(100) ],

            [ w.whiteReflection(70), w.straight(100) ],
            [ w.blackReflection(20), w.straight(80) ],
        )
    if distance == 2:
        w.run(
            [ w.blackReflection(20), w.straight(80) ],

            [ w.whiteReflection(70), w.straight(100) ],
            [ w.blackReflection(20), w.straight(80) ],
        )
    if distance == 1:
        w.run(
            [ w.blackReflection(20), w.straight(80) ],
        )

    if distance != 0:
        if center:
            w.run(
                [ w.ms(BRAKE_TIME), w.brake() ],
                w.resetEncoder(),
                [ w.degree(65), w.straight(60) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                w.resetEncoder(),
            )
        else:
            if sign == -1:
                w.run(
                    [ w.ms(BRAKE_TIME), w.brake() ],
                    w.resetEncoder(),
                    [ w.degree(50), w.straight(60) ],
                    [ w.degree(75), w.straight(80) ],
                    [ w.degree(110 if distance == 1 else 105), w.straight(60) ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                    w.resetEncoder(),
                )
            else:
                w.run(
                    [ w.ms(50), w.brake() ],
                    w.resetEncoder(),
                )

def keep(left_or_right, current_heading, keep_pos):
    if abs(w._hub.imu.heading() - current_heading) > 10:
        w.run(
            [ w.heading(current_heading), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
        )
        
    w.run(
        [ w.heading(current_heading - 25 if left_or_right == PIVOT_RIGHT else current_heading + 25), w.turn(left_or_right) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        w.resetEncoder()
    )

    w.runConcurrent(
        [ mf.degreeAt(-585, tolerance=2, stable=1), mf.track(-585) ],
        [ w.degree(100) ],
        *expr.mf_set0d(),
        mf.move(100)
    )

    w.run(
        [ w.degree(25), w.straight(-60) ],
        [ w.degree(50), w.straight(-80) ],
        [ w.degree(90), w.straight(-100) ],
        [ w.degree(115), w.straight(-80) ],
        [ w.degree(140), w.straight(-60) ],
        [ w.ms(350), w.brake() ],
        w.resetEncoder(),

        [ w.degree(25), w.straight(60) ],
        [ w.degree(50), w.straight(80) ],
        [ w.degree(90), w.straight(100) ],
        [ w.degree(115), w.straight(80) ],
        [ w.degree(140), w.straight(60) ],
        [ w.ms(BRAKE_TIME), w.brake() ],

        [ w.heading(current_heading), w.turn(left_or_right) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        w.resetEncoder(),
    )
    
    if keep_pos:
        w.runConcurrent(
            [ w.degree(50) ],
            [ mf.degreeAt(-220, tolerance=1, stable=1), mf.track(-220, kp=100000, ki=50000, kd=20000) ],
            mf.move(20)
        )
    
    if not keep_pos:
        w.run(
            [ w.degree(50), w.straight(60) ],
            [ w.degree(110), w.straight(60) ],
            mf.track(-140),
            [ w.degree(150), w.straight(60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
        )
        
        w.runConcurrent(
            [ w.degree(50) ],
            [ mf.degreeAt(-200, tolerance=2, stable=1), mf.track(-200) ],
            mf.move(20),
        )

        w.run( # RETURN TO ORIGINAL POSITION
            [ w.degree(25), w.straight(-60) ],
            [ w.degree(90), w.straight(-80) ],
            [ w.degree(115), w.straight(-60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
        )

class MoveToDestinationEx5:
    def __init__(self, pick_queue: list[int]):
        color_list = [ [], [], [], [] ]
        for i, v in enumerate(pick_queue):
            color_list[v].append(i)

        for i, v in enumerate(color_list):
            while len(color_list[i]) > 6:
                index = color_list[i].pop()
                for j, w in enumerate(color_list):
                    if len(color_list[j]) < 6:
                        color_list[j].append(index)
                        pick_queue[index] = j
                        break

        self.picked = 0
        self.picking = 0
        self.current = -1
        self.queue = pick_queue
        self.pickmap = [ [0, 0], [0, 0], [0, 0], [0, 0] ] #L, R

    def pickUp(self, pick_high = False):
        middle = 1.5
        prev = self.queue[self.picked-1]
        next_dist = middle - prev

        if next_dist > 0:
            sign = 1
            angle = 0
        if next_dist < 0:
            sign = -1
            angle = -181

        w.run(
            w.resetEncoder(),
            [ w.degree(25), w.straight(-60) ],
            [ w.degree(75), w.straight(-80) ],
            mf.track(-260),
            [ w.blackReflection(20), w.straight(-60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            [ w.degree(30), w.straight(60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            [ w.heading(angle), w.turn() ],
        )

        if prev == YELLOW or prev == BLUE:
            w.run(
                [ w.blackReflection(20), w.straight(-60 if prev == BLUE and self.picking == 1 else 80) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                w.resetEncoder(),
                [ w.degree(80 if prev == BLUE and self.picking == 1 else 70), w.straight(60) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
            )
        if prev == GREEN or prev == WHITE:
            w.run(
                [ w.blackReflection(20), w.straight(-60 if prev == GREEN and self.picking == 0 else 80) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                w.resetEncoder(),
                [ w.degree(70 if prev == GREEN and self.picking == 0 else 55), w.straight(60) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
            )

        w.run(
            [ w.heading(90.5 if sign == 1 else -272), w.turn() ],
            mf.track(-585),
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            [ w.degree(250), w.tagline(60, 15, PIVOT_LEFT) ],

            mf.track(-510),
            [ w.ms(BRAKE_TIME), w.brake() ],
            [ mf.stable(stable=15), mf.move(100) ],
            w.resetEncoder(),
            mf.resetEncoder(),
        )

        w.runConcurrent(
            [ w.degree(25) ],
            [ w.degree(100), mf.move(-30) ],
            [ mf.degree(125), mf.move(-75) ],
            [ mf.degree(300), mf.move(-30) ],
            mf.track(-445),
        )

        w.run(
            [ w.degree(50), w.straight(-60) ],
            [ w.degree(100), w.straight(-80) ],
            [ w.degree(250), w.straight(-100) ],
            [ w.degree(300), w.straight(-80) ],
            [ w.degree(350), w.straight(-60) ],
            [ mf.degree(360), w.brake() ],
            w.resetEncoder(),
        )

        w.runConcurrent(
            [ w.degree(450) ],
            [ w.degree(500), mf.move(50) ],
            [ w.degree(700 if pick_high else 600), mf.move(100) ],
            [ w.degree(800 if pick_high else 700), mf.move(75) ],
            mf.move(30),
        )

        w.run(
            [ w.degree(100), w.straight(60) ],
            [ w.degree(250), w.straight(80) ],
            [ w.degree(625), w.tagline(60, 15, PIVOT_LEFT) ],
        )

        self.current = middle
        return sign

    def gotoMid(self):
        sign = self.pickUp()
        w.run(
            [ w.degree(850), w.straight(60) ],
            [ w.ms(400), w.moveTank(45, 45) ],

            mf.move(-15),
            [ w.ms(150), w.brake() ],
            w.resetEncoder(),
            w.resetImu(90.5),

            [ mf.degreeAt(-400), mf.track(-400) ],
            [ mf.degree(435), mf.move(-75) ],
            [ mf.degree(450), mf.move(-50) ],
            [ mf.degreeAt(-528), mf.track(-528) ],
            [ w.ms(80), w.moveTank(-40, 40) ],
            [ w.ms(140), w.moveTank(40, -40) ],
            [ w.heading(90.5), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(50), w.straight(-60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ mf.degreeAt(-400), mf.track(-400) ],
            [ w.degree(35), w.straight(-60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ mf.degreeAt(-400), mf.track(-400) ],
            [ mf.degree(435), mf.move(-75) ],
            [ mf.degree(450), mf.move(-50) ],
            [ mf.degreeAt(-524), mf.track(-524) ], ###############
            [ w.ms(80), w.moveTank(-40, 40) ],
            [ w.ms(140), w.moveTank(40, -40) ],
            [ w.heading(90.5), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(50), w.straight(-60) ],
            [ w.degree(100), w.straight(-80) ],
            [ w.degree(250), w.straight(-100) ],
        )

        w.runConcurrent(
            *expr.mf_set0d(),
            mf.move(-50),
            [ w.ms(250) ],
            mf.move(-25),
            [ w.ms(220) ],
            mf.move(20)
        )

        w.run(
            [ w.degree(665), w.straight(-100) ],
            [ w.degree(715), w.straight(-80) ],
            [ w.degree(765), w.straight(-60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
        )

    def gotoTangrad(self, side = 1): #YELLOW: 1 WHITE: -1
        sign = self.pickUp(pick_high=True)

        w.run(
            [ w.degree(850), w.straight(60) ],
            [ w.ms(400), w.moveTank(45, 45) ],
            [ w.ms(100), w.brake() ],
            w.resetEncoder(),
            w.resetImu(90.5),

            [ w.degree(50), w.straight(-60) ],
            [ w.degree(175), w.straight(-80) ],
            [ w.degree(225), w.straight(-60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],

            [ w.heading(135 if side == 1 else 48), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(50), w.straight(60) ],
            [ w.degree(125), w.straight(80) ],
            [ w.colorReflection(YELLOWCLR if side == 1 else WHITECLR), w.straight(80) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(60), w.straight(60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.heading(90.5),
            w.resetEncoder(),

            [ w.degree(50), w.straight(60) ],
            [ w.degree(250), w.straight(80) ],
            [ w.negate(w.colorReflection(YELLOWCLR if side == 1 else WHITECLR)), w.straight(80) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(25), w.straight(60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            w.heading(45 if side == 1 else 140),

            [ w.degree(50), w.straight(60) ],
            [ w.degree(150), w.straight(80) ],
            [ w.degree(200), w.straight(60) ],
            [ w.blackReflection(20), w.straight(80) ],
            [ w.whiteReflection(70), w.straight(80) ],
            [ w.blackReflection(20), w.straight(80) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(60), w.straight(60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            [ w.heading(-90.5 if side == 1 else 272), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
            mf.move(-15),

            [ w.degree(150), w.straight(60) ],
            [ w.ms(400), w.moveTank(45, 45) ],
            [ w.ms(150), w.brake() ],
            w.resetEncoder(),
            w.resetImu(90.5),

            [ mf.degreeAt(-400), mf.track(-400) ],
            [ mf.degree(435), mf.move(-75) ],
            [ mf.degree(450), mf.move(-50) ],
            [ mf.degreeAt(-528), mf.track(-528) ],
            [ w.ms(80), w.moveTank(-40, 40) ],
            [ w.ms(140), w.moveTank(40, -40) ],
            [ w.heading(90.5), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ w.degree(50), w.straight(-60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ mf.degreeAt(-400), mf.track(-400) ],
            [ w.degree(35), w.straight(-60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),

            [ mf.degreeAt(-400), mf.track(-400) ],
            [ mf.degree(435), mf.move(-75) ],
            [ mf.degree(450), mf.move(-50) ],
            [ mf.degreeAt(-524), mf.track(-524) ],
            [ w.ms(80), w.moveTank(-40, 40) ],
            [ w.ms(140), w.moveTank(40, -40) ],
            [ w.heading(90.5), w.turn() ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
        )
        
    def peek_next(self):
        return self.queue[self.picked], self.queue[self.picked] - self.current

    def keep_info(self):
        _, next_dist = self.peek_next()
        angle = -90.5
        keep_pos = False
        if next_dist > 0:
            angle = 0
            keep_pos = True
        if next_dist < 0:
            angle = -181
            keep_pos = True
            
        return angle, keep_pos   

    def gotoNext(self, initial = False):
        next_color, next_dist = self.peek_next()
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
            if abs(w._hub.imu.heading() - angle) > 10:
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

            if not initial:
                w.runConcurrent(
                    [ w.ms(50) ],
                    [ mf.degreeAt(-220, tolerance=1, stable=1), mf.track(-220, kp=200000, ki=100000, kd=10000) ],
                    mf.move(20),
                )

        # MOVE TO DESTINATION
        if distance == 4:
            w.run(
                [ w.blackReflection(20), w.straight(80) ],

                [ w.whiteReflection(70), w.straight(100) ],
                [ w.blackReflection(20), w.straight(100) ],

                [ w.whiteReflection(70), w.straight(100) ],
                [ w.blackReflection(20), w.straight(100) ],
  
                [ w.whiteReflection(70), w.straight(100) ],
                [ w.blackReflection(20), w.straight(80) ],
            )
        if distance == 3:
            w.run(
                [ w.blackReflection(20), w.straight(80) ],

                [ w.whiteReflection(70), w.straight(100) ],
                [ w.blackReflection(20), w.straight(100) ],

                [ w.whiteReflection(70), w.straight(100) ],
                [ w.blackReflection(20), w.straight(80) ],
            )
        if distance == 2:
            w.run(
                [ w.blackReflection(20), w.straight(80) ],

                [ w.whiteReflection(70), w.straight(100) ],
                [ w.blackReflection(20), w.straight(80) ],
            )
        if distance == 1:
            w.run(
                [ w.blackReflection(20), w.straight(80) ],
            )

        counter = self.pickmap[next_color]
        
        if distance != 0:
            if (sign == -1 and counter[1] > counter[0]) or (sign == 1 and counter[0] > counter[1]) and next_color != WHITE:
                w.run(
                    [ w.ms(BRAKE_TIME), w.brake() ],
                    w.resetEncoder(),
                    [ w.degree(50), w.straight(60) ],
                    [ w.degree(75), w.straight(80) ],
                    [ w.degree(115 if distance == 1 else 112), w.straight(60) ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                    [ w.heading(-90.5), w.turn() ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                )
                
                self.picking = 0 if sign == -1 else 1
            else:
                w.run(
                    [ w.ms(100), w.brake() ],
                    [ w.heading(-90.5), w.turn() ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                )
                
                self.picking = 1 if sign == -1 else 0

        w.runConcurrent(
            [ w.ms(100) ],
            [ mf.degreeAt(-480, tolerance=5, stable=1), mf.track(-480) ],
            mf.track(-585),
        )

        w.run(
            w.resetEncoder(),
            [ w.degree(25), w.straight(-60) ],
            [ w.degree(75), w.straight(-80) ],
            [ w.blackReflection(20), w.straight(-60) ],
            [ w.ms(BRAKE_TIME), w.brake() ],
            w.resetEncoder(),
        )

        cntr = (counter[0]+counter[1])
        inverse_picking = 1 if self.picking == 0 else 0
        row = cntr // 2
        turn_next = counter[self.picking] > counter[inverse_picking]

        if not turn_next:
            if row == 0:
                w.run(
                    [ w.degree(50), w.straight(60) ],
                    [ w.degree(100), w.straight(80) ],
                    [ w.degree(160), w.straight(100) ],
                    [ w.degree(210), w.straight(80) ],
                    [ w.degree(260), w.straight(60) ],
                )
            if row == 1:
                w.run(
                    [ w.degree(50), w.straight(60) ],
                    [ w.degree(100), w.straight(80) ],
                    [ w.degree(280), w.straight(100) ],
                    [ w.degree(330), w.straight(80) ],
                    [ w.degree(380), w.straight(60) ],
                )
            if row == 2:
                w.run(
                    [ w.degree(50), w.straight(60) ],
                    [ w.degree(100), w.straight(80) ],
                    [ w.degree(400), w.straight(100) ],
                    [ w.degree(450), w.straight(80) ],
                    [ w.degree(500), w.straight(60) ],
                )

        if turn_next:
            if row == 0:
                w.run(
                    [ w.degree(50), w.straight(60) ],
                    [ w.degree(80), w.straight(80) ],
                    [ w.degree(130), w.straight(60) ],
                )
            if row == 1:
                w.run(
                    [ w.degree(50), w.straight(60) ],
                    [ w.degree(100), w.straight(80) ],
                    [ w.degree(150), w.straight(100) ],
                    [ w.degree(200), w.straight(80) ],
                    [ w.degree(250), w.straight(60) ],
                )
            if row == 2:
                w.run(
                    [ w.degree(50), w.straight(60) ],
                    [ w.degree(100), w.straight(80) ],
                    [ w.degree(270), w.straight(100) ],
                    [ w.degree(320), w.straight(80) ],
                    [ w.degree(370), w.straight(60) ],
                )

            w.run(
                [ w.heading(-74 if self.picking == 0 else -106), w.turn(PIVOT_RIGHT if self.picking == 0 else PIVOT_LEFT) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                w.resetEncoder(),
                [ w.degree(80), w.straight(60) ],
            )

        w.run(
            w.brake(),
            [ mf.degreeAt(-510, tolerance=5, stable=1), mf.track(-510) ],
            w.resetEncoder(),
        )

        if not turn_next:
            if row == 0:
                w.run(
                    [ w.degree(25), w.straight(-60) ],
                    [ w.degree(45), w.straight(-80) ],
                    [ w.degree(70), w.straight(-60) ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                )

            if row == 1:
                w.run(
                    [ w.degree(25), w.straight(-60) ],
                    [ w.degree(50), w.straight(-80) ],
                    [ w.degree(130), w.straight(-100) ],
                    [ w.degree(155), w.straight(-80) ],
                    [ w.degree(180), w.straight(-60) ],
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

        if turn_next:
            w.run(
                [ w.degree(60), w.straight(-60) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                [ w.heading(-90.5), w.turn(PIVOT_RIGHT if self.picking == 0 else PIVOT_LEFT) ],
                [ w.ms(BRAKE_TIME), w.brake() ],
                w.resetEncoder()
            )

            if row == 1:
                w.run(
                    [ w.degree(25), w.straight(-60) ],
                    [ w.degree(75), w.straight(-80) ],
                    [ w.degree(100), w.straight(-60) ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                )

            if row == 2:
                w.run(
                    [ w.degree(25), w.straight(-60) ],
                    [ w.degree(50), w.straight(-80) ],
                    [ w.degree(125), w.straight(-100) ],
                    [ w.degree(175), w.straight(-80) ],
                    [ w.degree(200), w.straight(-60) ],
                    [ w.ms(BRAKE_TIME), w.brake() ],
                )

        if turn_next and next_color == WHITE:
            self.pickmap[WHITE][1] += 1
        else:
            self.pickmap[next_color][self.picking if not turn_next else inverse_picking] += 1
        self.current = next_color
        self.picked += 1

def sec3():
    w.runConcurrent(
        *expr.mb_set0d(),
        mb.track(-205),
        w.beep(800),
        [ w.ms(150) ],
        w.beep(0),
        *expr.mf_set0d(),
    )

    w.run(
        [ w.degree(100), w.straight(-60) ],
        [ w.degree(200), w.straight(-80) ],
        [ w.degree(1200), w.straight(-100) ],
        [ w.degree(1325), w.straight(-80) ],
        mb.move(30),
        [ w.degree(1425), w.straight(-60) ],
        mb.move(45),
        [ w.ms(300), w.brake() ],
        w.resetEncoder(),
        mb.track(-50),

        [ w.degree(100), w.straight(-80) ],
        [ w.ms(400), w.moveTank(-40, -40) ],
        [ w.ms(100), w.brake() ],
        w.resetEncoder(),
        w.resetImu(),

        [ w.degree(100), w.straight(80) ],
        [ w.ms(BRAKE_TIME), w.brake() ],
        w.resetEncoder(),
        w.heading(-181),

        [ w.degree(400), w.straight(-80) ],
        [ w.degree(500), w.straight(-100) ],
        mb.track(-150),
        [ w.degree(1200), w.straight(-100) ],
        mb.track(-50),

        [ w.degree(1350), w.straight(-100) ],
        [ w.degree(1450), w.straight(-80) ],
        [ w.ms(400), w.moveTank(-40, -40) ],
        [ w.ms(100), w.brake() ],
        w.resetEncoder(),

        [ w.degree(65), w.straight(60) ],
        mb.track(-165),
        [ w.degree(200), w.straight(80) ],
        [ w.degree(400), w.straight(100) ],
        mb.track(-50),
        [ w.ms(300), w.brake() ], w.resetEncoder(),
    )

def section2_main():
    set0()
    color = expr.color_var
    asd = MoveToDestinationEx5([
        color[0][0], color[0][1], color[2][0], color[2][1], 
        color[1][0], color[1][1], color[0][3], color[0][2], 
        color[2][3], color[2][2], color[1][3], color[1][2], 
    ])

    asd = MoveToDestinationEx5([
        randint(0,3),randint(0,3),randint(0,3),randint(0,3),randint(0,3),randint(0,3),randint(0,3),randint(0,3),randint(0,3),randint(0,3),randint(0,3),randint(0,3),
    ])

    asd.gotoNext(initial=True)
    asd.gotoNext()
    angle, keep_pos = asd.keep_info()
    keep(PIVOT_RIGHT, angle, keep_pos)
    asd.gotoNext(initial=True)
    asd.gotoNext()
    angle, keep_pos = asd.keep_info()
    keep(PIVOT_LEFT, angle, keep_pos)
    asd.gotoNext(initial=True)
    asd.gotoNext()

    asd.gotoMid()

    asd.gotoNext(initial=True)
    asd.gotoNext()
    angle, keep_pos = asd.keep_info()
    keep(PIVOT_LEFT, angle, keep_pos)
    asd.gotoNext(initial=True)
    asd.gotoNext()
    angle, keep_pos = asd.keep_info()
    keep(PIVOT_RIGHT, angle, keep_pos)
    asd.gotoNext(initial=True)
    asd.gotoNext()

    asd.gotoTangrad(side=-1)
    sec3()

if __name__ == "__main__":
    print("mission2_start")
    section2_main()
    print("mission2_end")
