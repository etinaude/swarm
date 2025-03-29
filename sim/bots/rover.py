import uuid
from brick import Brick
from paths import draw_lineString, find_path
from position import Position
import pygame  # type: ignore
from shapely.geometry import LineString

size = [30, 30]

# ~~~ STATES ~~~
# idle
# getting brick
# glueing
# waiting
# placing brick

# TODO: later
# getting battery
# deliver battery
# get glue


class Rover:
    def __init__(self, x, y, state):
        self.pos = Position(x, y)
        self.global_state = state

        self.state = "idle"
        self.battery = 100
        self.brick = None
        self.target = None
        self.speed = 10 * state.sim_speed
        self.id = uuid.uuid4()
        self.path = None

    def draw(self):
        color = (20, 120, 20)
        if self.brick is not None:
            self.brick.pos = self.pos.copy_pos()

            self.brick.draw()

        location = (self.pos.x, self.pos.y, size[0], size[1])
        pygame.draw.rect(self.global_state.screen, color, location)

        if self.path is None or len(self.path) < 2:
            return

        line = LineString(self.path)
        draw_lineString(line, self.global_state.screen)

    def __repr__(self):
        return f"Rover - ({self.pos.x}, {self.pos.y})"

    def make_move(self):
        self.decide_next_task()
        self.battery -= 1

        if self.state == "getting_brick":
            self.get_brick()

        if self.state == "glueing":
            self.glue_brick()

        if self.state == "placing_brick":
            self.place_brick()

        if self.state == "waiting":
            self.wait_for_gluer()

    def get_brick(self):
        if self.path is None:
            self.path = find_path(
                self.pos, self.global_state.brick_pile, self.speed, self.global_state
            )

        arrived = self.move_along_path()
        if arrived:
            self.brick = Brick(self.pos.x, self.pos.y, 0, 0, self.global_state.screen)
            self.path = None

    def place_brick(self):
        canidates = self.global_state.house.get_rover_bricks()
        if canidates == []:
            print("No bricks to place")
            self.state = "idle"
            return
        print(f"Canidates: {canidates}")
        closest = self.pos.find_closest(canidates).pos.copy_pos()

        if closest is not self.target:
            self.path = None
            self.target = closest

        if self.path is None:
            self.path = find_path(self.pos, self.target, self.speed, self.global_state)

        arrived = self.move_along_path()
        if arrived:
            self.brick.pos = self.pos.copy_pos()
            self.global_state.loose_bricks.append(self.brick)
            self.brick = None
            self.path = None
            self.state = "idle"
            return

    def glue_brick(self):
        idle_gluers = list(
            filter(lambda x: x.status == "idle", self.global_state.gluers)
        )
        target = self.pos.find_closest(idle_gluers)

        if target is None:
            self.state = "idle"
            print("No gluer found")
            return

        if self.target is not target:
            self.path = None
            self.target = target

        if self.path is None:
            self.path = find_path(self.pos, self.target, self.speed, self.global_state)

        arrived = self.move_along_path()
        if arrived:
            for glue in self.global_state.gluers:
                if glue.pos == self.target:
                    glue.glue(self.brick)
                    self.brick = None
                    self.state = "waiting"
                    break

    def wait_for_gluer(self):
        # find gluers in raduis with state 'ready"
        gluers = list(
            filter(
                lambda x: x.pos.get_dist(self.pos) < self.speed and x.status == "ready",
                self.global_state.gluers,
            )
        )
        if gluers == []:
            return

        gluer = gluers[0]

        self.target = gluer.pos.copy_pos()
        self.pos.goto(gluer.pos.x, gluer.pos.y)
        self.brick = gluer.give_brick()
        self.target = None
        self.path = None

    def decide_next_task(self):
        if self.brick is None and self.state != "waiting":
            self.state = "getting_brick"

        if self.brick is not None:
            if self.brick.has_adhesive:
                self.state = "placing_brick"
            else:
                self.state = "glueing"

    def move_along_path(self):
        for i in range(self.speed):
            if self.path is None or len(self.path) < 1:
                print("Arrived")
                self.path = None
                return True
            pos = self.path.pop(0)
        self.pos.goto(pos[0], pos[1])
        print(f"Moving to {pos[0]}, {pos[1]}")

        return False
