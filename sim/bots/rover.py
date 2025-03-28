import random
import uuid
from brick import Brick
from position import Position
import pygame  # type: ignore

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

    def draw(self):
        color = (20, 120, 20)
        if self.brick is not None:
            self.brick.pos = self.pos.copy_pos()

            self.brick.draw()

        location = (self.pos.x, self.pos.y, size[0], size[1])
        pygame.draw.rect(self.global_state.screen, color, location)

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
        self.target = self.global_state.brick_pile.copy_pos()
        if self.pos.get_dist(self.target) > self.speed:
            self.move_towards_target()
        else:
            self.brick = Brick(self.pos.x, self.pos.y, 0, 0, self.global_state.screen)

    def place_brick(self):

        if self.target is None:
            self.state = "idle"

        if self.pos.get_dist(self.target) > self.speed + 60:
            self.move_towards_target()
        else:
            self.brick.pos = self.pos.copy_pos()
            self.global_state.loose_bricks.append(self.brick)
            self.brick = None

    def get_target_distance(self):
        return self.pos.get_dist(self.target)

    def glue_brick(self):
        idle_gluers = list(
            filter(lambda x: x.status == "idle", self.global_state.gluers)
        )
        poses = list(map(lambda x: x.pos, idle_gluers))
        self.target = self.pos.find_closest(poses)
        if self.target is None:
            self.state = "idle"
            return
        closest_gluer = list(filter(lambda x: x.pos == self.target, idle_gluers))[0]

        if self.pos.get_dist(self.target) > self.speed:
            self.move_towards_target()
        else:
            closest_gluer.glue(self.brick)

            self.brick = None
            self.state = "waiting"

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
        self.move_towards_target()
        self.brick = gluer.give_brick()
        self.target = None

    def move_towards_target(self):
        direction = self.pos.get_direction(self.target)
        distance = self.pos.get_dist(self.target)
        if distance < self.speed:
            self.pos = self.target
        else:
            self.pos.x += direction[0] * self.speed
            self.pos.y += direction[1] * self.speed

    def decide_next_task(self):
        if self.brick is None and self.state != "waiting":
            self.state = "getting_brick"

        if self.brick is not None:
            if self.brick.has_adhesive:
                self.state = "placing_brick"
                if self.target is None:
                    self.target = random.choice(
                        self.global_state.house.get_rover_bricks()
                    ).pos.copy_pos()

            else:
                self.state = "glueing"
