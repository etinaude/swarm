from brick import Brick
from position import Position
import pygame

speed = 10
size = [20, 20]

# states
# idle
# getting brick
# placing brick
# waiting
# glueing

# TODO: later
# getting battery
# deliver battery
# get glue


class Rover:
    def __init__(self, x, y, screen, gluers, brick_pile):
        self.pos = Position(x, y)
        self.screen = screen
        self.gluers = gluers

        self.state = "idle"
        self.battery = 100
        self.brick = None
        self.target = None
        self.brick_pile = brick_pile

    def draw(self):
        color = (20, 120, 20)
        if self.brick is not None:
            self.brick.pos = self.pos

            self.brick.draw()

        location = (self.pos.x, self.pos.y, size[0], size[1])
        pygame.draw.rect(self.screen, color, location)

    def __repr__(self):
        return f"Robot - ({self.pos.x}, {self.pos.y})"

    def make_move(self, canidate_bricks, house):
        decide_next_task(self)

        if self.state == "getting_brick":
            self.get_brick()

        if self.state == "glueing":
            self.glue_brick()

        if self.state == "placing_brick":
            canidate_bricks = self.place_brick(house, canidate_bricks)

        if self.state == "waiting":
            self.wait_for_gluer()

        return canidate_bricks

    def get_brick(self):
        self.target = self.brick_pile
        if self.pos.get_dist(self.target) > 10:
            self.move_towards_target()
        else:
            self.brick = Brick(self.pos.x, self.pos.y, 0, 0, self.screen)

    def place_brick(self, house, canidate_bricks):
        poses = list(map(lambda x: x.pos, canidate_bricks))
        self.target = self.pos.find_closest(poses)
        if self.target is None:
            self.state = "idle"
            return canidate_bricks

        if self.pos.get_dist(self.target) > 10:
            self.move_towards_target()
            return canidate_bricks

        print("Placing brick at", self.target.x, self.target.y)
        house.place_brick(self.target)
        self.brick = None
        canidate_bricks = list(filter(lambda x: x.pos != self.target, canidate_bricks))

        return canidate_bricks

    def get_target_distance(self):
        return self.pos.get_dist(self.target)

    def glue_brick(self):
        idle_gluers = list(filter(lambda x: x.status == "idle", self.gluers))
        poses = list(map(lambda x: x.pos, idle_gluers))
        self.target = self.pos.find_closest(poses)
        if self.target is None:
            self.state = "idle"
            return
        closest_gluer = list(filter(lambda x: x.pos == self.target, idle_gluers))[0]

        if self.pos.get_dist(self.target) > 10:
            self.move_towards_target()
        else:
            # hand brick to gluer
            closest_gluer.glue(self.brick)

            self.brick = None
            self.state = "waiting"

    def wait_for_gluer(self):
        # find gluers in 10 raduis with state 'ready"
        gluers = list(
            filter(
                lambda x: x.pos.get_dist(self.pos) < speed and x.status == "ready",
                self.gluers,
            )
        )
        if gluers == []:
            return

        gluer = gluers[0]

        self.target = gluer.pos
        self.move_towards_target()
        self.brick = gluer.give_brick()
        print("Got brick from gluer")
        print(self.brick)

    def move_towards_target(self):
        direction = self.pos.get_direction(self.target)
        distance = self.pos.get_dist(self.target)
        if distance < speed:
            self.pos = self.target
        else:
            self.pos.x += direction[0] * speed
            self.pos.y += direction[1] * speed


def decide_next_task(self):
    if self.state == "idle":
        self.state = "getting_brick"

    if self.brick is not None:
        if self.brick.has_adhesive:
            self.state = "placing_brick"
        else:
            self.state = "glueing"
