from brick import Brick
from position import Position
import pygame  # type: ignore

size = [20, 20]

# states
# idle
# getting brick
# placing brick


class Rover:
    def __init__(self, x, y, screen, sim_speed=1):
        self.pos = Position(x, y)
        self.screen = screen

        self.state = "idle"
        self.battery = 100
        self.brick = None
        self.target = None
        self.speed = 5 * sim_speed

    def draw(self):
        color = (120, 120, 20)
        if self.brick is not None:
            self.brick.pos = self.pos.copy_pos()

            self.brick.draw()

        location = (self.pos.x, self.pos.y, size[0], size[1])
        pygame.draw.rect(self.screen, color, location)

    def __repr__(self):
        return f"Drone - ({self.pos.x}, {self.pos.y})"

    def make_move(self, canidate_bricks, house):
        decide_next_task(self)
        self.battery -= 1

        if self.state == "getting_brick":
            self.get_brick()

        if self.state == "placing_brick":
            canidate_bricks = self.place_brick(house, canidate_bricks)

        return canidate_bricks

    def get_brick(self):
        self.target = self.brick_pile.copy_pos()
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

    def move_towards_target(self):
        direction = self.pos.get_direction(self.target)
        distance = self.pos.get_dist(self.target)
        if distance < self.speed:
            self.pos = self.target
        else:
            self.pos.x += direction[0] * self.speed
            self.pos.y += direction[1] * self.speed


def decide_next_task(self):
    if self.brick is None:
        self.state = "getting_brick"
    else:
        self.state = "placing_brick"
