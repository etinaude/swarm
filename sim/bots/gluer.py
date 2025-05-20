import pygame  # type: ignore
from shapely.geometry import Polygon, Point


size = [20, 20]

# idle
# Working
# Complete
# Power Error
# Adhesive Error
# no glue


class Gluer:
    def __init__(self, x, y, sim_speed=1):
        self.pos = Point(x, y)
        # self.screen = screen

        self.glue_time = (3 * 1000) / sim_speed
        self.state = "idle"
        self.glue_left = 100
        self.brick = None

    def draw(self):
        color = (120, 20, 120)
        if self.brick is not None:
            self.brick.pos = Point(self.pos.x, self.pos.y)
            self.brick.draw()

        location = (self.pos.x, self.pos.y, size[0], size[1])
        # pygame.draw.rect(self.screen, color, location)

    def glue(self, brick):
        if self.state != "idle":
            return

        self.state = "working"
        self.time_stamp = pygame.time.get_ticks()
        self.brick = brick

    def update(self):
        if self.state != "working":
            return

        if pygame.time.get_ticks() - self.time_stamp < self.glue_time:
            return

        self.glue_left -= 1
        self.brick.has_adhesive = True

        if self.glue_left == 0:
            self.state = "out_of_glue"
        else:
            self.state = "ready"

    def give_brick(self):
        brick = self.brick
        self.brick = None
        self.state = "idle"

        return brick

    def __repr__(self):
        return f"Gluer - ({self.pos.x}, {self.pos.y})"
