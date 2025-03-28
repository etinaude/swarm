from position import Position
import pygame
from specs import brick_size

glue_time = 5 * 1000
size = [20, 20]

# idle
# Working
# Complete
# Power Error
# Adhesive Error
# no glue


class Gluer:
    def __init__(self, x, y, screen, sim_speed=10):
        self.pos = Position(x, y)
        self.screen = screen
        self.sim_speed = sim_speed

        self.status = "idle"
        self.glue_left = 100
        self.brick = None

    def draw(self):
        color = (120, 20, 120)
        if self.brick is not None:
            brick_color = (79, 39, 2)
            brick_location = (
                self.pos.x,
                self.pos.y,
                brick_size[0],
                brick_size[1],
            )
            pygame.draw.rect(self.screen, brick_color, brick_location)

        location = (self.pos.x, self.pos.y, size[0], size[1])
        pygame.draw.rect(self.screen, color, location)

    def glue(self, brick):
        if self.status != "idle":
            return

        self.status = "working"
        self.time_stamp = pygame.time.get_ticks()
        self.brick = brick

    def update(self):
        if self.status != "working":
            return

        if pygame.time.get_ticks() - self.time_stamp < glue_time:
            return

        self.glue_left -= 1
        self.brick.has_adhesive = True

        if self.glue_left == 0:
            self.status = "out_of_glue"
        else:
            self.status = "ready"

    def give_brick(self):
        brick = self.brick
        self.brick = None
        self.status = "idle"

        return brick

    def __repr__(self):
        return f"Gluer - ({self.pos.x}, {self.pos.y})"
