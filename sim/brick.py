import uuid
import pygame  # type: ignore
from specs import brick_size
from shapely.geometry import Polygon, Point


class Brick:
    def __init__(self, x, y, layer, rotation, screen):
        self.screen = screen
        self.pos = Point(x, y)
        self.layer = layer
        self.rotation = rotation
        self.placed = False
        self.has_adhesive = False

        self.rover_claimed_by = None
        self.drone_claimed_by = None

        self.id = uuid.uuid4()

    def draw_outline(self):
        outline = (22, 22, 22)
        if self.rotation == 0:
            location = (self.pos.x, self.pos.y, brick_size[0], brick_size[1])
        else:
            location = (self.pos.x, self.pos.y, brick_size[1], brick_size[0]) 
        
        pygame.draw.rect(
            self.screen,
            outline,
            location,
            width=1,
        )

    def draw(self):
        color = (79, 39, 2) if self.layer % 2 else (97, 47, 1)
        outline = (150, 73, 2)

        if self.rotation == 0:
            location = (self.pos.x, self.pos.y, brick_size[0], brick_size[1])
        else:
            location = (self.pos.x, self.pos.y, brick_size[1], brick_size[0]) 

        pygame.draw.rect(self.screen, color, location)
        pygame.draw.rect(
            self.screen,
            outline,
            location,
            width=1,
        )

        if self.has_adhesive:
            glue_dot_size = 5
            adhesive_color = (255, 255, 255)
            pos = self.pos.coords
            x = pos[0] + pos[2] / 2
            y = pos[1] + pos[3] / 2
            pygame.draw.circle(self.screen, adhesive_color, (x, y), glue_dot_size)

    def __repr__(self):
        return "brick" + self.pos.__repr__()

    def __eq__(self, other):
        return self.pos == other.pos
