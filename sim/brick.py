import uuid
import pygame  # type: ignore
from specs import brick_size
from position import Position


class Brick:
    def __init__(self, x, y, layer, rotation, screen):
        self.screen = screen
        self.pos = Position(x, y, layer, rotation)
        self.placed = False
        self.has_adhesive = False

        self.rover_claimed_by = None
        self.drone_claimed_by = None

        self.id = uuid.uuid4()

    def draw_outline(self):
        outline = (22, 22, 22)
        location = self.get_location()
        pygame.draw.rect(
            self.screen,
            outline,
            location,
            width=1,
        )

    def draw(self):
        color = (79, 39, 2) if self.pos.layer % 2 else (97, 47, 1)
        outline = (150, 73, 2)
        location = self.get_location()
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
            pos = self.get_location()
            x = pos[0] + pos[2] / 2
            y = pos[1] + pos[3] / 2
            pygame.draw.circle(self.screen, adhesive_color, (x, y), glue_dot_size)

    def get_location(self):
        return self.pos.get_location()

    def __repr__(self):
        return "brick" + self.pos.__repr__()

    def __eq__(self, other):
        return self.pos == other.pos
