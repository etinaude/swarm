import pygame
from specs import brick_size
from position import Position


class Brick:
    def __init__(self, x, y, layer, rotation, screen):
        self.pos = Position(x, y, layer, rotation)
        self.placed = False
        self.screen = screen
        self.has_adhesive = False

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
            adhesive_color = (255, 255, 255)
            adhesive_location = (
                self.pos.x + brick_size[0] // 2 - 2,
                self.pos.y + brick_size[1] // 2 - 2,
                4,
                4,
            )
            pygame.draw.rect(self.screen, adhesive_color, adhesive_location)

    def get_location(self):
        return self.pos.get_location()

    def __repr__(self):
        return "brick" + self.pos.__repr__()

    def __eq__(self, other):
        return self.pos == other.pos
