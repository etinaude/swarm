import pygame
from specs import brick_size


class Brick:
    def __init__(self, x, y, layer, rotation, screen):
        self.x = x
        self.y = y
        self.layer = layer
        self.rotation = rotation
        self.placed = False
        self.screen = screen

    def draw(self):
        color = (79, 39, 2) if self.layer % 2 else (97, 47, 1)
        outline = (150, 73, 2)
        location = self.get_location()
        pygame.draw.rect(self.screen, color, location)
        pygame.draw.rect(
            self.screen,
            outline,
            location,
            width=1,
        )

    def get_location(self):
        location = (self.x, self.y, brick_size[0], brick_size[1])
        if self.rotation == 90:
            location = (self.x, self.y, brick_size[1], brick_size[0])
        return location

    def __repr__(self):
        return f"Brick({self.x}, {self.y}, {self.layer}, {self.rotation})"

    def __eq__(self, other):
        return (
            self.x == other.x
            and self.y == other.y
            and self.layer == other.layer
            and self.rotation == other.rotation
        )
