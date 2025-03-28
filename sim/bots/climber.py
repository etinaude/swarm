import pygame
from specs import brick_size, brick_pile

speed = 20
size = [20, 20]


class Rover:
    def __init__(self, x, y, screen):
        self.x = x
        self.y = y
        self.carrying = False
        self.screen = screen

    def draw(self):
        color = (20, 120, 20)
        if self.carrying:
            brick_color = (79, 39, 2)
            brick_location = (
                self.x,
                self.y,
                brick_size[0],
                brick_size[1],
            )
            pygame.draw.rect(self.screen, brick_color, brick_location)

        location = (self.x, self.y, size[0], size[1])
        pygame.draw.rect(self.screen, color, location)

    def __repr__(self):
        return f"Robot - ({self.x}, {self.y}, {self.layer}, {self.rotation})"

    def make_move(self, bricks, house):
        if not self.carrying or bricks == []:
            if self.get_distance(brick_pile) > 10:
                self.move_towards_target(brick_pile)
            else:
                self.carrying = True
        else:
            brick = self.find_brick_to_place(bricks)
            if self.get_distance(brick.get_location()) > 10:
                self.move_towards_target(brick.get_location())
            else:
                return self.place_brick(brick, house, bricks)
        return bricks

    def place_brick(self, brick, house, bricks):
        self.carrying = False
        print("Placing brick at", brick.x, brick.y)
        house.place_brick(brick)

        # remove brick from canidate bricks
        return list(filter(lambda x: x != brick, bricks))

    def find_brick_to_place(self, bricks):
        closest = bricks[0]
        closest_dist = self.get_distance(closest.get_location())
        for brick in bricks:
            if self.get_distance(brick.get_location()) < closest_dist:
                closest = brick
                closest_dist = self.get_distance(brick.get_location())
        return closest

    def get_distance(self, target):
        x_diff = target[0] - self.x
        y_diff = target[1] - self.y
        return (x_diff**2 + y_diff**2) ** 0.5

    def get_direction(self, target):
        x_diff = target[0] - self.x
        y_diff = target[1] - self.y
        distance = (x_diff**2 + y_diff**2) ** 0.5
        if distance == 0:
            return (0, 0)

        x = x_diff / distance
        y = y_diff / distance
        return (x, y)

    def move_towards_target(self, target):
        direction = self.get_direction(target)
        distance = self.get_distance(target)
        if distance < speed:
            self.x = target[0]
            self.y = target[1]
        else:
            self.x += direction[0] * speed
            self.y += direction[1] * speed
