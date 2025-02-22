import json
import random
import time
import pygame
from specs import brick_size, brick_pile, robot_count
from brick import Brick
from house import House
from robot import Robot


pygame.init()
screen = pygame.display.set_mode((1000, 1000))
clock = pygame.time.Clock()

house = House(screen)
robots = []
pile = Brick(
    brick_pile[0],
    brick_pile[1],
    0,
    0,
    screen,
)


def load_data():
    with open("bricks.json") as f:
        d = json.load(f)
        for brick in d:
            house.bricks.append(
                Brick(brick["x"], brick["y"], brick["layer"], brick["rotation"])
            )


def init_robots():
    for i in range(robot_count):
        x = random.randint(0, 500)
        y = random.randint(0, 500)
        robots.append(Robot(x, y, screen))


def draw():
    screen.fill((255, 255, 255))
    house.draw()

    pile.draw()

    for robot in robots:
        robot.draw()

    pygame.display.flip()
    clock.tick(10)


def step():
    global canidate_bricks
    for robot in robots:
        if canidate_bricks == []:
            canidate_bricks = house.get_canidate_bricks()
        canidate_bricks = robot.make_move(canidate_bricks, house)
    draw()


init_robots()

running = True
canidate_bricks = []

if __name__ == "__main__":

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for i in range(100):
            step()

            if len(canidate_bricks) == 0:
                canidate_bricks = house.get_canidate_bricks()
                if len(canidate_bricks) == 0:
                    print("Done")

            pygame.display.flip()
            clock.tick(120)
