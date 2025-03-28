import json
import random
from position import Position
import pygame  # type: ignore
from specs import rover_count, gluer_count
from brick import Brick
from bots.house import House
from bots.gluer import Gluer
from bots.rover import Rover


pygame.init()
screen = pygame.display.set_mode((1000, 1000))
clock = pygame.time.Clock()
sim_speed = 10

house = House(screen)

rovers = []
gluers = []
climbers = []

brick_pile = Position(100, 100)

pile = Brick(
    brick_pile.x,
    brick_pile.y,
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
    i = 0
    while i < gluer_count:
        x = random.randint(200, 500)
        y = random.randint(0, 200)

        pos = Position(x, y)
        for gluer in gluers:
            if gluer.pos.get_dist(pos) < 100:
                continue
        i += 1
        gluers.append(Gluer(x, y, screen, sim_speed))

    for i in range(rover_count):
        x = random.randint(0, 500)
        y = random.randint(0, 500)
        rovers.append(Rover(x, y, screen, gluers, brick_pile, sim_speed))


def draw():
    screen.fill((255, 255, 255))
    house.draw_goal()
    house.draw()

    pile.draw()

    for rover in rovers:
        rover.draw()

    for gluer in gluers:
        gluer.draw()

    for climber in climbers:
        climber.draw()

    pygame.display.flip()
    clock.tick(10)


def step():
    global canidate_bricks
    for rover in rovers:
        if canidate_bricks == []:
            canidate_bricks = house.get_canidate_bricks()
        canidate_bricks = rover.make_move(canidate_bricks, house)

    for gluer in gluers:
        gluer.update()

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
