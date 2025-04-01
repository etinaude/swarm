import json
import random
from global_state import State
import pygame  # type: ignore
from specs import rover_count, gluer_count, drone_count, map_size
from brick import Brick
from bots.house import House, top_left, house_size
from bots.gluer import Gluer
from bots.rover import Rover
from bots.drone import Drone
from shapely.geometry import Polygon, Point


pygame.init()
screen = pygame.display.set_mode((map_size[0], map_size[1]))
clock = pygame.time.Clock()
sim_speed = 2

house = House(screen)

rovers = []
gluers = []
drones = []
running = True
brick_pile = Point(10, 10)

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
        x = random.randint(200, map_size[0] - 200)
        y = random.randint(0, 100)

        pos = Point(x, y)
        found = False
        for gluer in gluers:
            if gluer.pos.distance(pos) < 50:
                found = True
                break
        if found:
            continue
        i += 1
        gluers.append(Gluer(x, y, screen, sim_speed))

    i = 0
    while i < drone_count:
        x = random.randint(top_left[0], house_size[0] + top_left[0])
        y = random.randint(top_left[1], house_size[1] + top_left[1])

        pos = Point(x, y)
        found = False
        for drone in drones:
            if drone.pos.distance(pos) < 50:
                found = True
                break
        if found:
            continue
        i += 1
        drones.append(Drone(x, y, state))

    for i in range(rover_count):
        x = random.randint(0, top_left[0])
        y = random.randint(0, top_left[1])
        rovers.append(Rover(x, y, state))


def draw():
    screen.fill((255, 255, 255))
    house.draw_goal()
    house.draw()

    pile.draw()

    for brick in state.loose_bricks:
        brick.draw()

    for rover in rovers:
        rover.draw()

    for gluer in gluers:
        gluer.draw()

    for drone in drones:
        drone.draw()

    pygame.display.flip()
    clock.tick(10)


def step():
    draw()

    for rover in rovers:
        rover.make_move(gluers)

    for drone in drones:
        drone.make_move(state.loose_bricks, house)

    for gluer in gluers:
        gluer.update()


if __name__ == "__main__":
    state = State(gluers, brick_pile, house, screen, sim_speed)

    init_robots()
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        for i in range(100):
            step()
            if len(state.canidate_bricks) == 0:
                if len(house.get_rover_bricks()) == 0:
                    pass

            pygame.display.flip()
            clock.tick(120)
