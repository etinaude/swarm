import uuid
from position import Position
import pygame  # type: ignore

size = [20, 20]

# states
# idle
# getting brick
# placing brick


class Drone:
    def __init__(self, x, y, global_state):
        self.pos = Position(x, y)

        self.state = "idle"
        self.battery = 100
        self.brick = None
        self.target = None
        self.speed = 2 * global_state.sim_speed
        self.global_state = global_state
        self.id = uuid.uuid4()

    def draw(self):
        color = (120, 120, 20)
        if self.brick is not None:
            self.brick.pos = self.pos.copy_pos()

            self.brick.draw()

        location = (self.pos.x, self.pos.y, size[0], size[1])
        pygame.draw.rect(self.global_state.screen, color, location)

    def __repr__(self):
        return f"Drone - ({self.pos.x}, {self.pos.y})"

    def make_move(self):
        decide_next_task(self)
        self.battery -= 1

        if self.state == "getting_brick":
            self.get_brick()

        if self.state == "placing_brick":
            self.place_brick()

    def get_brick(self):
        possible_bricks = []
        found = False
        for brick in self.global_state.loose_bricks:
            if brick.claimed_by is self.id:
                found = True
            if brick.claimed_by is None:
                possible_bricks.append(brick)
                break

        if not found:
            if len(possible_bricks) == 0:
                self.state = "idle"
                return
            # find closest brick
            best_brick = possible_bricks[0]
            for brick in possible_bricks:
                if self.pos.get_dist(brick.pos) < self.pos.get_dist(best_brick.pos):
                    best_brick = brick
            self.target = best_brick.pos
            best_brick.claimed_by = self.id

        if self.target is None:
            self.state = "idle"
            return

        if self.pos.get_dist(self.target) > self.speed:
            self.move_towards_target()
        else:
            # find index of target in loose_bricks
            loose_bricks = self.global_state.loose_bricks
            for i in range(len(loose_bricks)):
                if loose_bricks[i].pos == self.target:
                    self.brick = loose_bricks[i]
                    self.global_state.loose_bricks.pop(i)
                    break

    def place_brick(self):
        poses = list(map(lambda x: x.pos, self.global_state.canidate_bricks))
        self.target = self.pos.find_closest(poses)

        if self.target is None:
            self.state = "idle"

        if self.pos.get_dist(self.target) > self.speed:
            self.move_towards_target()

        else:
            self.global_state.house.place_brick(self.target)
            self.brick = None

    def move_towards_target(self):
        direction = self.pos.get_direction(self.target)
        distance = self.pos.get_dist(self.target)
        if distance < self.speed:
            self.pos = self.target
        else:
            self.pos.x += direction[0] * self.speed
            self.pos.y += direction[1] * self.speed


def decide_next_task(self):
    if self.brick is None:
        self.state = "getting_brick"
    else:
        self.state = "placing_brick"
