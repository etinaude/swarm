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
        self.wall_target = None

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

    def claim_brick(self, brick_list):
        possible_bricks = []
        found = False
        for brick in brick_list:
            if brick.drone_claimed_by is self.id:
                return
            if brick.drone_claimed_by is None:
                # if disncae from brick to wall target < 100
                if self.wall_target is not None:
                    if brick.pos.get_dist(self.wall_target) < 100:
                        possible_bricks.append(brick)

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

    def get_brick(self):
        self.claim_brick(self.global_state.loose_bricks)

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
        if self.wall_target is None:
            self.state = "idle"
            return
        self.target = self.wall_target

        if self.pos.get_dist(self.wall_target) > self.speed:
            self.move_towards_target()
        else:
            self.global_state.house.place_brick(self.wall_target)
            self.brick = None
            self.wall_target = None

    def move_towards_target(self):
        direction = self.pos.get_direction(self.target)
        distance = self.pos.get_dist(self.target)
        if distance < self.speed:
            self.pos = self.target
        else:
            self.pos.x += direction[0] * self.speed
            self.pos.y += direction[1] * self.speed


def decide_next_task(self):
    if not self.wall_target:
        canidates = self.global_state.house.get_drone_bricks()
        if canidates == []:
            self.state = "idle"
            return
        closest = canidates[0]
        for brick in canidates:
            if self.pos.get_dist(brick.pos) < self.pos.get_dist(closest.pos):
                closest = brick
        self.wall_target = closest.pos
        closest.drone_claimed_by = self.id

    if self.brick is None:
        self.state = "getting_brick"
    else:
        self.state = "placing_brick"
