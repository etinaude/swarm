import uuid
from position import Position
import pygame  # type: ignore
from specs import brick_size
from shapely.geometry import Polygon, Point
from shapely.ops import nearest_points
from paths import move_directly, draw_path, draw_lineString

size = [10, 10]

class Drone:
    def __init__(self, x, y, global_state):
        self.pos = Point(x, y)

        self.global_state = global_state
        self.speed = 2 * global_state.sim_speed
        self.battery = 100
        self.id = uuid.uuid4()

        self.state = "pick_wall_target"
        self.brick = None
        self.target = None
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
        self.battery -= 1

        print(self.state)

        if self.state == "pick_wall_target":
            self.pick_wall_target()
        elif self.state == "move_to_pickup":
            self.move_to_pickup()
        elif self.state == "claim_brick":
            self.get_brick()
        elif self.state == "getting_brick":
            self.get_brick()
        elif self.state == "placing_brick":
            self.place_brick()

    def move_to_pickup(self):
        if self.wall_target is None:
            return

        # move to outside of wall
        target = self.wall_target
        house = self.global_state.house.polygon
        bounds = house.buffer(30).boundary

        draw_lineString(bounds, self.global_state.screen)

        points = nearest_points(
            self.pos, bounds
        )
        self.target = points[1]

        dist = self.pos.distance(self.target)

        if self.pos.distance(self.target) > self.speed:
            self.pos = move_directly(self.pos, self.target, self.speed)
            return

        # pick up complete
        self.state = "claim_brick"
        self.wall_target = None

    def claim_brick(self, brick_list):
        possible_bricks = []
        for brick in brick_list:
            if brick.drone_claimed_by is None:
                if self.wall_target is not None:
                    if brick.pos.distance(self.wall_target) < brick_size * 3:
                        possible_bricks.append(brick)

        if len(possible_bricks) == 0:
            return

        # find closest brick
        best_brick = possible_bricks[0]
        for brick in possible_bricks:
            if self.pos.distance(brick.pos) < self.pos.distance(best_brick.pos):
                best_brick = brick

        self.target = best_brick.pos
        best_brick.claimed_by = self.id

        # claim complete
        self.state = "get_brick"

    def get_brick(self):
        if self.target is None:
            self.target = self.wall_target
            self.target.x += 10
            self.target.y += 10

        if self.pos.distance(self.target) > self.speed:
            self.pos.move_towards(self.speed, self.target)
            return

        # find index of target in loose_bricks
        loose_bricks = self.global_state.loose_bricks
        for i in range(len(loose_bricks)):
            if loose_bricks[i].pos == self.target:
                self.brick = loose_bricks[i]
                self.global_state.loose_bricks.pop(i)
                break

        # picking up complete
        self.state = "placing_brick"

    def place_brick(self):
        if self.wall_target is None:
            return
        self.target = self.wall_target

        if self.pos.distance(self.wall_target) > self.speed:
            self.pos.move_towards(self.speed, self.target)
            return

        # place brick complete
        self.global_state.house.place_brick(self.wall_target)
        self.brick = None
        self.wall_target = None
        self.state = "pick_wall_target"

    def pick_wall_target(self):
        canidates = self.global_state.house.get_drone_bricks()
        if canidates == []:
            return
        closest = canidates[0]
        for brick in canidates:
            if self.pos.distance(brick.pos) < self.pos.distance(closest.pos):
                closest = brick

        self.wall_target = closest.pos
        closest.drone_claimed_by = self.id
        self.state = "move_to_pickup"


# pick wall target
# move to pick up
# (wait until brick) claim brick
# pick up
# place brick
# repeat
