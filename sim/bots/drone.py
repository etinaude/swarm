import uuid
import pygame  # type: ignore
from specs import brick_size
from shapely.geometry import Polygon, Point
from shapely.ops import nearest_points
from paths import move_directly, draw_path, draw_lineString, find_closest

size = [10, 10]

# pick wall target
# move to pick up
# (wait until brick) claim brick
# pick up
# place brick
# repeat

class Drone:
    def __init__(self, x, y, global_state):
        self.pos = Point(x, y)

        self.speed = 2 * global_state.sim_speed
        self.screen = global_state.screen
        self.housepoly = global_state.house.polygon
        self.battery = 100
        self.id = uuid.uuid4()

        self.state = "move_to_pickup"
        self.brick = None
        self.target = None
        self.wall_target = None

    def draw(self):
        color = (120, 120, 20)
        if self.brick is not None:
            self.brick.pos = Point(self.pos.x, self.pos.y)

            self.brick.draw()

        location = (self.pos.x, self.pos.y, size[0], size[1])
        pygame.draw.rect(self.screen, color, location)

    def __repr__(self):
        return f"Drone - ({self.pos.x}, {self.pos.y})"

    def make_move(self, loose_bricks, house):
        self.battery -= 1

        # print("DRONE", self.state)


        if self.state == "move_to_pickup":
            self.move_to_pickup(house)
        elif self.state == "waiting_for_brick":
            return
        elif self.state == "placing_brick":
            self.place_brick(house)

    def move_to_pickup(self, house):
        if self.wall_target is None:
            self.pick_wall_target(house)

        # move to outside of wall
        target = self.wall_target
        bounds = self.housepoly.buffer(30).boundary

        points = nearest_points(
            self.pos, bounds
        )
        self.target = points[1]

        dist = self.pos.distance(self.target)

        if self.pos.distance(self.target) > self.speed:
            self.pos = move_directly(self.pos, self.target, self.speed)
            return

        # pick up complete
        self.state = "waiting_for_brick"

    def get_brick(self, brick):
        self.state = "placing_brick"
        self.brick = brick

    def place_brick(self, house):
        if self.wall_target is None:
            return
        self.target = self.wall_target

        if self.pos.distance(self.target) > self.speed:
            self.pos = move_directly(self.pos, self.target, self.speed)
            return

        # place brick complete
        house.place_brick(self.wall_target)
        self.brick = None
        self.wall_target = None
        self.state = "move_to_pickup"

    def pick_wall_target(self, house):
        canidates = house.get_drone_bricks()
        if canidates == []:
            return
        closest = canidates[0]
        for brick in canidates:
            if self.pos.distance(brick.pos) < self.pos.distance(closest.pos):
                closest = brick

        self.wall_target = closest.pos
        closest.drone_claimed_by = self.id



