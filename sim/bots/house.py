from brick import Brick
from specs import brick_size, map_size, house_size, top_left
from shapely.geometry import Polygon, Point

class House:
    def __init__(self, screen):
        self.bricks = []
        self.screen = screen
        self.make_structure()
        self.top_left = top_left
        self.maze = []
        self.make_maze()

        top = top_left[0]
        bottom = top_left[0] + house_size[0] + brick_size[0]
        left = top_left[1]
        right = top_left[1] + house_size[1] + brick_size[1]

        self.polygon = Polygon(
            [
                (top, left),
                (top, right),
                (bottom, right),
                (bottom, left),
            ]
        )

    def make_structure(self):
        for layer in range(house_size[2]):
            for x in range(top_left[0], house_size[0] + top_left[0], brick_size[0]):
                bottom_wall_y = top_left[1] + house_size[1] - brick_size[1]
                bottom_wall_x = x if (layer % 2) else x + brick_size[1]
                top_wall_x = x + brick_size[1] if (layer % 2) else x

                top_brick = Brick(top_wall_x, top_left[1], layer, 0, self.screen)
                bottom_brick = Brick(
                    bottom_wall_x, bottom_wall_y, layer, 0, self.screen
                )
                self.bricks.extend([top_brick, bottom_brick])

            for y in range(
                top_left[1],
                house_size[1] + top_left[1] - brick_size[1],
                brick_size[0],
            ):
                right_wall_x = top_left[0] + house_size[0]
                right_wall_y = y + brick_size[1] if (layer % 2) else y
                left_wall_y = y if (layer % 2) else y + brick_size[1]

                left_brick = Brick(top_left[0], left_wall_y, layer, 90, self.screen)
                right_brick = Brick(right_wall_x, right_wall_y, layer, 90, self.screen)
                self.bricks.extend([left_brick, right_brick])

    def draw(self):
        for brick in self.bricks:
            if brick.placed:
                brick.draw()

    def draw_goal(self):
        for brick in self.bricks:
            brick.draw_outline()

    def detect_collision(self):
        pass

    def current_layer(self):
        not_placed = list(filter(lambda x: not (x.placed), self.bricks))
        return min([brick.layer for brick in not_placed] + [999])

    def get_rover_bricks(self):
        current_layer = self.current_layer()
        canidates = []
        while canidates == [] and current_layer < house_size[2]:
            canidates = list(
                filter(
                    lambda x: (
                        (not x.placed)
                        and x.layer == current_layer
                        and x.drone_claimed_by
                        and not (x.rover_claimed_by)
                    ),
                    self.bricks,
                )
            )
            current_layer += 1
        return canidates

    def get_drone_bricks(self):
        current_layer = self.current_layer()
        canidates = []
        while canidates == [] and current_layer < house_size[2]:
            canidates = list(
                filter(
                    lambda x: (
                        (not x.placed)
                        and x.layer == current_layer
                        and not (x.drone_claimed_by)
                    ),
                    self.bricks,
                )
            )
            current_layer += 1
        return canidates

    def place_brick(self, target):
        bricks = list(filter(lambda x: x.pos == target, self.bricks))

        if bricks == []:
            print("ERROR: No bricks found")
            print(f"target: {target}")
            return
        brick = bricks[0]

        brick.placed = True
        brick.draw()

    def make_maze(self):
        # maze is mapped down 10:1
        self.maze = []
        empty_row = [0] * (map_size[0])
        house_row = []
        padding = 10

        for y in range(map_size[0]):
            if y < top_left[0] or y > (top_left[0] + house_size[0] + brick_size[0]):
                house_row.append(0)
            else:
                house_row.append(1)

        for x in range(map_size[1]):
            if x < top_left[1] or y > (top_left[1] + house_size[1] + brick_size[1]):
                self.maze.append(empty_row)
            else:
                self.maze.append(house_row)

