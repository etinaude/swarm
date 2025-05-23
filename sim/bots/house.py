from brick import Brick
from specs import brick_size, map_size
from shapely.geometry import Polygon, Point
from paths import draw_point, draw_lineString


house_size = [400, 225, 1]
top_left = Point(100, 200)
bottom_right = Point(
    top_left.x + house_size[0] + brick_size[1],
    top_left.y + house_size[1],
)
top_right = Point(
    top_left.x + house_size[0] + brick_size[1],
    top_left.y,
)
bottom_left = Point(
    top_left.x,
    top_left.y + house_size[1],
)



class House:
    def __init__(self):
        self.bricks = []
        self.make_structure()
        
        self.maze = []
        self.polygon = Polygon(
            [
                top_left,
                top_right,
                bottom_right,
                bottom_left,
            ]
        )

        self.make_maze()

    def make_structure(self):
        for layer in range(house_size[2]):
            for x in range(int(top_left.x), int(house_size[0] + top_left.x), brick_size[0]):
                bottom_wall_y = top_left.y + house_size[1] - brick_size[1]
                bottom_wall_x = x if (layer % 2) else x + brick_size[1]
                top_wall_x = x + brick_size[1] if (layer % 2) else x

                top_brick = Brick(top_wall_x, top_left.y, layer, 0)
                bottom_brick = Brick(
                    bottom_wall_x, bottom_wall_y, layer, 0
                )
                self.bricks.extend([top_brick, bottom_brick])

            for y in range(
                int(top_left.y),
                int(house_size[1] + top_left.y - brick_size[1]),
                brick_size[0],
            ):
                right_wall_x = top_left.x + house_size[0]
                right_wall_y = y + brick_size[1] if (layer % 2) else y
                left_wall_y = y if (layer % 2) else y + brick_size[1]

                left_brick = Brick(top_left.x, left_wall_y, layer, 90)
                right_brick = Brick(right_wall_x, right_wall_y, layer, 90)
                self.bricks.extend([left_brick, right_brick])

    def draw(self):
        for brick in self.bricks:
            if brick.placed:
                brick.draw()
        self.draw_goal()

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
                        and x.lifter_claimed_by
                        and not (x.rover_claimed_by)
                    ),
                    self.bricks,
                )
            )
            current_layer += 1
        return canidates

    def get_lifter_bricks(self):
        current_layer = self.current_layer()
        canidates = []
        while canidates == [] and current_layer < house_size[2]:
            canidates = list(
                filter(
                    lambda x: (
                        (not x.placed)
                        and x.layer == current_layer
                        and not (x.lifter_claimed_by)
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

        bounds = self.polygon.buffer(20).boundary

        for y in range(map_size[1]):
            if y < bounds.bounds[1] or y > bounds.bounds[3]:
                house_row.append(0)
            else:
                house_row.append(1)

        for x in range(map_size[0]):
            if x < bounds.bounds[0] or x > bounds.bounds[2]:
                self.maze.append(empty_row)
            else:
                self.maze.append(house_row)




    # def draw_maze(self):
    #     for x in range(len(self.maze)):
    #         for y in range(0, len(self.maze[0]),10):
    #             if self.maze[x][y] == 1:
    #                 draw_point(x, y, self.screen, (255, 0, 0))
    #             else:
    #                 pass
    #                 # draw_point(x, y, self.screen)