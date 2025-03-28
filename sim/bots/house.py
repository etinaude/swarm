from brick import Brick
from specs import brick_size

house_size = [500, 350, 5]
top_left = [100, 300]


class House:
    def __init__(self, screen):
        self.bricks = []
        self.screen = screen
        self.make_structure()

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
        return min([brick.pos.layer for brick in not_placed] + [999])

    def get_rover_bricks(self):
        current_layer = self.current_layer()
        return list(
            filter(
                lambda x: (
                    (not x.placed)
                    and x.pos.layer == current_layer
                    and x.drone_claimed_by
                    and not (x.rover_claimed_by)
                ),
                self.bricks,
            )
        )

    def get_drone_bricks(self):
        current_layer = self.current_layer()
        return list(
            filter(
                lambda x: (
                    (not x.placed)
                    and x.pos.layer == current_layer
                    and not (x.drone_claimed_by)
                ),
                self.bricks,
            )
        )

    def place_brick(self, target):
        bricks = list(filter(lambda x: x.pos == target, self.bricks))

        if bricks == []:
            print("ERROR: No bricks found")
            print(f"target: {target}")
            return
        brick = bricks[0]

        brick.placed = True
        brick.draw()
