from specs import brick_size, house_size, top_left
from brick import Brick


# generate bricks
bricks = []

for x in range(top_left[0], house_size[0] + top_left[0], brick_size[0]):
    for y in range(top_left[1], house_size[1] + top_left[1], brick_size[1]):
        bricks.append(Brick(x, y, 1, 0))
