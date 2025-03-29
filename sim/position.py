from specs import brick_size


class Position:
    def __init__(self, x, y, layer=0, rotation=0):
        self.x = x
        self.y = y
        self.layer = layer
        self.rotation = rotation

    def get_location(self):
        location = (self.x, self.y, brick_size[0], brick_size[1])
        if self.rotation == 90:
            location = (self.x, self.y, brick_size[1], brick_size[0])
        return location

    def __repr__(self):
        return f"({round(self.x, 2)}, {round(self.y, 2)}, {round(self.layer, 2)}, {round(self.rotation, 2)})"

    def __eq__(self, other):
        if not isinstance(other, Position):
            other = other.pos
        return (
            self.x == other.x
            and self.y == other.y
            and self.layer == other.layer
            and self.rotation == other.rotation
        )

    def get_dist(self, other):

        if isinstance(other, Position):
            x = other.x
            y = other.y
        else:
            x = other.pos.x
            y = other.pos.y

        x_diff = x - self.x
        y_diff = y - self.y
        return (x_diff**2 + y_diff**2) ** 0.5

    def get_direction(self, other):
        x_diff = other.x - self.x
        y_diff = other.y - self.y
        distance = (x_diff**2 + y_diff**2) ** 0.5
        if distance == 0:
            return (0, 0)
        x = x_diff / distance
        y = y_diff / distance
        return (x, y)

    def copy_pos(self):
        return Position(self.x, self.y, self.layer, self.rotation)

    def find_closest(self, others):
        if not others:
            return None

        closest = others[0]
        closest_dist = self.get_dist(closest)
        for other in others:
            if not isinstance(other, Position):
                other_pos = other.pos
            if self.get_dist(other_pos) < closest_dist:
                closest = other
                closest_dist = self.get_dist(other_pos)
        return closest

    def move_towards(self, speed, target):
        direction = self.get_direction(target)
        distance = self.get_dist(target)
        if distance < speed:
            self = target.copy_pos()
            return True
        else:
            self.x += direction[0] * speed
            self.y += direction[1] * speed
            return False

    def goto(self, x, y):
        self.x = x
        self.y = y
