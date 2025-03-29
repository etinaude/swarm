from position import Position
import pygame  # type: ignore
from shapely.geometry import LineString


class Node:
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def a_star(start, end, maze):
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0
    open_list = []
    closed_list = []

    open_list.append(start_node)

    while len(open_list) > 0:
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path

        # Generate children
        children = []
        for new_position in [
            (0, -1),
            (0, 1),
            (-1, 0),
            (1, 0),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]:  # Adjacent squares
            node_position = (
                current_node.position[0] + new_position[0],
                current_node.position[1] + new_position[1],
            )
            if (
                node_position[0] > (len(maze) - 1)
                or node_position[0] < 0
                or node_position[1] > (len(maze[len(maze) - 1]) - 1)
                or node_position[1] < 0
            ):
                continue
            if maze[node_position[0]][node_position[1]] != 0:
                continue
            new_node = Node(current_node, node_position)
            children.append(new_node)

        # Loop through children
        for child in children:
            for closed_child in closed_list:
                if child == closed_child:
                    continue
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                (child.position[1] - end_node.position[1]) ** 2
            )
            child.f = child.g + child.h
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue
            open_list.append(child)


def find_path(start, target, speed, state=None):
    if not isinstance(target, Position):
        target = target.pos

    if not isinstance(start, Position):
        start = start

    start_tuple = (start.x, start.y)
    target_tuple = (target.x, target.y)

    a_star_path = a_star(start_tuple, target_tuple, state.house.maze)

    if a_star_path is None or len(a_star_path) == 0:
        print("No path found")
        return None

    return a_star_path


def move_directly(self, speed, target):
    direction = self.get_direction(target)
    distance = self.get_dist(target)
    if distance < speed:
        self = target.copy_pos()
        return True
    else:
        self.x += direction[0] * speed
        self.y += direction[1] * speed
        return False


def draw_point(x, y, screen):
    pygame.draw.circle(screen, (255, 0, 0), (x, y), 5)


def draw_lineString(line, screen):
    x, y = line.xy
    pygame.draw.lines(screen, (0, 0, 255), False, list(zip(x, y)), 2)
