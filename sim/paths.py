import pygame  # type: ignore
from shapely.geometry import Polygon, Point

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


def a_star(start_pos, target, maze, speed=1):
    start = (int(start_pos.x), int(start_pos.y))
    end = (int(target.x), int(target.y))
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
        distance = ((current_node.position[0] - end_node.position[0]) ** 2 + (
            current_node.position[1] - end_node.position[1]
        ) ** 2) ** 0.5

        if distance <= speed:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path

        # Generate children
        children = []
        for new_position in [
            (0, -speed),
            (0, speed),
            (-speed, 0),
            (speed, 0),
            (-speed, -speed),
            (-speed, speed),
            (speed, -speed),
            (speed, speed),
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
                print("WALL")
                print(len(open_list))
                display_node(node_position, state.screen)
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

    print("Open list length: ", len(open_list))
    print("Closed list length: ", len(closed_list))
    print("CANT FIND PATH")
    return []


def find_path(start, target, maze, speed=1):
    a_star_path = a_star(start, target, maze, speed)

    if a_star_path is None or len(a_star_path) == 0:
        print("No path found")
        return []

    return a_star_path

def get_direction(pointFrom, pointTo):
    x_diff = pointTo.x - pointFrom.x
    y_diff = pointTo.y - pointFrom.y
    distance = (x_diff**2 + y_diff**2) ** 0.5
    if distance == 0:
        return (0, 0)
    x = x_diff / distance
    y = y_diff / distance
    return (x, y)

def move_directly(selfPos, target, speed):
    distance = selfPos.distance(target)
    if distance < speed:
        return Point(target.x, target.y)
    else:
        direction = get_direction(selfPos, target)
        x = (selfPos.x + direction[0] * speed)
        y = (selfPos.y + direction[1] * speed)
        return Point(x, y)

def draw_point(x, y, screen):
    pygame.draw.circle(screen, (255, 0, 0), (x, y), 5)


def draw_lineString(line, screen):
    x, y = line.xy
    pygame.draw.lines(screen, (0, 0, 255), False, list(zip(x, y)), 2)


def display_node(node, screen):
    x, y = node
    pygame.draw.circle(screen, (0, 255, 0), (x, y), 2)
    pygame.display.flip()
    pygame.time.delay(100)


def draw_path(path, screen):
    for node in path:
        display_node(node, screen)

def find_closest(pos, others):
        if not others:
            return None

        closest = others[0]
        if isinstance(closest, Point):
            closest_dist = pos.distance(closest) 
        else:
            closest_dist = pos.distance(closest.pos)
        
        for other in others:
            if isinstance(other, Point):
                other_pos = other
            else:
                other_pos = other.pos
            if pos.distance(other_pos) < closest_dist:
                closest = other
                closest_dist = pos.distance(other_pos)
        return closest