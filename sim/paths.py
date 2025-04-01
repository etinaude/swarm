import pygame  # type: ignore
from shapely.geometry import Polygon, Point
import heapq
import time

class Node():
    """A node class for Anytime A* Pathfinding"""
    
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
    
    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f

def anytime_a_star(start_pos, target, maze, speed=1, time_limit=0.1):
    start_time = time.time()
    
    start = (int(start_pos.x), int(start_pos.y))
    end = (int(target.x), int(target.y))
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0
    
    open_list = []
    closed_list = set()
    
    heapq.heappush(open_list, start_node)
    
    best_path = None
    best_cost = float('inf')
    
    while open_list and (time.time() - start_time) < time_limit:
        
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)
        
        distance = ((current_node.position[0] - end_node.position[0]) ** 2 +
                    (current_node.position[1] - end_node.position[1]) ** 2) ** 0.5
        
        if distance < speed:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            path.reverse()
            
            cost = current_node.f
            if cost < best_cost:
                best_cost = cost
                best_path = path
        
        for new_position in [
            (0, -speed), (0, speed), (-speed, 0), (speed, 0),
            (-speed, -speed), (-speed, speed), (speed, -speed), (speed, speed)
        ]:
            node_position = (
                current_node.position[0] + new_position[0],
                current_node.position[1] + new_position[1],
            )
            if (
                node_position[0] < 0 or node_position[0] >= len(maze) or
                node_position[1] < 0 or node_position[1] >= len(maze[0])
            ):
                continue
            if maze[node_position[0]][node_position[1]] != 0:
                continue
            if node_position in closed_list:
                continue
            
            new_node = Node(current_node, node_position)
            new_node.g = current_node.g + 1
            new_node.h = ((new_node.position[0] - end_node.position[0]) ** 2) + \
                         ((new_node.position[1] - end_node.position[1]) ** 2)
            new_node.f = new_node.g + new_node.h
            
            heapq.heappush(open_list, new_node)
    
    return best_path if best_path else []


def find_path(start, target, maze, speed=1):
    a_star_path = anytime_a_star(start, target, maze, speed)

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

def draw_point(x, y, screen, color=(0, 0, 0)):
    pygame.draw.circle(screen, color, (x, y), 5)


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