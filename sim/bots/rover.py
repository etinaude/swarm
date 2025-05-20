import uuid
from brick import Brick
from paths import draw_lineString, find_path, move_directly, find_closest
import pygame  # type: ignore
from shapely.geometry import Polygon, Point, LineString
from direct.showbase.ShowBase import ShowBase
from panda3d.core import Vec3

size = [10, 10]

# ~~~ STATES ~~~
# idle
# getting brick
# going to gluer
# waiting for gluer
# going to lifter
# drop off brick

# TODO: later
# getting battery
# deliver battery
# get glue

class Rover:
    def __init__(self, start_pos, render, state):
        self.node = loader.loadModel("assests/rover.gltf")
        self.node.reparentTo(render)
        self.node.setPos(start_pos)
        self.node.set_scale(10)
        self.name = uuid.uuid4()

        self.pos = start_pos

        # self.pos = Point(x, y)
        # self.screen = state.screen
        self.maze = state.house.maze
        self.brick_pile = state.brick_pile
        
        self.state = "getting_brick"
        self.battery = 100
        self.brick = None
        self.target = None
        self.speed = 6 * state.sim_speed
        self.path = None

    def update(self):
        self.node.setPos(self.node.getPos() + Vec3(0, 0, 0))


        # color = (20, 120, 20)
        # if self.brick is not None:
        #     self.brick.pos = Point(self.pos.x, self.pos.y)

        #     self.brick.draw()

        # location = (self.pos.x, self.pos.y, size[0], size[1])
        # # pygame.draw.rect(self.screen, color, location)

        # if self.path is None or len(self.path) < 2:
        #     return

        # line = LineString(self.path)
        # # draw_lineString(line, self.screen)

    def __repr__(self):
        return f"Rover - ({self.pos.x}, {self.pos.y})"

    def make_move(self, gluers, lifters):
        # self.decide_next_task()
        self.battery -= 1

        if self.state == "getting_brick":
            self.get_brick()

        if self.state == "goto_gluer":
            self.goto_gluer(gluers)

        if self.state == "wait_for_gluer":
            self.wait_for_gluer(gluers)

        if self.state == "goto_lifter":
            self.goto_lifter(lifters)

        if self.state == "idle":
            self.idle()

    def get_brick(self):
        if self.path is None:
            self.path = find_path(
                self.pos, self.brick_pile, self.maze, self.speed
            )

        arrived = self.move_along_path()
        if arrived:
            self.brick = Brick(self.pos.x, self.pos.y, 0, 0)
            self.path = None
            self.state = "goto_gluer" 

    def goto_gluer(self, gluers):
        idle_gluers = list(
            filter(lambda x: x.state == "idle", gluers)
        )
        target = find_closest(self.pos, idle_gluers)

        if target is None:
            self.state = "idle"
            print("No gluer found")
            return

        if self.target is not target.pos:
            self.path = None
            self.target = target.pos

        if self.path is None:
            self.path = find_path(self.pos, self.target, self.maze, self.speed)

        arrived = self.move_along_path()
        if arrived:
            target.glue(self.brick)
            self.brick = None
            self.state = "wait_for_gluer"

    def wait_for_gluer(self, gluers):
        # find gluers in raduis with state 'ready"
        gluers = list(
            filter(
                lambda x: x.pos.distance(self.pos) < self.speed+100 and x.state == "ready", gluers
            )
        )
        if gluers == []:
            return

        gluer = gluers[0]

        self.target = Point(gluer.pos.x, gluer.pos.y)

        self.pos = move_directly(self.pos, self.target, self.speed)
        self.brick = gluer.give_brick()
        self.target = None
        self.path = None
        self.state = "goto_lifter"

    def goto_lifter(self, lifters):
        idle_lifters = list(
            filter(lambda x: x.state == "waiting_for_brick", lifters)
        )
        target = find_closest(self.pos, idle_lifters)

        if target is None:
            return

        if self.target is not target.pos:
            self.path = None
            self.target = target.pos

        if self.path is None:
            self.path = find_path(self.pos, self.target, self.maze, self.speed)

        arrived = self.move_along_path()
        if arrived:
            print("arrived at lifter")
            target.get_brick(self.brick)
            self.brick = None
            self.state = "idle"
            self.path = None

    def idle(self):
        self.state = "getting_brick"
        self.path = None
        
        
    def move_along_path(self):
        temp = self.path.pop(0)
        self.pos = Point(temp[0], temp[1])
        if len(self.path) == 0:
            return True

        return False
