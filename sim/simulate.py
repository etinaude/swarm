import random
from global_state import State
from specs import rover_count, gluer_count, lifter_count, map_size
from brick import Brick
from bots.house import House
from bots.gluer import Gluer
from bots.rover import Rover
from bots.lifter import Lifter
from shapely.geometry import Point
import numpy as np
from direct.showbase.ShowBase import ShowBase
from panda3d.core import AmbientLight, DirectionalLight, Point3

sim_speed = 2

house = House()

brick_pile = Point(10, 10)

pile = Brick(
    brick_pile.x,
    brick_pile.y,
    0,
    0,
)

# def draw():
#     house.draw_goal()
#     house.draw()
#     pile.draw()

#     for brick in state.loose_bricks:
#         brick.draw()

#     for rover in rovers:
#         rover.draw()

#     for gluer in gluers:
#         gluer.draw()

#     for lifter in lifters:
#         lifter.draw()

    # pygame.display.flip()
    # clock.tick(10)

# def make_rover_move():
#     with concurrent.futures.ThreadPoolExecutor() as executor:
#         executor.map(lambda rover: rover.make_move(gluers, lifters), rovers)


# def step():
#     make_rover_move()

#     for lifter in lifters:
#         lifter.make_move(state.loose_bricks, house)

#     for gluer in gluers:
#         gluer.update()

#     draw()



class Simulation(ShowBase):
    def __init__(self):
        super().__init__()
        self.speed = 1.0

        self.lifters = []
        
        self.rovers = []
        self.gluers = []
        self.house = House()
        self.pile = None

        self.state = State(self.gluers, self.pile, self.house, self.speed)


        self.pile = self.loader.load_model("assests/brick pile.gltf") 
        self.pile.reparent_to(self.render)
        self.pile.set_scale(5)
        self.pile.set_pos(0, 0, 0)


        for i in range(rover_count):
            x = random.randint(0, 50)
            y = random.randint(0, 50)

            robot = Rover(start_pos=(0, 10, 0), render=self.render, state=self.state)

            self.rovers.append(robot)

        # for i in range(lifter_count):
        #     x = random.randint(0, 50)
        #     y = random.randint(0, 20)

        #     robot = self.loader.load_model("assests/lifter.gltf") 
        #     robot.reparent_to(self.render)
        #     robot.set_scale(10)
        #     robot.set_pos(50, 0, 0)
        #     self.lifters.append(robot)

        # for i in range(gluer_count):
        #     x = random.randint(0, 5)
        #     y = random.randint(0, 20)

        #     robot = self.loader.load_model("assests/glue.gltf") 
        #     robot.reparent_to(self.render)
        #     robot.set_pos(x, y, 0)
        #     robot.set_scale(10)

        #     self.gluers.append(robot)




        # Add some basic lighting
        ambient = AmbientLight("ambient")
        ambient.set_color((0.9, 0.9, 0.9, 1))
        ambient_node = self.render.attach_new_node(ambient)
        self.render.set_light(ambient_node)

        # directional = DirectionalLight("directional")
        # directional.set_color((0.8, 0.8, 0.8, 1))
        # directional_node = self.render.attach_new_node(directional)
        # directional_node.set_hpr(-45, -45, 0)
        # self.render.set_light(directional_node)

        # Enable basic camera controls
        self.disable_mouse()
        self.camera.set_pos(25,25,100)
        self.camera.look_at(Point3(25, 25, 30))
        self.taskMgr.add(self.move_rovers, "MoveRovers")


    def move_rovers(self, task):
        # dt = globalClock.getDt()  # Time since last frame
        # # self.lifters[0].setY(self.lifters[0], self.speed * dt)  # Move forward along its Y-axis
        
        for rover in self.rovers:
            rover.make_move(self.gluers, self.lifters)

        return task.cont



if __name__ == "__main__":
    app = Simulation()
    app.run()





