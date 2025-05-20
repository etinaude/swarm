from direct.showbase.ShowBase import ShowBase
from panda3d.core import Vec3

class Robot:
    def __init__(self, model_path, start_pos, render, name):
        # Load and set up the robot model
        self.node = loader.loadModel(model_path)
        self.node.reparentTo(render)
        self.node.setPos(start_pos)
        self.name = name

        # Movement direction and speed
        self.direction = Vec3(1, 0, 0)  # Move along X axis
        self.speed = 1.0  # units per second

    def update(self, dt):
        # Move in a straight line
        delta = self.direction * self.speed * dt
        self.node.setPos(self.node.getPos() + delta)

class MyApp(ShowBase):
    def __init__(self):
        super().__init__()

        # List of robot objects
        self.robots = [
            Robot("assests/lifter.gltf", start_pos=(0, 10, 0), render=self.render, name="R1"),
            Robot("assests/rover.gltf", start_pos=(5, 10, 0), render=self.render, name="R2"),
        ]

        # Add update task
        self.taskMgr.add(self.update_robots, "UpdateRobots")

    def update_robots(self, task):
        dt = globalClock.getDt()

        for robot in self.robots:
            robot.update(dt)

        return task.cont

app = MyApp()
app.run()