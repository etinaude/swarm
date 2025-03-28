import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pybullet as p
import pybullet_data
import time

class BulletSim(Node):
    def __init__(self):
        super().__init__('bullet_sim')

        # Initialize PyBullet
        self.physicsClient = p.connect(p.GUI)  # Use p.DIRECT for headless
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Load default assets
        p.setGravity(0, 0, -9.81)

        # Load a simple box as the robot
        self.robot = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 1])

        # Subscribe to Twist messages
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        """Apply velocity from Twist message to Bullet object"""
        linear_vel = [msg.linear.x, msg.linear.y, msg.linear.z]
        angular_vel = [msg.angular.x, msg.angular.y, msg.angular.z]

        # Apply linear velocity
        p.resetBaseVelocity(self.robot, linearVelocity=linear_vel, angularVelocity=angular_vel)

    def run_simulation(self):
        """Run the physics loop"""
        while rclpy.ok():
            p.stepSimulation()
            time.sleep(1./240.)  # Bullet default timestep

def main(args=None):
    rclpy.init(args=args)
    sim = BulletSim()
    sim.run_simulation()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
