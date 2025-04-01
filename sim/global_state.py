class State:
    def __init__(self, gluers, brick_pile, house, screen, sim_speed=1):
        self.gluers = gluers
        self.brick_pile = brick_pile
        self.loose_bricks = []
        self.house = house
        self.canidate_bricks = []
        self.screen = screen
        self.sim_speed = sim_speed

    def add_brick(self, brick):
        self.loose_bricks.append(brick)
