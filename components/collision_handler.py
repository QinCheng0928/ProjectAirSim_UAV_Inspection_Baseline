from projectairsim.utils import projectairsim_log

class CollisionHandler:
    def __init__(self):
        self.collision = False

    def collision_callback(self, value=True):
        projectairsim_log().info("Drone collision.")
        self.collision = value