import math
from projectairsim import Rover
from projectairsim.utils import projectairsim_log
from config.settings import MOVE_VELOCITY, SMOOTHING_FACTOR

class FollowingUGVControll:
    def __init__(
            self,
            client,
            world,
            followed_UAV,
            image_handler,
            name="Rover1",  
        ):
        self.client = client
        self.world=world
        self.rover = Rover(client, world, name)
        self.rover.enable_api_control()
        self.rover.arm()

        self.prev_heading = 0
        self.heading = 0

        self.image_handler = image_handler
        # setup subscriptions and displays
        # self._setup_subscriptions()

        self.followed_UAV=followed_UAV
        projectairsim_log().info("Rover init completed.")


    def _setup_subscriptions(self):
        # windows
        chase_cam = "RoverChaseCam"
        self.image_handler.register_window(chase_cam, 3)
        # subscribe cameras
        self.image_handler.subscribe_camera(self.rover.sensors["Chase"]["scene_camera"], chase_cam)

    def _compute_heading(self):
        to_x, to_y, _ = self.followed_UAV.cur_position

        kinematics = self.rover.get_ground_truth_kinematics()
        position = kinematics['pose']['position']
        cur_x, cur_y = position["x"], position["y"]
        dx = to_x - cur_x
        dy = to_y - cur_y
        
        heading = math.atan2(dy, dx)

        self.heading = self.prev_heading * (1 - SMOOTHING_FACTOR) + heading * SMOOTHING_FACTOR
        self.prev_heading = self.heading

    async def step(self):
        self._compute_heading()
        task = self.rover.move_by_heading_async(heading=self.heading, speed=MOVE_VELOCITY / 50, duration=1)
        await task


    def shutdown(self):
        self.rover.cancel_last_task()
        self.rover.disarm()
        self.rover.disable_api_control()
