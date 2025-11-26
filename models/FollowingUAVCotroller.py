import math
from projectairsim import Drone
from projectairsim.drone import YawControlMode
from projectairsim.utils import projectairsim_log, quaternion_to_rpy
from config.settings import MOVE_VELOCITY, SMOOTHING_FACTOR

class FollowingUAVControll:
    def __init__(
            self,
            client,
            world,
            followed_UAV,
            image_handler,
            name="Drone2",  
        ):
        self.client = client
        self.world=world
        self.drone = Drone(client, world, name)
        self.drone.enable_api_control()
        self.drone.arm()

        self.prev_v_north = 0.0
        self.prev_v_east = 0.0

        self.image_handler = image_handler
        
        self.followed_UAV=followed_UAV

    def _compute_xy(self):
        to_x, to_y, to_z = self.followed_UAV.cur_position

        kinematics = self.drone.get_ground_truth_kinematics()
        position = kinematics['pose']['position']
        cur_x, cur_y = position["x"], position["y"]
        dx = to_x - cur_x
        dy = to_y - cur_y
        distance = math.hypot(dx, dy)
        unit_dx = dx / (distance + 1e-6)
        unit_dy = dy / (distance + 1e-6)
        v_north = unit_dx * MOVE_VELOCITY
        v_east = unit_dy * MOVE_VELOCITY
        z = to_z

        v_north = self.prev_v_north * (1 - SMOOTHING_FACTOR) + v_north * SMOOTHING_FACTOR
        v_east = self.prev_v_east * (1 - SMOOTHING_FACTOR) + v_east * SMOOTHING_FACTOR
        self.prev_v_north, self.prev_v_east = v_north, v_east

        return (v_north, v_east, z)

    async def step(self):
        v_north, v_east, z = self._compute_xy()
        task = self.drone.move_by_velocity_z_async(v_north, v_east, z, 1, yaw_control_mode=YawControlMode.ForwardOnly, yaw_is_rate=False, yaw=0)
        await task

    def shutdown(self):
        self.drone.disarm()
        self.drone.disable_api_control()
