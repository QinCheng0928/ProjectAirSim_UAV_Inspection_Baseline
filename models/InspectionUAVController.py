import math
import numpy as np
from projectairsim import Drone
from projectairsim.utils import projectairsim_log, quaternion_to_rpy, unpack_image
from projectairsim.drone import YawControlMode
from projectairsim.types import ImageType

from models.roadnetwork import RoadNetwork
from components.collision_handler import CollisionHandler
from config.settings import (MOVE_VELOCITY, TARGET_DISTANCE_THRESHOLD, 
                           CHANGE_DIRECTION_DISTANCE_THRESHOLD, SMOOTHING_FACTOR)
import time

class InspectionUAVController:
    def __init__(
            self, 
            client, 
            world,
            image_handler,
            name = "Drone1",
            start_intersection="J",
            ):
        self.client = client
        self.world = world
        self.drone = Drone(self.client, self.world, name)
        self.drone.enable_api_control()
        self.drone.arm()

        self.image_handler = image_handler
        self.collision_state = CollisionHandler()

        # road network and navigation
        self.roadnetwork = RoadNetwork()
        self.target_distance_threshold = TARGET_DISTANCE_THRESHOLD
        self.change_direction_distance_threshold = CHANGE_DIRECTION_DISTANCE_THRESHOLD

        # smoothing state
        self.prev_v_north = 0.0
        self.prev_v_east = 0.0
        self.smoothing_factor = SMOOTHING_FACTOR
        self._last_kinematics_time = 0.0

        # positions and targets
        self.cur_id = start_intersection
        self._init_positions()

        # setup subscriptions and displays
        self._setup_subscriptions()

    def _init_positions(self):
        self.update_cur_position()
        self.from_position = self.cur_position
        self.target_id = self.roadnetwork.random_neighbor(self.cur_id)
        self.target_id = "D"
        if self.target_id is None:
            raise RuntimeError(f"No neighbor found for start node {self.cur_id}")
        coords = self.roadnetwork.coords(self.target_id)
        if coords is None:
            raise RuntimeError(f"Coordinates for {self.target_id} not found")
        self.to_position = coords

    def _setup_subscriptions(self):
        # windows
        chase_cam = "ChaseCam"
        front_cam = "FrontCam"
        depth_cam = "FrontDepthImage"
        self.image_handler.register_window(chase_cam, 0)
        self.image_handler.register_window(front_cam, 1)
        self.image_handler.register_window(depth_cam, 2)

        # subscribe cameras
        self.image_handler.subscribe_camera(self.drone.sensors["Chase"]["scene_camera"], chase_cam)
        self.image_handler.subscribe_camera(self.drone.sensors["front_center"]["scene_camera"], front_cam)
        self.image_handler.subscribe_camera(self.drone.sensors["front_center"]["depth_planar_camera"], depth_cam)

        # collision topic
        self.client.subscribe(
            self.drone.robot_info["collision_info"],
            lambda topic, msg: self.collision_state.collision_callback(True)
        )

    def update_cur_position(self):
        self.kinematics = self.drone.get_ground_truth_kinematics()
        pos = self.kinematics['pose']['position']
        self.cur_position = (pos['x'], pos['y'], pos['z'])

    def update_from_and_to_position(self):
        self.from_position = self.cur_position
        next_target = self.roadnetwork.random_neighbor(self.target_id)
        while next_target == self.cur_id:
            next_target = self.roadnetwork.random_neighbor(self.target_id)
            if next_target is None:
                projectairsim_log().warning("No further neighbors from current target; staying at current target.")
                return
        self.cur_id = self.target_id
        self.target_id = next_target
        self.target_id = "E"
        self.to_position = self.roadnetwork.coords(self.target_id)
        projectairsim_log().info(f"New target intersection ID: {self.target_id}, coordinates: {self.to_position}")

    def has_arrived(self):
        current_pos = np.array(self.cur_position)
        target_pos = np.array(self.to_position)
        distance = np.linalg.norm(current_pos - target_pos)
        return distance < self.target_distance_threshold

    def _compute_obstacle_avoidance(self):
        images = self.drone.get_images("front_center", [ImageType.DEPTH_PLANAR])
        depth_image = unpack_image(images[ImageType.DEPTH_PLANAR])
        
        if depth_image is None:
            projectairsim_log().warning("No depth image available; defaulting to forward small movement.")
            return (0.1, 0.0, self.to_position[2])
        
        num_bands = 3
        height, width = depth_image.shape
        middle_start = height // 3
        middle_end = 2 * height // 3
        middle_band = depth_image[middle_start:middle_end, :]
        
        band_width = width // num_bands
        band_depths = []
        for i in range(num_bands):
            start_col = i * band_width
            end_col = (i + 1) * band_width
            band = middle_band[:, start_col:end_col]
            band_depths.append(np.min(band))

        center_index = num_bands // 2
        projectairsim_log().info(f"Band depths (mm): {band_depths}")

        # obstacle detection
        front_min = band_depths[center_index]
        if front_min < self.change_direction_distance_threshold:
            projectairsim_log().info("Obstacle detected ahead.")
            safe_index = int(np.argmax(band_depths))
            projectairsim_log().info(f"Steering toward band index: {safe_index}")

            angle_per_band = (math.pi / 2) / num_bands
            angle_offset = (safe_index - center_index) * angle_per_band

            orientation = self.kinematics['pose']['orientation']
            roll, pitch, yaw = quaternion_to_rpy(orientation["w"], orientation["x"], 
                                               orientation["y"], orientation["z"])
            abs_yaw = yaw + angle_offset
            v_north = MOVE_VELOCITY * math.cos(abs_yaw)
            v_east = MOVE_VELOCITY * math.sin(abs_yaw)
            z = self.to_position[2]
        else:
            # go toward target
            cur_x, cur_y, cur_z = self.cur_position
            to_x, to_y, to_z = self.to_position
            dx = to_x - cur_x
            dy = to_y - cur_y
            distance = math.hypot(dx, dy)
            unit_dx = dx / (distance + 1e-6)
            unit_dy = dy / (distance + 1e-6)
            v_north = unit_dx * MOVE_VELOCITY
            v_east = unit_dy * MOVE_VELOCITY
            z = to_z

        # smoothing
        v_north = self.prev_v_north * (1 - self.smoothing_factor) + v_north * self.smoothing_factor
        v_east = self.prev_v_east * (1 - self.smoothing_factor) + v_east * self.smoothing_factor
        self.prev_v_north, self.prev_v_east = v_north, v_east
        projectairsim_log().info(f"Computed velocities - North: {v_north:.2f}, East: {v_east:.2f}, Z: {z:.2f}")

        return v_north, v_east, z

    async def step(self):
        v_north, v_east, z = self._compute_obstacle_avoidance()
        task = self.drone.move_by_velocity_z_async(v_north, v_east, z, 1, yaw_control_mode=YawControlMode.ForwardOnly, yaw_is_rate=False, yaw=0)
        await task
        self.update_cur_position()

    def shutdown(self):
        """Clean up resources: stop image handler and disconnect drone client."""
        self.drone.cancel_last_task()
        self.drone.disarm()
        self.drone.disable_api_control()
