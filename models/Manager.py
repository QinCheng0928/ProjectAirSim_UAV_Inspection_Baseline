from projectairsim import ProjectAirSimClient, World
from projectairsim.utils import projectairsim_log
from models.InspectionUAVController import InspectionUAVController
from models.FollowingUAVCotroller import FollowingUAVControll
from models.FollowingUGVController import FollowingUGVControll
from components.image_handler import ImageHandler
import asyncio
import time

async def brake_rover(rover, brake=0.5):
    rover_task = await rover.set_rover_controls(
        engine=0.0, steering_angle=0.0, brake=brake
    )
    await rover_task

    rover_task = await rover.set_rover_controls(
        engine=0.0, steering_angle=0.0, brake=0.0
    )
    await rover_task
class Manager:
    def __init__(
            self,
            following_UAV_flag=False,
            following_UGV_flag=False,
            display_images=True, 
            save_images=False

    ):
        self.client = ProjectAirSimClient()
        self.client.connect()
        self.world = World(self.client, "scene_inspection.jsonc", delay_after_load_sec=2)

        self.image_handler = ImageHandler(
            client=self.client, 
            num_subwim=5,
            display_enabled=display_images, 
            save_enabled=save_images
            )

        self.inspection_UAV = InspectionUAVController(
            self.client,
            self.world,
            image_handler=self.image_handler,      
            name = "Drone1",
            start_intersection="J",
        )
        self.following_UAV = None
        self.following_UGV = None
        if following_UAV_flag:
            self.following_UAV = FollowingUAVControll(
                self.client,
                self.world,
                self.inspection_UAV,
                image_handler=self.image_handler,   
                name="Drone2",  
            )
        if following_UGV_flag:
            self.following_UGV = FollowingUGVControll(
                self.client,
                self.world,
                self.inspection_UAV,
                image_handler=self.image_handler,   
                name="Rover1", 
            )
        self.image_handler.start()

    async def start(self):
        tasks = []
        projectairsim_log().info("Inspection_UAV Starting takeoff...")
        takeoff_task = self.inspection_UAV.drone.takeoff_async()
        tasks.append(takeoff_task) 
        if self.following_UAV is not None: 
            projectairsim_log().info("Following_UAV Starting takeoff...")
            takeoff_task = self.inspection_UAV.drone.takeoff_async()
            tasks.append(takeoff_task)
        if tasks:
            await asyncio.gather(*tasks)
            projectairsim_log().info("Take off completed.")

        # start the image display if needed
        self.image_handler.start()

    async def shutdown(self):
        tasks = []
        projectairsim_log().info("Inspection_UAV Starting Land...")
        land_task = self.inspection_UAV.drone.land_async()
        tasks.append(land_task) 
        if self.following_UAV is not None: 
            projectairsim_log().info("Following_UAV Starting Land...")
            land_task = self.following_UAV.drone.land_async()
            tasks.append(land_task)
        if self.following_UGV is not None:
            brake_task = brake_rover(self.following_UGV.rover)
            tasks.append(brake_task)
        if tasks:
            await asyncio.gather(*tasks)
            projectairsim_log().info("Land and Brake completed.")

        self.inspection_UAV.shutdown()  
        if self.following_UAV is not None: 
            self.following_UAV.shutdown()
        if self.following_UGV is not None:
            self.following_UGV.shutdown()

    async def step(self):
        tasks = []
        inspection_task = self.inspection_UAV.step()
        await inspection_task
        if self.following_UAV is not None:
            tasks.append(self.following_UAV.step())
        if self.following_UGV is not None:
            tasks.append(self.following_UGV.step())
        
        await asyncio.gather(*tasks)

    async def run(self):
        try:
            await self.start()
            while not self.inspection_UAV.collision_state.collision:
                start_total = time.time()
                
                step_start = time.time()
                await self.step()
                step_time = time.time() - step_start
                
                # 测量 has_arrived() 执行时间
                has_arrived_start = time.time()
                has_arrived_result = self.inspection_UAV.has_arrived()
                has_arrived_time = time.time() - has_arrived_start
                
                # 测量 update_from_and_to_position() 执行时间
                update_start = time.time()
                if has_arrived_result:
                    self.inspection_UAV.update_from_and_to_position()
                update_time = time.time() - update_start if has_arrived_result else 0
                
                total_time = time.time() - start_total
                
                print(f"Step time: {step_time:.6f}s | Has_arrived: {has_arrived_time:.6f}s | Update: {update_time:.6f}s | Total: {total_time:.6f}s ")

            projectairsim_log().info("While break because of collision.")
        except KeyboardInterrupt:
            projectairsim_log().info("Interrupted by user, shutting down.")
        finally:
            await asyncio.sleep(0.1)
            await self.shutdown()
            self.image_handler.stop()
            self.client.disconnect()