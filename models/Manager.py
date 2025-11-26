from projectairsim import ProjectAirSimClient, World
from projectairsim.utils import projectairsim_log
from models.InspectionUAVController import InspectionUAVController
from models.FollowingUAVCotroller import FollowingUAVControll
from models.FollowingUGVController import FollowingUGVControll
from components.image_handler import ImageHandler

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
            num_subwim=4,
            display_enabled=display_images, 
            save_enabled=save_images
            )
        self.image_handler.start()

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

    async def start(self):
        projectairsim_log().info("Inspection_UAV Starting takeoff...")
        takeoff_task = await self.inspection_UAV.drone.takeoff_async()
        await takeoff_task
        projectairsim_log().info("Inspection_UAV Take off completed.")
        self.inspection_UAV.update_cur_position()

        if self.following_UAV is not None: 
            projectairsim_log().info("Inspection_UAV Starting takeoff...")
            takeoff_task = await self.inspection_UAV.drone.takeoff_async()
            await takeoff_task
            projectairsim_log().info("Inspection_UAV Take off completed.")

        # start the image display if needed
        self.image_handler.start()

    async def shutdown(self):
        projectairsim_log().info("Inspection_UAV Starting Land...")
        land_task = await self.inspection_UAV.drone.land_async()
        await land_task
        projectairsim_log().info("Inspection_UAV Land completed.")    
        self.inspection_UAV.shutdown()  
    
        if self.following_UAV is not None: 
            projectairsim_log().info("Following_UAV Starting Land...")
            land_task = await self.following_UAV.drone.land_async()
            await land_task
            projectairsim_log().info("Following_UAV Land completed.")    
            self.following_UAV.shutdown()
        if self.following_UGV is not None:
            rover_task = await self.following_UGV.rover.set_rover_controls(
                engine=0.0, steering_angle=0.0, brake=0.0
            )
            await rover_task
            self.following_UGV.shutdown()

    async def step(self):
        await self.inspection_UAV.step()
        if self.following_UAV is not None:
            await self.following_UAV.step()
        if self.following_UGV is not None:
            await self.following_UGV.step()  

    async def run(self):
        try:
            await self.start()
            while not self.inspection_UAV.collision_state.collision:
                await self.step()
                if self.inspection_UAV.has_arrived():
                    self.inspection_UAV.update_from_and_to_position()
                    projectairsim_log().info(f"New target intersection ID: {self.target_id}, coordinates: {self.to_position}")
        except KeyboardInterrupt:
            projectairsim_log().info("Interrupted by user, shutting down.")
        finally:
            await self.shutdown()
            self.client.disconnect()