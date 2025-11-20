import asyncio
from inspection import InspectionManager

def main():
    manager = InspectionManager(
        scene_file="scene_px4_gimbal.jsonc", 
        start_intersection="J", 
        display_images=True, 
        save_images=True
    )
    asyncio.run(manager.run())

if __name__ == "__main__":
    main()