import os
import sys
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)

import asyncio
from tasks.inspection import Inspection

def main():
    inspection = Inspection(
        scene_file="scene_px4_gimbal.jsonc", 
        start_intersection="J", 
        display_images=True, 
        save_images=False
    )
    asyncio.run(inspection.run())

if __name__ == "__main__":
    main()