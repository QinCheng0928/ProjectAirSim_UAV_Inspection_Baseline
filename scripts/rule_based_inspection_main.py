import os
import sys
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)

import asyncio
# from tasks.inspection import Inspection

# def main():
#     task = Inspection(
#         scene_file="scene_px4_gimbal.jsonc", 
#         start_intersection="J", 
#         display_images=True, 
#         save_images=False
#     )
#     asyncio.run(task.run())

from models.Manager import Manager
def main():
    manager = Manager(
        following_UAV_flag=False,
        following_UGV_flag=True,
        display_images=True, 
        save_images=False
    )
    asyncio.run(manager.run())


if __name__ == "__main__":
    main()