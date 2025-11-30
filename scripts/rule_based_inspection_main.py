import os
import sys
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)

import asyncio
from models.Manager import Manager

def main():
    manager = Manager(
        following_UAV_flag=True,
        following_UGV_flag=False,
        display_images=True, 
        save_images=False
    )
    asyncio.run(manager.run())


if __name__ == "__main__":
    main()