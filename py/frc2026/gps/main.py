import argparse
import asyncio
from pathlib import Path
from amiga_robot_controller import AmigaRobotController

# Run this in terminal with:
# python main.py --service-config=service_config.json

async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--service-config", type=Path, required=True)
    args = parser.parse_args()

    controller = AmigaRobotController(args.service_config)

    try:
        await asyncio.gather(
            controller.update_gps_task(),
            controller.start_mission()
        )
    except asyncio.CancelledError:
        pass
    
if __name__ == "__main__":
    asyncio.run(main())