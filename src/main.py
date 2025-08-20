import asyncio
from src.fsm import MissionFSM

async def main():
    fsm = MissionFSM()
    running = True
    while running:
        running = await fsm.step()

if __name__ == "__main__":
    asyncio.run(main())
