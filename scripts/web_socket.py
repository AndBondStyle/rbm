import asyncio
import json
import websockets

SERVER_URL = "ws://10.1.30.63:8765"
ROBOT_ID = "robot_1"

async def main():
    async with websockets.connect(SERVER_URL) as ws:
        await ws.send(json.dumps({
            "type": "hello",
            "robot_id": ROBOT_ID
        }))

        await ws.send(json.dumps({
            "type": "state",
            "robot_id": ROBOT_ID,
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,
            "status": "idle"
        }))

        async for msg in ws:
            data = json.loads(msg)
            print("command from server:", data)

if __name__ == "__main__":
    asyncio.run(main())