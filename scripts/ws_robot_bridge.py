#!/usr/bin/env python3

import asyncio
import json
import math
import os
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
import websockets


DEFAULT_SERVER_URL = "ws://10.1.30.63:8765"


def yaw_to_quaternion(theta: float):
    half = theta / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


def normalize_goal_status(raw_status: str) -> Optional[str]:
    raw_status = raw_status.strip()

    if raw_status == "GOAL SUCCESS":
        return "success"

    if raw_status == "GOAL ABORTED":
        return "aborted"

    if raw_status == "GOAL CANCELLED":
        return "cancelled"

    if raw_status == "GOAL REJECTED":
        return "rejected"

    if raw_status.startswith("GOAL STATUS"):
        return "unknown"

    return None


class WebSocketRobotBridge(Node):
    def __init__(self, robot_id: str, loop: asyncio.AbstractEventLoop, outgoing: asyncio.Queue):
        super().__init__("ws_robot_bridge")

        self.robot_id = robot_id
        self.loop = loop
        self.outgoing = outgoing

        self.active_goal_id: Optional[str] = None
        self.active_node_id: Optional[str] = None
        self.active_goal_sent_at: Optional[float] = None

        self.goal_pub = self.create_publisher(PoseStamped, "/goal", 10)

        self.status_sub = self.create_subscription(
            String,
            "/goal/status",
            self.goal_status_callback,
            10,
        )

        self.get_logger().info(f"WebSocket robot bridge started, robot_id={robot_id}")

    def publish_goal(self, command: dict):
        goal_id = str(command.get("goal_id", f"{self.robot_id}_{int(time.time())}"))
        node_id = str(command.get("node_id", ""))

        x = float(command["x"])
        y = float(command["y"])
        theta = float(command.get("theta", 0.0))

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(theta)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.active_goal_id = goal_id
        self.active_node_id = node_id
        self.active_goal_sent_at = time.time()

        self.goal_pub.publish(msg)

        self.get_logger().info(
            f"Published goal: "
            f"goal_id={goal_id}, node_id={node_id}, "
            f"x={x:.3f}, y={y:.3f}, theta={theta:.3f}"
        )

        self.send_to_server({
            "type": "goal_accepted_by_bridge",
            "robot_id": self.robot_id,
            "goal_id": goal_id,
            "node_id": node_id,
            "x": x,
            "y": y,
            "theta": theta,
            "time": time.time(),
        })

    def goal_status_callback(self, msg: String):
        raw_status = msg.data.strip()
        final_status = normalize_goal_status(raw_status)

        goal_id = self.active_goal_id
        node_id = self.active_node_id

        if final_status is None:
            self.send_to_server({
                "type": "goal_feedback",
                "robot_id": self.robot_id,
                "goal_id": goal_id,
                "node_id": node_id,
                "raw_status": raw_status,
                "time": time.time(),
            })
            return

        self.send_to_server({
            "type": "goal_status",
            "robot_id": self.robot_id,
            "goal_id": goal_id,
            "node_id": node_id,
            "status": final_status,
            "raw_status": raw_status,
            "time": time.time(),
        })

        self.get_logger().info(
            f"Goal finished: "
            f"goal_id={goal_id}, node_id={node_id}, status={final_status}"
        )

        self.active_goal_id = None
        self.active_node_id = None
        self.active_goal_sent_at = None

    def send_to_server(self, payload: dict):
        self.loop.call_soon_threadsafe(self.outgoing.put_nowait, payload)


async def sender_loop(ws, outgoing: asyncio.Queue):
    while True:
        payload = await outgoing.get()
        await ws.send(json.dumps(payload, ensure_ascii=False))


async def handle_command(node: WebSocketRobotBridge, ws, data: dict):
    command_type = data.get("type")

    if command_type == "goal":
        try:
            node.publish_goal(data)
        except Exception as e:
            await ws.send(json.dumps({
                "type": "goal_error",
                "robot_id": node.robot_id,
                "goal_id": data.get("goal_id"),
                "node_id": data.get("node_id"),
                "error": str(e),
                "time": time.time(),
            }, ensure_ascii=False))
        return

    if command_type == "ping":
        await ws.send(json.dumps({
            "type": "pong",
            "robot_id": node.robot_id,
            "time": time.time(),
        }, ensure_ascii=False))
        return

    await ws.send(json.dumps({
        "type": "unknown_command",
        "robot_id": node.robot_id,
        "command": data,
        "time": time.time(),
    }, ensure_ascii=False))


async def websocket_loop(node: WebSocketRobotBridge, server_url: str, outgoing: asyncio.Queue):
    reconnect_delay = 2.0

    while rclpy.ok():
        sender_task = None

        try:
            print(f"[ws] connecting to {server_url}")

            async with websockets.connect(
                server_url,
                ping_interval=20,
                ping_timeout=20,
            ) as ws:
                print(f"[ws] connected as {node.robot_id}")

                await ws.send(json.dumps({
                    "type": "hello",
                    "robot_id": node.robot_id,
                    "time": time.time(),
                }, ensure_ascii=False))

                sender_task = asyncio.create_task(sender_loop(ws, outgoing))

                async for message in ws:
                    try:
                        data = json.loads(message)
                    except json.JSONDecodeError:
                        await ws.send(json.dumps({
                            "type": "bad_json",
                            "robot_id": node.robot_id,
                            "message": message,
                            "time": time.time(),
                        }, ensure_ascii=False))
                        continue

                    await handle_command(node, ws, data)

        except asyncio.CancelledError:
            raise

        except Exception as e:
            print(f"[ws] disconnected: {e}")
            print(f"[ws] reconnect in {reconnect_delay:.1f} sec")
            await asyncio.sleep(reconnect_delay)

        finally:
            if sender_task is not None:
                sender_task.cancel()


async def main_async():
    robot_id = os.environ.get("ROBOT_ID", "robot_1")
    server_url = os.environ.get("MAPF_SERVER_URL", DEFAULT_SERVER_URL)

    outgoing = asyncio.Queue()
    loop = asyncio.get_running_loop()

    rclpy.init()
    node = WebSocketRobotBridge(robot_id=robot_id, loop=loop, outgoing=outgoing)

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    spin_thread.start()

    try:
        await websocket_loop(node, server_url, outgoing)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
