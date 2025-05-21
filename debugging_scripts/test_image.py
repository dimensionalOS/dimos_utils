import asyncio
import json
import time
import base64
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_websocket.types import ChannelId

async def main():
    async with FoxgloveServer(
        host="0.0.0.0",
        port=8765,
        name="image server",
        capabilities=["clientPublish"],
        supported_encodings=["json"],
    ) as server:
        # 1) Define the CompressedImage channel
        chan_id: ChannelId = await server.add_channel({
            "topic": "/image_raw",
            "encoding": "json",
            "schemaName": "sensor_msgs/msg/CompressedImage",
            "schemaEncoding": "jsonschema",
            "schema": json.dumps({
                "type": "object",
                "properties": {
                    "header": {
                        "type": "object",
                        "properties": {
                            "stamp": {
                                "type": "object",
                                "properties": {
                                    "sec":   {"type": "integer"},
                                    "nanosec": {"type": "integer"}
                                }
                            },
                            "frame_id": {"type": "string"}
                        }
                    },
                    "format": {"type": "string"},
                    "data": {
                        "type": "string",
                        "contentEncoding": "base64"
                    }
                }
            }),
        })

        # 2) Read and base64-encode the PNG file once
        with open("zen.png", "rb") as f:
            png_data = f.read()
        b64_data = base64.b64encode(png_data).decode("ascii")

        # 3) Send it in a loop (or just once)
        while True:
            ts = time.time_ns()
            msg = {
                "header": {
                    "stamp": {"sec": ts // 1_000_000_000, "nanosec": ts % 1_000_000_000},
                    "frame_id": "camera"
                },
                "format": "png",
                "data": b64_data,
            }
            await server.send_message(
                chan_id,
                ts,
                json.dumps(msg).encode("utf8")
            )
            await asyncio.sleep(0.1)

if __name__ == "__main__":
    run_cancellable(main())
