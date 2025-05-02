import asyncio
import json
import time
import math
import struct
import base64
from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_websocket.types import ChannelId

async def main():
    class Listener(FoxgloveServerListener):
        async def on_subscribe(self, server: FoxgloveServer, channel_id: ChannelId):
            print("Client subscribed to", channel_id)
        async def on_unsubscribe(self, server: FoxgloveServer, channel_id: ChannelId):
            print("Client unsubscribed from", channel_id)

    # 1) Start the WebSocket server
    async with FoxgloveServer(
        host="0.0.0.0",
        port=8765,
        name="pointcloud server",
        capabilities=["clientPublish"],
        supported_encodings=["json"],
    ) as server:
        server.set_listener(Listener())

        # 2) Declare the /pointcloud channel with a PointCloud2 JSON schema
        chan_id: ChannelId = await server.add_channel({
            "topic": "/pointcloud",
            "encoding": "json",
            "schemaName": "sensor_msgs/msg/PointCloud2",
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
                                },
                                "required": ["sec", "nanosec"]
                            },
                            "frame_id": {"type": "string"}
                        },
                        "required": ["stamp", "frame_id"]
                    },
                    "height": {"type": "integer"},
                    "width": {"type": "integer"},
                    "fields": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "name":     {"type": "string"},
                                "offset":   {"type": "integer"},
                                "datatype": {"type": "integer"},
                                "count":    {"type": "integer"},
                            },
                            "required": ["name","offset","datatype","count"]
                        }
                    },
                    "is_bigendian": {"type": "boolean"},
                    "point_step":  {"type": "integer"},
                    "row_step":    {"type": "integer"},
                    "data": {
                        "type": "string",
                        "contentEncoding": "base64"
                    },
                    "is_dense": {"type": "boolean"}
                },
                "required": [
                    "header","height","width","fields",
                    "is_bigendian","point_step","row_step",
                    "data","is_dense"
                ]
            }),
        })

        # 3) Publish a rotating ring of points
        num_points = 200
        radius     = 1.0
        omega      = 0.5  # radians per second

        while True:
            ts = time.time_ns()
            t  = ts / 1e9

            # pack each (x,y,z) as float32 little-endian
            buf = bytearray()
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points + omega * t
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                z = 0.5 * math.sin(t)  # add a slow vertical oscillation
                buf.extend(struct.pack("<fff", x, y, z))

            b64 = base64.b64encode(buf).decode("ascii")

            msg = {
                "header": {
                    "stamp":    {"sec": ts // 1_000_000_000, "nanosec": ts % 1_000_000_000},
                    "frame_id": "map"
                },
                "height":      1,
                "width":       num_points,
                "fields": [
                    {"name":"x","offset":0, "datatype":7, "count":1},
                    {"name":"y","offset":4, "datatype":7, "count":1},
                    {"name":"z","offset":8, "datatype":7, "count":1}
                ],
                "is_bigendian": False,
                "point_step":   12,
                "row_step":     12 * num_points,
                "data":         b64,
                "is_dense":     False
            }

            await server.send_message(
                chan_id,
                ts,
                json.dumps(msg).encode("utf8"),
            )
            await asyncio.sleep(0.1)

if __name__ == "__main__":
    run_cancellable(main())
