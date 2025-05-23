"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

from lcm_msgs import geometry_msgs
from . import *
from .Color import Color
class CylinderPrimitive(object):

    __slots__ = ["pose", "size", "bottom_scale", "top_scale", "color"]

    __typenames__ = ["geometry_msgs.Pose", "geometry_msgs.Vector3", "double", "double", "Color"]

    __dimensions__ = [None, None, None, None, None]

    def __init__(self):
        self.pose = geometry_msgs.Pose()
        """ LCM Type: geometry_msgs.Pose """
        self.size = geometry_msgs.Vector3()
        """ LCM Type: geometry_msgs.Vector3 """
        self.bottom_scale = 0.0
        """ LCM Type: double """
        self.top_scale = 0.0
        """ LCM Type: double """
        self.color = Color()
        """ LCM Type: Color """

    def encode(self):
        buf = BytesIO()
        buf.write(CylinderPrimitive._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.pose._get_packed_fingerprint() == geometry_msgs.Pose._get_packed_fingerprint()
        self.pose._encode_one(buf)
        assert self.size._get_packed_fingerprint() == geometry_msgs.Vector3._get_packed_fingerprint()
        self.size._encode_one(buf)
        buf.write(struct.pack(">dd", self.bottom_scale, self.top_scale))
        assert self.color._get_packed_fingerprint() == Color._get_packed_fingerprint()
        self.color._encode_one(buf)

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != CylinderPrimitive._get_packed_fingerprint():
            raise ValueError("Decode error")
        return CylinderPrimitive._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = CylinderPrimitive()
        self.pose = geometry_msgs.Pose._decode_one(buf)
        self.size = geometry_msgs.Vector3._decode_one(buf)
        self.bottom_scale, self.top_scale = struct.unpack(">dd", buf.read(16))
        self.color = Color._decode_one(buf)
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if CylinderPrimitive in parents: return 0
        newparents = parents + [CylinderPrimitive]
        tmphash = (0xa52103034bfe0bac+ geometry_msgs.Pose._get_hash_recursive(newparents)+ geometry_msgs.Vector3._get_hash_recursive(newparents)+ Color._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if CylinderPrimitive._packed_fingerprint is None:
            CylinderPrimitive._packed_fingerprint = struct.pack(">Q", CylinderPrimitive._get_hash_recursive([]))
        return CylinderPrimitive._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", CylinderPrimitive._get_packed_fingerprint())[0]

