"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

from . import *
from lcm_msgs import std_msgs
from .Point import Point
class PointStamped(object):

    __slots__ = ["header", "point"]

    __typenames__ = ["std_msgs.Header", "Point"]

    __dimensions__ = [None, None]

    def __init__(self):
        self.header = std_msgs.Header()
        """ LCM Type: std_msgs.Header """
        self.point = Point()
        """ LCM Type: Point """

    def encode(self):
        buf = BytesIO()
        buf.write(PointStamped._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.header._get_packed_fingerprint() == std_msgs.Header._get_packed_fingerprint()
        self.header._encode_one(buf)
        assert self.point._get_packed_fingerprint() == Point._get_packed_fingerprint()
        self.point._encode_one(buf)

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != PointStamped._get_packed_fingerprint():
            raise ValueError("Decode error")
        return PointStamped._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = PointStamped()
        self.header = std_msgs.Header._decode_one(buf)
        self.point = Point._decode_one(buf)
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if PointStamped in parents: return 0
        newparents = parents + [PointStamped]
        tmphash = (0xf012413a2c8028c2+ std_msgs.Header._get_hash_recursive(newparents)+ Point._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if PointStamped._packed_fingerprint is None:
            PointStamped._packed_fingerprint = struct.pack(">Q", PointStamped._get_hash_recursive([]))
        return PointStamped._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", PointStamped._get_packed_fingerprint())[0]

