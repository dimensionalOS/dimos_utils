"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

from . import *
from lcm_msgs import std_msgs
from .Polygon import Polygon
class PolygonStamped(object):

    __slots__ = ["header", "polygon"]

    __typenames__ = ["std_msgs.Header", "Polygon"]

    __dimensions__ = [None, None]

    def __init__(self):
        self.header = std_msgs.Header()
        """ LCM Type: std_msgs.Header """
        self.polygon = Polygon()
        """ LCM Type: Polygon """

    def encode(self):
        buf = BytesIO()
        buf.write(PolygonStamped._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.header._get_packed_fingerprint() == std_msgs.Header._get_packed_fingerprint()
        self.header._encode_one(buf)
        assert self.polygon._get_packed_fingerprint() == Polygon._get_packed_fingerprint()
        self.polygon._encode_one(buf)

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != PolygonStamped._get_packed_fingerprint():
            raise ValueError("Decode error")
        return PolygonStamped._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = PolygonStamped()
        self.header = std_msgs.Header._decode_one(buf)
        self.polygon = Polygon._decode_one(buf)
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if PolygonStamped in parents: return 0
        newparents = parents + [PolygonStamped]
        tmphash = (0x413a2f753630b1d7+ std_msgs.Header._get_hash_recursive(newparents)+ Polygon._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if PolygonStamped._packed_fingerprint is None:
            PolygonStamped._packed_fingerprint = struct.pack(">Q", PolygonStamped._get_hash_recursive([]))
        return PolygonStamped._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", PolygonStamped._get_packed_fingerprint())[0]

