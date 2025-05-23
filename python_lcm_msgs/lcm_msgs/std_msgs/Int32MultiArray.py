"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

from . import *
from .MultiArrayLayout import MultiArrayLayout
class Int32MultiArray(object):

    __slots__ = ["data_length", "layout", "data"]

    __typenames__ = ["int32_t", "MultiArrayLayout", "int32_t"]

    __dimensions__ = [None, None, ["data_length"]]

    def __init__(self):
        self.data_length = 0
        """ LCM Type: int32_t """
        self.layout = MultiArrayLayout()
        """ LCM Type: MultiArrayLayout """
        self.data = []
        """ LCM Type: int32_t[data_length] """

    def encode(self):
        buf = BytesIO()
        buf.write(Int32MultiArray._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">i", self.data_length))
        assert self.layout._get_packed_fingerprint() == MultiArrayLayout._get_packed_fingerprint()
        self.layout._encode_one(buf)
        buf.write(struct.pack('>%di' % self.data_length, *self.data[:self.data_length]))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != Int32MultiArray._get_packed_fingerprint():
            raise ValueError("Decode error")
        return Int32MultiArray._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = Int32MultiArray()
        self.data_length = struct.unpack(">i", buf.read(4))[0]
        self.layout = MultiArrayLayout._decode_one(buf)
        self.data = struct.unpack('>%di' % self.data_length, buf.read(self.data_length * 4))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if Int32MultiArray in parents: return 0
        newparents = parents + [Int32MultiArray]
        tmphash = (0xaa51366c8f222486+ MultiArrayLayout._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if Int32MultiArray._packed_fingerprint is None:
            Int32MultiArray._packed_fingerprint = struct.pack(">Q", Int32MultiArray._get_hash_recursive([]))
        return Int32MultiArray._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", Int32MultiArray._get_packed_fingerprint())[0]

