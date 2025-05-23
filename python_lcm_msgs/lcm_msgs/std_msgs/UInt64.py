"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

class UInt64(object):

    __slots__ = ["data"]

    __typenames__ = ["int64_t"]

    __dimensions__ = [None]

    def __init__(self):
        self.data = 0
        """ LCM Type: int64_t """

    def encode(self):
        buf = BytesIO()
        buf.write(UInt64._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.data))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != UInt64._get_packed_fingerprint():
            raise ValueError("Decode error")
        return UInt64._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = UInt64()
        self.data = struct.unpack(">q", buf.read(8))[0]
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if UInt64 in parents: return 0
        tmphash = (0x165e7cfef748811f) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if UInt64._packed_fingerprint is None:
            UInt64._packed_fingerprint = struct.pack(">Q", UInt64._get_hash_recursive([]))
        return UInt64._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", UInt64._get_packed_fingerprint())[0]

