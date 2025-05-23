"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

class Bool(object):

    __slots__ = ["data"]

    __typenames__ = ["boolean"]

    __dimensions__ = [None]

    def __init__(self):
        self.data = False
        """ LCM Type: boolean """

    def encode(self):
        buf = BytesIO()
        buf.write(Bool._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">b", self.data))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != Bool._get_packed_fingerprint():
            raise ValueError("Decode error")
        return Bool._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = Bool()
        self.data = bool(struct.unpack('b', buf.read(1))[0])
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if Bool in parents: return 0
        tmphash = (0xf5f7835284a871f) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if Bool._packed_fingerprint is None:
            Bool._packed_fingerprint = struct.pack(">Q", Bool._get_hash_recursive([]))
        return Bool._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", Bool._get_packed_fingerprint())[0]

