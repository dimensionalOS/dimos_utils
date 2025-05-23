"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

class Quaternion(object):

    __slots__ = ["x", "y", "z", "w"]

    __typenames__ = ["double", "double", "double", "double"]

    __dimensions__ = [None, None, None, None]

    def __init__(self):
        self.x = 0.0
        """ LCM Type: double """
        self.y = 0.0
        """ LCM Type: double """
        self.z = 0.0
        """ LCM Type: double """
        self.w = 0.0
        """ LCM Type: double """

    def encode(self):
        buf = BytesIO()
        buf.write(Quaternion._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">dddd", self.x, self.y, self.z, self.w))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != Quaternion._get_packed_fingerprint():
            raise ValueError("Decode error")
        return Quaternion._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = Quaternion()
        self.x, self.y, self.z, self.w = struct.unpack(">dddd", buf.read(32))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if Quaternion in parents: return 0
        tmphash = (0x9b1dee9dfc8c0515) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if Quaternion._packed_fingerprint is None:
            Quaternion._packed_fingerprint = struct.pack(">Q", Quaternion._get_hash_recursive([]))
        return Quaternion._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", Quaternion._get_packed_fingerprint())[0]

