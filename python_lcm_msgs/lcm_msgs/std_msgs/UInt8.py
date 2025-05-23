"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

class UInt8(object):

    __slots__ = ["data"]

    __typenames__ = ["byte"]

    __dimensions__ = [None]

    def __init__(self):
        self.data = 0
        """ LCM Type: byte """

    def encode(self):
        buf = BytesIO()
        buf.write(UInt8._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">B", self.data))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != UInt8._get_packed_fingerprint():
            raise ValueError("Decode error")
        return UInt8._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = UInt8()
        self.data = struct.unpack(">B", buf.read(1))[0]
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if UInt8 in parents: return 0
        tmphash = (0x74856d0f697d2dc2) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if UInt8._packed_fingerprint is None:
            UInt8._packed_fingerprint = struct.pack(">Q", UInt8._get_hash_recursive([]))
        return UInt8._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", UInt8._get_packed_fingerprint())[0]

