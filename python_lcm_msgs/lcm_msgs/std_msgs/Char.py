"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

class Char(object):

    __slots__ = ["data"]

    __typenames__ = ["byte"]

    __dimensions__ = [None]

    def __init__(self):
        self.data = 0
        """ LCM Type: byte """

    def encode(self):
        buf = BytesIO()
        buf.write(Char._get_packed_fingerprint())
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
        if buf.read(8) != Char._get_packed_fingerprint():
            raise ValueError("Decode error")
        return Char._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = Char()
        self.data = struct.unpack(">B", buf.read(1))[0]
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if Char in parents: return 0
        tmphash = (0x74856d0f697d2dc2) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if Char._packed_fingerprint is None:
            Char._packed_fingerprint = struct.pack(">Q", Char._get_hash_recursive([]))
        return Char._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", Char._get_packed_fingerprint())[0]

