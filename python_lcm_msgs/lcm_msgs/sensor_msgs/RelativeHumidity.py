"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

from lcm_msgs import std_msgs
class RelativeHumidity(object):

    __slots__ = ["header", "relative_humidity", "variance"]

    __typenames__ = ["std_msgs.Header", "double", "double"]

    __dimensions__ = [None, None, None]

    def __init__(self):
        self.header = std_msgs.Header()
        """ LCM Type: std_msgs.Header """
        self.relative_humidity = 0.0
        """ LCM Type: double """
        self.variance = 0.0
        """ LCM Type: double """

    def encode(self):
        buf = BytesIO()
        buf.write(RelativeHumidity._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.header._get_packed_fingerprint() == std_msgs.Header._get_packed_fingerprint()
        self.header._encode_one(buf)
        buf.write(struct.pack(">dd", self.relative_humidity, self.variance))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != RelativeHumidity._get_packed_fingerprint():
            raise ValueError("Decode error")
        return RelativeHumidity._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = RelativeHumidity()
        self.header = std_msgs.Header._decode_one(buf)
        self.relative_humidity, self.variance = struct.unpack(">dd", buf.read(16))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if RelativeHumidity in parents: return 0
        newparents = parents + [RelativeHumidity]
        tmphash = (0x1db00395f858f771+ std_msgs.Header._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if RelativeHumidity._packed_fingerprint is None:
            RelativeHumidity._packed_fingerprint = struct.pack(">Q", RelativeHumidity._get_hash_recursive([]))
        return RelativeHumidity._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", RelativeHumidity._get_packed_fingerprint())[0]

