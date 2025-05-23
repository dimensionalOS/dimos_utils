"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

class JoyFeedback(object):

    __slots__ = ["type", "id", "intensity"]

    __typenames__ = ["byte", "byte", "float"]

    __dimensions__ = [None, None, None]

    TYPE_LED = 0
    TYPE_RUMBLE = 1
    TYPE_BUZZER = 2

    def __init__(self):
        self.type = 0
        """ LCM Type: byte """
        self.id = 0
        """ LCM Type: byte """
        self.intensity = 0.0
        """ LCM Type: float """

    def encode(self):
        buf = BytesIO()
        buf.write(JoyFeedback._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">BBf", self.type, self.id, self.intensity))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != JoyFeedback._get_packed_fingerprint():
            raise ValueError("Decode error")
        return JoyFeedback._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = JoyFeedback()
        self.type, self.id, self.intensity = struct.unpack(">BBf", buf.read(6))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if JoyFeedback in parents: return 0
        tmphash = (0x244b242f84e14215) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if JoyFeedback._packed_fingerprint is None:
            JoyFeedback._packed_fingerprint = struct.pack(">Q", JoyFeedback._get_hash_recursive([]))
        return JoyFeedback._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", JoyFeedback._get_packed_fingerprint())[0]

