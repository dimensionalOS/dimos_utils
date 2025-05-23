"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

from lcm_msgs import geometry_msgs
from lcm_msgs import std_msgs
class InteractiveMarkerPose(object):

    __slots__ = ["header", "pose", "name"]

    __typenames__ = ["std_msgs.Header", "geometry_msgs.Pose", "string"]

    __dimensions__ = [None, None, None]

    def __init__(self):
        self.header = std_msgs.Header()
        """ LCM Type: std_msgs.Header """
        self.pose = geometry_msgs.Pose()
        """ LCM Type: geometry_msgs.Pose """
        self.name = ""
        """ LCM Type: string """

    def encode(self):
        buf = BytesIO()
        buf.write(InteractiveMarkerPose._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.header._get_packed_fingerprint() == std_msgs.Header._get_packed_fingerprint()
        self.header._encode_one(buf)
        assert self.pose._get_packed_fingerprint() == geometry_msgs.Pose._get_packed_fingerprint()
        self.pose._encode_one(buf)
        __name_encoded = self.name.encode('utf-8')
        buf.write(struct.pack('>I', len(__name_encoded)+1))
        buf.write(__name_encoded)
        buf.write(b"\0")

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != InteractiveMarkerPose._get_packed_fingerprint():
            raise ValueError("Decode error")
        return InteractiveMarkerPose._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = InteractiveMarkerPose()
        self.header = std_msgs.Header._decode_one(buf)
        self.pose = geometry_msgs.Pose._decode_one(buf)
        __name_len = struct.unpack('>I', buf.read(4))[0]
        self.name = buf.read(__name_len)[:-1].decode('utf-8', 'replace')
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if InteractiveMarkerPose in parents: return 0
        newparents = parents + [InteractiveMarkerPose]
        tmphash = (0x8c873ae70410464d+ std_msgs.Header._get_hash_recursive(newparents)+ geometry_msgs.Pose._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if InteractiveMarkerPose._packed_fingerprint is None:
            InteractiveMarkerPose._packed_fingerprint = struct.pack(">Q", InteractiveMarkerPose._get_hash_recursive([]))
        return InteractiveMarkerPose._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", InteractiveMarkerPose._get_packed_fingerprint())[0]

