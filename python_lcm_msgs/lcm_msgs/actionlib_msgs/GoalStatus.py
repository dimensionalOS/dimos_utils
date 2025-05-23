"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

from . import *
from .GoalID import GoalID
class GoalStatus(object):

    __slots__ = ["goal_id", "status", "text"]

    __typenames__ = ["GoalID", "byte", "string"]

    __dimensions__ = [None, None, None]

    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9

    def __init__(self):
        self.goal_id = GoalID()
        """ LCM Type: GoalID """
        self.status = 0
        """ LCM Type: byte """
        self.text = ""
        """ LCM Type: string """

    def encode(self):
        buf = BytesIO()
        buf.write(GoalStatus._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.goal_id._get_packed_fingerprint() == GoalID._get_packed_fingerprint()
        self.goal_id._encode_one(buf)
        buf.write(struct.pack(">B", self.status))
        __text_encoded = self.text.encode('utf-8')
        buf.write(struct.pack('>I', len(__text_encoded)+1))
        buf.write(__text_encoded)
        buf.write(b"\0")

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != GoalStatus._get_packed_fingerprint():
            raise ValueError("Decode error")
        return GoalStatus._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = GoalStatus()
        self.goal_id = GoalID._decode_one(buf)
        self.status = struct.unpack(">B", buf.read(1))[0]
        __text_len = struct.unpack('>I', buf.read(4))[0]
        self.text = buf.read(__text_len)[:-1].decode('utf-8', 'replace')
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if GoalStatus in parents: return 0
        newparents = parents + [GoalStatus]
        tmphash = (0xc0b4e95febdcd994+ GoalID._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if GoalStatus._packed_fingerprint is None:
            GoalStatus._packed_fingerprint = struct.pack(">Q", GoalStatus._get_hash_recursive([]))
        return GoalStatus._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", GoalStatus._get_packed_fingerprint())[0]

