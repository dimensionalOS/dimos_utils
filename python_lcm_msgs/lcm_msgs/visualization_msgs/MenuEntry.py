"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

class MenuEntry(object):

    __slots__ = ["id", "parent_id", "title", "command", "command_type"]

    __typenames__ = ["int32_t", "int32_t", "string", "string", "byte"]

    __dimensions__ = [None, None, None, None, None]

    FEEDBACK = 0
    ROSRUN = 1
    ROSLAUNCH = 2

    def __init__(self):
        self.id = 0
        """ LCM Type: int32_t """
        self.parent_id = 0
        """ LCM Type: int32_t """
        self.title = ""
        """ LCM Type: string """
        self.command = ""
        """ LCM Type: string """
        self.command_type = 0
        """ LCM Type: byte """

    def encode(self):
        buf = BytesIO()
        buf.write(MenuEntry._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">ii", self.id, self.parent_id))
        __title_encoded = self.title.encode('utf-8')
        buf.write(struct.pack('>I', len(__title_encoded)+1))
        buf.write(__title_encoded)
        buf.write(b"\0")
        __command_encoded = self.command.encode('utf-8')
        buf.write(struct.pack('>I', len(__command_encoded)+1))
        buf.write(__command_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">B", self.command_type))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != MenuEntry._get_packed_fingerprint():
            raise ValueError("Decode error")
        return MenuEntry._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = MenuEntry()
        self.id, self.parent_id = struct.unpack(">ii", buf.read(8))
        __title_len = struct.unpack('>I', buf.read(4))[0]
        self.title = buf.read(__title_len)[:-1].decode('utf-8', 'replace')
        __command_len = struct.unpack('>I', buf.read(4))[0]
        self.command = buf.read(__command_len)[:-1].decode('utf-8', 'replace')
        self.command_type = struct.unpack(">B", buf.read(1))[0]
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if MenuEntry in parents: return 0
        tmphash = (0x667b2d15ef03e972) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if MenuEntry._packed_fingerprint is None:
            MenuEntry._packed_fingerprint = struct.pack(">Q", MenuEntry._get_hash_recursive([]))
        return MenuEntry._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", MenuEntry._get_packed_fingerprint())[0]

