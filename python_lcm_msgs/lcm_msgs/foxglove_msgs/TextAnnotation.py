"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

from . import *
from lcm_msgs import builtin_interfaces
from .Color import Color
from .Point2 import Point2
class TextAnnotation(object):

    __slots__ = ["timestamp", "position", "text", "font_size", "text_color", "background_color"]

    __typenames__ = ["builtin_interfaces.Time", "Point2", "string", "double", "Color", "Color"]

    __dimensions__ = [None, None, None, None, None, None]

    def __init__(self):
        self.timestamp = builtin_interfaces.Time()
        """ LCM Type: builtin_interfaces.Time """
        self.position = Point2()
        """ LCM Type: Point2 """
        self.text = ""
        """ LCM Type: string """
        self.font_size = 0.0
        """ LCM Type: double """
        self.text_color = Color()
        """ LCM Type: Color """
        self.background_color = Color()
        """ LCM Type: Color """

    def encode(self):
        buf = BytesIO()
        buf.write(TextAnnotation._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.timestamp._get_packed_fingerprint() == builtin_interfaces.Time._get_packed_fingerprint()
        self.timestamp._encode_one(buf)
        assert self.position._get_packed_fingerprint() == Point2._get_packed_fingerprint()
        self.position._encode_one(buf)
        __text_encoded = self.text.encode('utf-8')
        buf.write(struct.pack('>I', len(__text_encoded)+1))
        buf.write(__text_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">d", self.font_size))
        assert self.text_color._get_packed_fingerprint() == Color._get_packed_fingerprint()
        self.text_color._encode_one(buf)
        assert self.background_color._get_packed_fingerprint() == Color._get_packed_fingerprint()
        self.background_color._encode_one(buf)

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != TextAnnotation._get_packed_fingerprint():
            raise ValueError("Decode error")
        return TextAnnotation._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = TextAnnotation()
        self.timestamp = builtin_interfaces.Time._decode_one(buf)
        self.position = Point2._decode_one(buf)
        __text_len = struct.unpack('>I', buf.read(4))[0]
        self.text = buf.read(__text_len)[:-1].decode('utf-8', 'replace')
        self.font_size = struct.unpack(">d", buf.read(8))[0]
        self.text_color = Color._decode_one(buf)
        self.background_color = Color._decode_one(buf)
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if TextAnnotation in parents: return 0
        newparents = parents + [TextAnnotation]
        tmphash = (0x1354af1f564701e9+ builtin_interfaces.Time._get_hash_recursive(newparents)+ Point2._get_hash_recursive(newparents)+ Color._get_hash_recursive(newparents)+ Color._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if TextAnnotation._packed_fingerprint is None:
            TextAnnotation._packed_fingerprint = struct.pack(">Q", TextAnnotation._get_hash_recursive([]))
        return TextAnnotation._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", TextAnnotation._get_packed_fingerprint())[0]

