"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

from lcm_msgs import geometry_msgs
from lcm_msgs import std_msgs
class ImageMarker(object):

    __slots__ = ["points_length", "outline_colors_length", "header", "ns", "id", "type", "action", "position", "scale", "outline_color", "filled", "fill_color", "lifetime", "points", "outline_colors"]

    __typenames__ = ["int32_t", "int32_t", "std_msgs.Header", "string", "int32_t", "int32_t", "int32_t", "geometry_msgs.Point", "float", "std_msgs.ColorRGBA", "byte", "std_msgs.ColorRGBA", "std_msgs.Duration", "geometry_msgs.Point", "std_msgs.ColorRGBA"]

    __dimensions__ = [None, None, None, None, None, None, None, None, None, None, None, None, None, ["points_length"], ["outline_colors_length"]]

    CIRCLE = 0
    LINE_STRIP = 1
    LINE_LIST = 2
    POLYGON = 3
    POINTS = 4
    ADD = 0
    REMOVE = 1

    def __init__(self):
        self.points_length = 0
        """ LCM Type: int32_t """
        self.outline_colors_length = 0
        """ LCM Type: int32_t """
        self.header = std_msgs.Header()
        """ LCM Type: std_msgs.Header """
        self.ns = ""
        """ LCM Type: string """
        self.id = 0
        """ LCM Type: int32_t """
        self.type = 0
        """ LCM Type: int32_t """
        self.action = 0
        """ LCM Type: int32_t """
        self.position = geometry_msgs.Point()
        """ LCM Type: geometry_msgs.Point """
        self.scale = 0.0
        """ LCM Type: float """
        self.outline_color = std_msgs.ColorRGBA()
        """ LCM Type: std_msgs.ColorRGBA """
        self.filled = 0
        """ LCM Type: byte """
        self.fill_color = std_msgs.ColorRGBA()
        """ LCM Type: std_msgs.ColorRGBA """
        self.lifetime = std_msgs.Duration()
        """ LCM Type: std_msgs.Duration """
        self.points = []
        """ LCM Type: geometry_msgs.Point[points_length] """
        self.outline_colors = []
        """ LCM Type: std_msgs.ColorRGBA[outline_colors_length] """

    def encode(self):
        buf = BytesIO()
        buf.write(ImageMarker._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">ii", self.points_length, self.outline_colors_length))
        assert self.header._get_packed_fingerprint() == std_msgs.Header._get_packed_fingerprint()
        self.header._encode_one(buf)
        __ns_encoded = self.ns.encode('utf-8')
        buf.write(struct.pack('>I', len(__ns_encoded)+1))
        buf.write(__ns_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">iii", self.id, self.type, self.action))
        assert self.position._get_packed_fingerprint() == geometry_msgs.Point._get_packed_fingerprint()
        self.position._encode_one(buf)
        buf.write(struct.pack(">f", self.scale))
        assert self.outline_color._get_packed_fingerprint() == std_msgs.ColorRGBA._get_packed_fingerprint()
        self.outline_color._encode_one(buf)
        buf.write(struct.pack(">B", self.filled))
        assert self.fill_color._get_packed_fingerprint() == std_msgs.ColorRGBA._get_packed_fingerprint()
        self.fill_color._encode_one(buf)
        assert self.lifetime._get_packed_fingerprint() == std_msgs.Duration._get_packed_fingerprint()
        self.lifetime._encode_one(buf)
        for i0 in range(self.points_length):
            assert self.points[i0]._get_packed_fingerprint() == geometry_msgs.Point._get_packed_fingerprint()
            self.points[i0]._encode_one(buf)
        for i0 in range(self.outline_colors_length):
            assert self.outline_colors[i0]._get_packed_fingerprint() == std_msgs.ColorRGBA._get_packed_fingerprint()
            self.outline_colors[i0]._encode_one(buf)

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != ImageMarker._get_packed_fingerprint():
            raise ValueError("Decode error")
        return ImageMarker._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = ImageMarker()
        self.points_length, self.outline_colors_length = struct.unpack(">ii", buf.read(8))
        self.header = std_msgs.Header._decode_one(buf)
        __ns_len = struct.unpack('>I', buf.read(4))[0]
        self.ns = buf.read(__ns_len)[:-1].decode('utf-8', 'replace')
        self.id, self.type, self.action = struct.unpack(">iii", buf.read(12))
        self.position = geometry_msgs.Point._decode_one(buf)
        self.scale = struct.unpack(">f", buf.read(4))[0]
        self.outline_color = std_msgs.ColorRGBA._decode_one(buf)
        self.filled = struct.unpack(">B", buf.read(1))[0]
        self.fill_color = std_msgs.ColorRGBA._decode_one(buf)
        self.lifetime = std_msgs.Duration._decode_one(buf)
        self.points = []
        for i0 in range(self.points_length):
            self.points.append(geometry_msgs.Point._decode_one(buf))
        self.outline_colors = []
        for i0 in range(self.outline_colors_length):
            self.outline_colors.append(std_msgs.ColorRGBA._decode_one(buf))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if ImageMarker in parents: return 0
        newparents = parents + [ImageMarker]
        tmphash = (0x3a3ea371b474d924+ std_msgs.Header._get_hash_recursive(newparents)+ geometry_msgs.Point._get_hash_recursive(newparents)+ std_msgs.ColorRGBA._get_hash_recursive(newparents)+ std_msgs.ColorRGBA._get_hash_recursive(newparents)+ std_msgs.Duration._get_hash_recursive(newparents)+ geometry_msgs.Point._get_hash_recursive(newparents)+ std_msgs.ColorRGBA._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if ImageMarker._packed_fingerprint is None:
            ImageMarker._packed_fingerprint = struct.pack(">Q", ImageMarker._get_hash_recursive([]))
        return ImageMarker._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", ImageMarker._get_packed_fingerprint())[0]

