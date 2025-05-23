"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

from . import *
from .TextAnnotation import TextAnnotation
from .PointsAnnotation import PointsAnnotation
from .CircleAnnotation import CircleAnnotation
class ImageAnnotations(object):

    __slots__ = ["circles_length", "points_length", "texts_length", "circles", "points", "texts"]

    __typenames__ = ["int32_t", "int32_t", "int32_t", "CircleAnnotation", "PointsAnnotation", "TextAnnotation"]

    __dimensions__ = [None, None, None, ["circles_length"], ["points_length"], ["texts_length"]]

    def __init__(self):
        self.circles_length = 0
        """ LCM Type: int32_t """
        self.points_length = 0
        """ LCM Type: int32_t """
        self.texts_length = 0
        """ LCM Type: int32_t """
        self.circles = []
        """ LCM Type: CircleAnnotation[circles_length] """
        self.points = []
        """ LCM Type: PointsAnnotation[points_length] """
        self.texts = []
        """ LCM Type: TextAnnotation[texts_length] """

    def encode(self):
        buf = BytesIO()
        buf.write(ImageAnnotations._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">iii", self.circles_length, self.points_length, self.texts_length))
        for i0 in range(self.circles_length):
            assert self.circles[i0]._get_packed_fingerprint() == CircleAnnotation._get_packed_fingerprint()
            self.circles[i0]._encode_one(buf)
        for i0 in range(self.points_length):
            assert self.points[i0]._get_packed_fingerprint() == PointsAnnotation._get_packed_fingerprint()
            self.points[i0]._encode_one(buf)
        for i0 in range(self.texts_length):
            assert self.texts[i0]._get_packed_fingerprint() == TextAnnotation._get_packed_fingerprint()
            self.texts[i0]._encode_one(buf)

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != ImageAnnotations._get_packed_fingerprint():
            raise ValueError("Decode error")
        return ImageAnnotations._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = ImageAnnotations()
        self.circles_length, self.points_length, self.texts_length = struct.unpack(">iii", buf.read(12))
        self.circles = []
        for i0 in range(self.circles_length):
            self.circles.append(CircleAnnotation._decode_one(buf))
        self.points = []
        for i0 in range(self.points_length):
            self.points.append(PointsAnnotation._decode_one(buf))
        self.texts = []
        for i0 in range(self.texts_length):
            self.texts.append(TextAnnotation._decode_one(buf))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if ImageAnnotations in parents: return 0
        newparents = parents + [ImageAnnotations]
        tmphash = (0x8b3a52c632c59b07+ CircleAnnotation._get_hash_recursive(newparents)+ PointsAnnotation._get_hash_recursive(newparents)+ TextAnnotation._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if ImageAnnotations._packed_fingerprint is None:
            ImageAnnotations._packed_fingerprint = struct.pack(">Q", ImageAnnotations._get_hash_recursive([]))
        return ImageAnnotations._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", ImageAnnotations._get_packed_fingerprint())[0]

