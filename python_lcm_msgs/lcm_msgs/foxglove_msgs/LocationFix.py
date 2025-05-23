"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

from lcm_msgs import builtin_interfaces
class LocationFix(object):

    __slots__ = ["timestamp", "frame_id", "latitude", "longitude", "altitude", "position_covariance", "position_covariance_type"]

    __typenames__ = ["builtin_interfaces.Time", "string", "double", "double", "double", "double", "byte"]

    __dimensions__ = [None, None, None, None, None, [9], None]

    UNKNOWN = 0
    APPROXIMATED = 1
    DIAGONAL_KNOWN = 2
    KNOWN = 3

    def __init__(self):
        self.timestamp = builtin_interfaces.Time()
        """ LCM Type: builtin_interfaces.Time """
        self.frame_id = ""
        """ LCM Type: string """
        self.latitude = 0.0
        """ LCM Type: double """
        self.longitude = 0.0
        """ LCM Type: double """
        self.altitude = 0.0
        """ LCM Type: double """
        self.position_covariance = [ 0.0 for dim0 in range(9) ]
        """ LCM Type: double[9] """
        self.position_covariance_type = 0
        """ LCM Type: byte """

    def encode(self):
        buf = BytesIO()
        buf.write(LocationFix._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.timestamp._get_packed_fingerprint() == builtin_interfaces.Time._get_packed_fingerprint()
        self.timestamp._encode_one(buf)
        __frame_id_encoded = self.frame_id.encode('utf-8')
        buf.write(struct.pack('>I', len(__frame_id_encoded)+1))
        buf.write(__frame_id_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">ddd", self.latitude, self.longitude, self.altitude))
        buf.write(struct.pack('>9d', *self.position_covariance[:9]))
        buf.write(struct.pack(">B", self.position_covariance_type))

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != LocationFix._get_packed_fingerprint():
            raise ValueError("Decode error")
        return LocationFix._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = LocationFix()
        self.timestamp = builtin_interfaces.Time._decode_one(buf)
        __frame_id_len = struct.unpack('>I', buf.read(4))[0]
        self.frame_id = buf.read(__frame_id_len)[:-1].decode('utf-8', 'replace')
        self.latitude, self.longitude, self.altitude = struct.unpack(">ddd", buf.read(24))
        self.position_covariance = struct.unpack('>9d', buf.read(72))
        self.position_covariance_type = struct.unpack(">B", buf.read(1))[0]
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if LocationFix in parents: return 0
        newparents = parents + [LocationFix]
        tmphash = (0xc7c172f7d2332f54+ builtin_interfaces.Time._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if LocationFix._packed_fingerprint is None:
            LocationFix._packed_fingerprint = struct.pack(">Q", LocationFix._get_hash_recursive([]))
        return LocationFix._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", LocationFix._get_packed_fingerprint())[0]

