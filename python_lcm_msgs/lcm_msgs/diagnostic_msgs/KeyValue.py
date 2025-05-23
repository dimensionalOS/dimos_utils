"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""


from io import BytesIO
import struct

class KeyValue(object):

    __slots__ = ["key", "value"]

    __typenames__ = ["string", "string"]

    __dimensions__ = [None, None]

    def __init__(self):
        self.key = ""
        """ LCM Type: string """
        self.value = ""
        """ LCM Type: string """

    def encode(self):
        buf = BytesIO()
        buf.write(KeyValue._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        __key_encoded = self.key.encode('utf-8')
        buf.write(struct.pack('>I', len(__key_encoded)+1))
        buf.write(__key_encoded)
        buf.write(b"\0")
        __value_encoded = self.value.encode('utf-8')
        buf.write(struct.pack('>I', len(__value_encoded)+1))
        buf.write(__value_encoded)
        buf.write(b"\0")

    @staticmethod
    def decode(data: bytes):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != KeyValue._get_packed_fingerprint():
            raise ValueError("Decode error")
        return KeyValue._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = KeyValue()
        __key_len = struct.unpack('>I', buf.read(4))[0]
        self.key = buf.read(__key_len)[:-1].decode('utf-8', 'replace')
        __value_len = struct.unpack('>I', buf.read(4))[0]
        self.value = buf.read(__value_len)[:-1].decode('utf-8', 'replace')
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if KeyValue in parents: return 0
        tmphash = (0x97574015d52eedde) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if KeyValue._packed_fingerprint is None:
            KeyValue._packed_fingerprint = struct.pack(">Q", KeyValue._get_hash_recursive([]))
        return KeyValue._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", KeyValue._get_packed_fingerprint())[0]

