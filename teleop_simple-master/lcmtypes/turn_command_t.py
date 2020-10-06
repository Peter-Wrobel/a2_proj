"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class turn_command_t(object):
    __slots__ = ["p_term", "d_term"]

    __typenames__ = ["float", "float"]

    __dimensions__ = [None, None]

    def __init__(self):
        self.p_term = 0.0
        self.d_term = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(turn_command_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">ff", self.p_term, self.d_term))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != turn_command_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return turn_command_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = turn_command_t()
        self.p_term, self.d_term = struct.unpack(">ff", buf.read(8))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if turn_command_t in parents: return 0
        tmphash = (0x5e8d3cf67c7e07f1) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if turn_command_t._packed_fingerprint is None:
            turn_command_t._packed_fingerprint = struct.pack(">Q", turn_command_t._get_hash_recursive([]))
        return turn_command_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
