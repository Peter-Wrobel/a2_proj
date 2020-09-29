"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class simple_motor_command(object):
    __slots__ = ["timestamp", "forward_velocity", "angular_velocity"]

    __typenames__ = ["int64_t", "float", "float"]

    __dimensions__ = [None, None, None]

    def __init__(self):
        self.timestamp = 0
        self.forward_velocity = 0.0
        self.angular_velocity = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(simple_motor_command._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qff", self.timestamp, self.forward_velocity, self.angular_velocity))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != simple_motor_command._get_packed_fingerprint():
            raise ValueError("Decode error")
        return simple_motor_command._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = simple_motor_command()
        self.timestamp, self.forward_velocity, self.angular_velocity = struct.unpack(">qff", buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if simple_motor_command in parents: return 0
        tmphash = (0x395b957e90f950ff) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if simple_motor_command._packed_fingerprint is None:
            simple_motor_command._packed_fingerprint = struct.pack(">Q", simple_motor_command._get_hash_recursive([]))
        return simple_motor_command._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

