import struct


class Helper:
    @staticmethod
    def bit32ToFloat(byte_array):
        byts = bytearray()
        byts.extend(byte_array)
        return float(struct.unpack('<f', byts)[0])
