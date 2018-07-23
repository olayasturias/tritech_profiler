# -*- coding: utf-8 -*-

"""It is used to prepare Tritech Profiler commands."""

import bitstring

__author__ = "Olaya Alvarez, Anass Al-Wohoush, Erin Havens"


class Command(object):

    """Sonar command."""

    def __init__(self, id, payload=None):
        """Constructs Command object.

        Args:
            id: Message ID.
            payload: Message payload (optional).
        """
        self.id = id
        self.payload = payload if payload else bitstring.BitStream()
        self.size = (self.payload.length / 8) + 8


    def serialize(self):
        """Constructs corresponding string of bytes to send to sonar.

        **Atributes**:

        .. data:: id

            ID of the message (mtAlive, mtHeadData,...)

        .. data:: hex_size

            Size in bytes of the packet

        .. data:: payload

            Data to be added to packet, like configuration parameters

        Returns:
            String representation of data.
        """
        hex_size = bytearray("{:04X}".format(self.size))
        values = {
            "id": self.id,
            "hex": hex_size,
            "bin": self.size,
            "bytes_left": self.size - 5,
            "payload_length": self.payload.length,
            "payload": self.payload,

        }

        serial_format = (
            "0x40, bits:32=hex, uintle:16=bin, 0xFF, 0x14, uint:8=bytes_left,"
            "uint:8=id, 0x80, 0x14, bits:payload_length=payload,"
            "0x0A"
        )
        message = bitstring.pack(serial_format, **values)
        return message.tobytes()
