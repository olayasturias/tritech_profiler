# -*- coding: utf-8 -*-

"""Tritech Profiler serial communication handler."""

import errno
import rospy
import serial
import select
import bitstring
from replies import Reply
from messages import Message
from commands import Command
from exceptions import PacketIncomplete

__author__ = "Anass Al-Wohoush, Jana Pavlasek, Malcolm Watt"


class Socket(object):

    """
    *Socket* class for Serial communication socket.
    This class constains the necessary methods for configuring the serial
    communication between the sensor and the node itself.

    **Attributes**:

    .. data:: conn

        Serial connection attribute.

    """

    def __init__(self, port,baudrate):
        """Constructs Socket object.

        Args:
            port: Serial port.
        """

        self.conn = serial.Serial(port=port,baudrate=baudrate,timeout=40)
        self.conn.port = port
        self.conn.baudrate = baudrate
        self.port_enabled = True

    def open(self):
        """Opens serial connection."""
        if not self.conn.is_open:
            self.conn.open()

    def close(self):
        """Closes serial connection."""
        self.conn.close()

    def send(self, message, payload=None):
        """Formats message and payload into packet and sends it to device.

        Args:
            command: Command to send.
            payload: Additional payload to send in packet.
        """
        cmd = Command(message, payload)
        rospy.logdebug("Sending %s: %s", Message.to_string(message), payload)
        self.conn.write(cmd.serialize())

    def get_reply(self):
        """Waits for and returns Reply.

        Returns:
            First complete reply if expected message ID was not specified,
            otherwise first complete reply of expected message ID.

        Raises:
            PacketCorrupted: Packet is corrupt.
        """
        try:
            #self.open() ################################ IN CASE GUI CLOSED PORT
            # Wait for the '@' character.
            while not self.conn.read() == "@" and self.port_enabled:
                pass

            # Read one line at a time until packet is complete and parsed.
            packet = bitstring.BitStream("0x40")
            while self.port_enabled:
                # Read until new line.
                current_line = self.conn.readline()
                for char in current_line:
                    packet.append("0x{:02X}".format(ord(char)))

                # Try to parse.
                try:
                    reply = Reply(packet)
                    break
                except PacketIncomplete:
                    # Keep looking.
                    continue
            if self.port_enabled:
                rospy.logdebug("Received %s: %s", reply.name, reply.payload)
                return reply
            else:
                return Reply(bitstring.BitStream("0x000A"))
        except select.error as (code, msg):
            # Set SIGINT as KeyboardInterrupt correctly, because pyserial has
            # problems.
            # if code == errno.EINTR:
            #     raise KeyboardInterrupt()
            #
            # # Otherwise, reraise.
            # raise
            rospy.logerr('could not read stream. Have you closed the port?')
            return
