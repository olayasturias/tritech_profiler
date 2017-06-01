# -*- coding: utf-8 -*-

"""Tritech Profiler sonar."""

import rospy
import serial
import datetime
import bitstring
import exceptions
from math import pow
from socket import Socket
from messages import Message
from tools import ScanSlice, to_radians, to_sonar_angles

__author__ = "Olaya Alvarez, Anass Al-Wohoush"

"""
.. codeauthor:: Olaya Alvarez Tunon
: file sonar.py
"""


class TritechProfiler(object):

    """ *TritechProfiler* class for Tritech Profiler sonar.

    This class provides all the necesary methods to operate the sonar profiler,
    including the sending of commands to configure, scan, ping, or reboot the sensor.

    In order for attribute changes to immediately be reflected on the device,
    use the set() method with the appropriate keyword.

    For example, to setup a 20 m sector scan:

        with TritechProfiler() as sonar:
            sonar.set(prf_alt=False, range=20)

    All angles are in radians and headings are relative to the red LED. So, if
    the sonar were not inverted, 0 radians would be facing forward.

    **Attributes**:
        .. data:: adc_threshold

            Analogue detect threshold level.

        .. data:: filt_gain

            Percentage of Adaptive Gain Control that is applied to receiver.

        .. data:: agc

            True if automatic refining of receiver gain, otherwise 'manual' gain.

        .. data:: centred

            Whether the sonar motor is centred.

        .. data:: clock

            Sonar time of the day.

        .. data:: conn

            Serial connection.

        .. data:: prf_alt

            True if alternating scan, false if scan in one direction.

        .. data:: gain

            Initial gain percentage (0.00-1.00).

        .. data:: has_cfg

            Whether the sonar has acknowledged with mtBBUserData.

        .. data:: heading

            Current sonar heading in radians.

        .. data:: initialized

            Whether the sonar has been initialized with parameters.

        .. data:: inverted

            Whether the sonar is mounted upside down.

        .. data:: left_limit

            Left limit of sector scan in radians.

        .. data:: mo_time

            High speed limit of the motor in units of 10 microseconds.

        .. data:: motoring

            Whether the sonar motor is moving.

        .. data:: motor_on

            Whether device is powered and motor is primed.

        .. data:: nbins

            Number of bins per scan line.

        .. data:: no_params

            Whether the sonar needs parameters before it can scan.

        .. data:: up_time

            Sonar up time.

        .. data:: port

            Serial port.

        .. data:: range

            Scan range in meters.

        .. data:: recentering

            Whether the sonar is recentering its motor.

        .. data:: right_limit

            Right limit of sector scans in radians.

        .. data:: scanning

            Whether the sonar is scanning.

        .. data:: scanright

            Whether the sonar scanning direction is clockwise.

        .. data:: speed

            Speed of sound in medium.

        .. data:: step

            Mechanical resolution (Resolution enumeration).

    """

    def __init__(self, port="/dev/sonar", **kwargs):
        """
        Constructs Sonar object.

        :param port: Serial port (default: /dev/sonar).

        :param kwargs: Key-word arguments to pass to set() on initialization

        :return:

        """
        # Parameter defaults.
        self.adc_threshold = 50.0
        self.filt_gain = 20.00
        self.agc = True
        self.prf_alt = True
        self.gain = 0.50
        self.inverted = False
        self.left_limit = to_radians(0)
        self.mo_time = 250
        self.nbins = 800
        self.range = 10.00
        self.right_limit = to_radians(6399)
        self.scanright = True
        self.speed = 1500.0
        self.step = Resolution.ULTIMATE
        self.lockout = 100

        # Override defaults with key-word arguments or ROS parameters.
        for key, value in self.__dict__.iteritems():
            if key in kwargs:
                self.__setattr__(key, value)
            else:
                param = "{}/{}".format(rospy.get_name(), key)
                if rospy.has_param(param):
                    self.__setattr__(key, rospy.get_param(param))

        # Connection properties.
        self.port = port
        self.conn = None
        self.initialized = False

        # Head info.
        self.centred = False
        self.has_cfg = False
        self.heading = None
        self.motor_on = False
        self.motoring = False
        self.no_params = True
        self.up_time = datetime.timedelta(0)
        self.recentering = False
        self.scanning = False

        # Additional properties.
        self.clock = datetime.timedelta(0)
        self._time_offset = datetime.timedelta(0)
        self.preempted = False

    def __enter__(self):
        """
        Initializes sonar for first use.

        Raises:
            SonarNotFound: Sonar port could not be opened.
        """
        self.open()
        return self

    def __exit__(self, type, value, traceback):
        """
        Cleans up.
        """
        self.close()

    def open(self):
        """
        Initializes sonar connection and sets default properties.

        Raises:
            SonarNotFound: Sonar port could not be opened.
        """
        if not self.conn:
            try:
                self.conn = Socket(self.port)
            except OSError as e:
                raise exceptions.SonarNotFound(self.port, e)

        # Update properties.
        rospy.loginfo("Initializing sonar on %s", self.port)
        self.initialized = True

        # Reboot to make sure the sonar is clean.
        self.send(Message.REBOOT)
        self.update()

        # Set default properties.
        self.set(force=True)

        # Wait for settings to go through.
        while not self.has_cfg or self.no_params:
            rospy.loginfo(
                "Waiting for configuration: (HAS CFG: %s, NO PARAMS: %s)",
                self.has_cfg, self.no_params
            )
            self.update()

        rospy.loginfo("Sonar is ready for use")

    def close(self):
        """
        Closes sonar connection.
        """
        # Reboot first to clear sonar of all parameters.
        self.send(Message.REBOOT)
        self.conn.close()
        self.initialized = False
        rospy.loginfo("Closed sonar socket")

    def get(self, message=None, wait=2):
        """
        Sends command and returns reply.

        :param message: Message to expect (default: first to come in).

        :param kwargs: wait: Seconds to wait until received if a specific message is required (default: 2).

        :return: Reply.

        :raises SonarNotInitialized: Attempt reading serial without opening port.

        :raises TimeoutError: Process timed out while waiting for specific message.

        """
        # Verify sonar is initialized.
        if not self.initialized:
            raise exceptions.SonarNotInitialized()

        expected_name = None
        if message:
            expected_name = Message.to_string(message)
            rospy.logdebug("Waiting for %s message", expected_name)

        # Determine end time.
        end = datetime.datetime.now() + datetime.timedelta(seconds=wait)


        # Wait until received if a specific message ID is requested, otherwise
        # wait forever.
        while message is None or datetime.datetime.now() < end:
            try:
                reply = self.conn.get_reply()

                # Update state if mtAlive.
                if reply.id == Message.ALIVE:
                    self.__update_state(reply)


                # If first was asked, respond immediately.
                if message is None:
                    return reply

                # Otherwise, verify reply ID.
                if reply.id == message:
                    rospy.logdebug("Found %s message", expected_name)
                    return reply
                elif reply.id != Message.ALIVE:
                    rospy.logwarn(
                        "Received unexpected %s message",
                        reply.name
                    )
            except exceptions.PacketCorrupted, serial.SerialException:
                # Keep trying.
                continue

        # Timeout.
        rospy.logerr("Timed out before receiving message: %s", expected_name)
        raise exceptions.TimeoutError()

    def send(self, command, payload=None):
        print 'send'
        """Sends command and returns reply.

        Args:
            command: Command to send.

        Raises:
            SonarNotInitialized: Attempt sending command without opening port.
        """
        if not self.initialized:
            raise exceptions.SonarNotInitialized(command, payload)

        self.conn.send(command, payload)

    def set(self, agc=None, prf_alt=None, scanright=None, step=None,
            filt_gain=None, adc_threshold=None, left_limit=None, right_limit=None,
            mo_time=None, range=None, gain=None, speed=None, lockout = None,
            inverted=None, force=False):
        print 'set'
        """Sends Sonar head command with new properties if needed.

        Only the parameters specified will be modified.

        If initialized, arguments are compared to current properties in order
        to see if sending the command is necessary.

        Args:
            filt_gain: Percentage of Adaptive Gain Control applied to receiver.
            adc_threshold: Analogue detect threshold level.
            agc: True if automatic AGC, otherwise manual AGC.
            prf_alt: True if alternate scan, otherwise continuous scan.
            gain: Initial gain percentage (0.00-1.00).
            inverted: Whether the sonar is mounted upside down.
            left_limit: Left limit of sector scan in radians.
            mo_time: High speed limit of the motor in units of 10 microseconds.
            range: Scan range in meters.
            right_limit: Right limit of sector scans in radians.
            scanright: Whether the sonar scanning direction is clockwise.
            speed: Speed of sound in medium.
            step: Mechanical resolution (Resolution enumeration).
            force: Whether to force setting the parameters or not.

        Raises:
            SonarNotInitialized: Sonar is not initialized.
        """
        if not self.initialized:
            raise exceptions.SonarNotInitialized()

        self.__set_parameters(
            agc=agc, prf_alt=prf_alt, scanright=scanright,
            step=step, filt_gain=filt_gain, adc_threshold=adc_threshold, left_limit=left_limit,
            right_limit=right_limit, mo_time=mo_time, range=range,
            gain=gain, speed=speed, lockout=lockout, inverted=inverted, force=force
        )

    def __set_parameters(self, force, **kwargs):
        print '__set parameters'
        """Sends Sonar head command to set sonar properties.

        Only the parameters specified will be modified.

        If initialized, arguments are compared to current properties in order
        to see if sending the command is necessary.

        Args:
            force: Whether to force set parameters.
            See set().
        """
        rospy.logwarn("Setting parameters...")

        # Set and compare sonar properties.
        necessary = not self.has_cfg or self.no_params or force
        only_reverse = not necessary
        for key, value in kwargs.iteritems():
            if value is not None:
                if hasattr(self, key) and self.__getattribute__(key) != value:
                    self.__setattr__(key, value)
                    necessary = True
                    if key != "scanright":
                        only_reverse = False

        # Return if unnecessary.
        if not necessary:
            rospy.logwarn("Parameters are already set")
            return

        # Return if only switching the motor's direction is necessary.
        if only_reverse:
            rospy.logwarn("Only reversing direction")
            self.scanright = not self.scanright
            return self.reverse()

        # self.left_limit = to_radians(16)
        #
        # self.range = 1.00
        # self.right_limit = to_radians(16)
        # self.scanright = True
        # self.step = Resolution.ULTIMATE

        # Log properties.
        rospy.loginfo("PRF Alternate scan:   %s", self.prf_alt)
        rospy.loginfo("LEFT LIMIT:           %s rad", self.left_limit)
        rospy.loginfo("RIGHT LIMIT:          %s rad", self.right_limit)
        rospy.loginfo("STEP SIZE:            %s rad", self.step)
        rospy.loginfo("RANGE:                %s m", self.range)
        rospy.loginfo("INVERTED:             %s", self.inverted)
        rospy.loginfo("ADC THRESHOLD:        %s ", self.adc_threshold)
        rospy.loginfo("LOCKOUT:              %s%%", self.lockout)
        rospy.loginfo("ADAPTIVE GAIN CONTROL %s", self.agc)
        rospy.loginfo("GAIN:                 %s%%", self.gain * 100)
        rospy.loginfo("MOTOR TIME:           %s us", self.mo_time)
        rospy.loginfo("CLOCKWISE:            %s", self.scanright)
        rospy.loginfo("SPEED:                %s m/s", self.speed)

        # This device is not Dual Channel so skip the “V3B” Gain Parameter
        # block: 0x01 for normal, 0x1D for extended V3B Gain Parameters.
        v3b = bitstring.pack("0x01")

        # Construct the HdCtrl bytes to control operation:
        #   Bit 0:  prf_AGC         0: AGC off(def) 1: AGC On
        #   Bit 1:  prf_alt         0: continuous   1: alternating scan
        #   Bit 2:  scanright       0: left         1: right
        #   Bit 3:  invert          0: upright      1: inverted
        #   Bit 4:  motoff          0: on           1: off
        #   Bit 5:  txoff           0: on           1: off (for testing)
        #   Bit 6:  prf_t10         0: default      1: N/A
        #   Bit 7:  chan2           0: default      1: N/A
        #   Bit 8:  prf_first       0: N/A          1: default
        #   Bit 9:  hasmot          0: lol          1: has a motor (always)
        #   Bit 10: prf_PingSync    0: no sync      1: Ping Sync control on, default
        #   Bit 11: prf_ScanSync    0: default      1: use when dual head profiler
        #   Bit 12: stareLLim       0: default      1: N/A
        #   Bit 13: prf_Master      0: N/A          1: default for single profiler
        #   Bit 14: prf_Mirror      0: default      1: N/A
        #   Bit 15: IgnoreSensor    0: default      1: emergencies
        hd_ctrl = bitstring.pack(
            "0b001000110000, bool, bool, bool, bool",
            self.inverted, self.scanright, self.prf_alt, self.agc
        )

        hd_ctrl.byteswap()  # Little endian please.

        # Set the sonar type: 0x05 for Profiler.
        # Profiler allows also Imaging functionality (0x02)
        hd_type = bitstring.pack("0x05")

        # TX/RX transmitter constants:
        # f = transmitter frequency in Hertz
        # TxN = f*2^32/32e6
        # RxN = (f+455000)*2^32/32e6
        f_txrx_ch1 = 1100000 # in Hz
        f_txrx_ch2 = 1100000 # in Hz
        TxN_ch1 = int(f_txrx_ch1*pow(2,32)/(32*pow(10,6)))
        TxN_ch2 = int(f_txrx_ch2 * pow(2, 32) / (32 * pow(10, 6)))
        RxN_ch1 = int((f_txrx_ch1 + 455000)*pow(2,32)/(32 * pow(10,6)))
        RxN_ch2 = int((f_txrx_ch2 + 455000)*pow(2,32)/(32 * pow(10,6)))

        # TX pulse length: length of sonar pulse in microseconds. Variable with rangescale
        # use default ofs and mul
        Ofs = 10
        Mul = 25
        TxPulseLen = (self.range + Ofs)*Mul/10

        # Packing Tx/Rx constants and  TX pulse length
        tx_rx = bitstring.pack('uintle:32,uintle:32,uintle:32,uintle:32,uintle:16',
                                TxN_ch1, TxN_ch2, RxN_ch1, RxN_ch2, TxPulseLen)

        # Range scale does not control the sonar, only provides a way to note
        # the current settings in a human readable format.
        # The lower 14 bits are the range scale * 10 units and the higher 2
        # bits are coded units:
        #   0: meters
        #   1: feet
        #   2: fathoms
        #   3: yards
        # Only the metric system is implemented for now, because it is better.
        range_scale = bitstring.pack("uintle:16", int(self.range * 10))

        # Left/right angles limits are in 1/16th of a gradian.
        _left_angle = to_sonar_angles(self.left_limit)
        _right_angle = to_sonar_angles(self.right_limit)
        left_limit = bitstring.pack("uintle:16", _left_angle)
        right_limit = bitstring.pack("uintle:16", _right_angle)

        # The ADC Threshold sets the analogue threshold level on the receiver and controls
        # the sensitive of the profiler. Higher threshold makes the device less sensitive
        # Default value for Seaking Profiler is 50
        adc_threshold = bitstring.pack("uint:8", int(50))

        # This is the level of Adaptive Gain Control (%) that is applied to the receiver.
        # For SeaKing Profilers use default of 20
        filt_gain = bitstring.pack("uint:8", int(20))


        # Set the initial gain of each channel of the sonar receiver.
        # The gain ranges from 0 to 210.
        AGC_Max = int(self.gain * 210)
        AGC_SetPoint = 90 # default value
        AGC_args = bitstring.pack("uint:8, uint:8",AGC_Max, AGC_SetPoint)
        #

        # Slope setting is according to the sonar frequency
        # The equation is slope = f*2/55+106
        _slope_ch1 = (f_txrx_ch1-1210)*3/79000+150
        _slope_ch2 = (f_txrx_ch2-1210)*3/79000+150
        slope = bitstring.pack('uintle:16, uintle:16', _slope_ch1,_slope_ch2)

        # Set the high speed limit of the motor in units of 10 microseconds.
        mo_time = bitstring.pack("uint:8", int(self.mo_time / 10))

        # Set the step angle size in 1/16th of a gradian.
        #   32: low resolution
        #   24: medium resolution
        #   16:  high resolution
        #   8:  ultimate resolution
        _step_size = to_sonar_angles(self.step)
        step = bitstring.pack("uint:8", _step_size)

        # ScanTime in 10 ms units.
        nbins = bitstring.pack("uintle:16", self.nbins)
        _interval = 2 * self.range * self.nbins/ self.speed / 10e-3
        _interval = _interval + 1 if _interval % 2 else _interval
        ScanTime = bitstring.pack("uintle:16", int(_interval))

        # Spare bytes, fill with zero
        PrfSp1 = bitstring.pack('uintle:16', 0)

        # Controls for special operation. Normally set to zero
        PrfCtl2 = bitstring.pack('uintle:16', 0)

        # Factory defaults. Don't ask.
        lockout = bitstring.pack("uintle:16", self.lockout)
        minor_axis = bitstring.pack("uintle:16", 1600)
        major_axis = bitstring.pack("uint:8", 1)

        # Ctl2 is for testing.
        ctl2 = bitstring.pack("uint:8", 0)

        # Special devices setting. Should be left blank.
        scanz = bitstring.pack("uint:8, uint:8", 0, 0)

        # Order and construct bitstream.
        bitstream = (
            v3b, hd_ctrl, hd_type, tx_rx, range_scale, left_limit, right_limit,
            adc_threshold, filt_gain, AGC_args, slope, mo_time, step, ScanTime, PrfSp1,
            PrfCtl2, lockout, minor_axis, major_axis, ctl2, scanz
        )

        payload = bitstring.BitStream()
        for chunk in bitstream:
            payload.append(chunk)

        self.send(Message.HEAD_COMMAND, payload)
        rospy.logwarn("Parameters are sent")

    def reverse(self):
        print 'reverse'
        """Instantaneously reverses scan direction."""
        payload = bitstring.pack("0x0F")
        self.send(Message.HEAD_COMMAND, payload)
        self.scanright = not self.scanright

    def _ping(self):
        print '_ping'
        """Commands the sonar to ping once."""
        # Get current time in milliseconds.
        now = datetime.datetime.now()
        current_time = datetime.timedelta(
            hours=now.hour, minutes=now.minute,
            seconds=now.second, microseconds=0
        )
        payload = bitstring.pack(
            "uintle:32",
            current_time.total_seconds() * 1000
        )

        # Reset offset for up time.
        self._time_offset = current_time - self.up_time

        # Send command.
        self.send(Message.SEND_DATA, payload)

    def __parse_head_data(self, data):
        print '__parse head data'
        """Parses mtHeadData payload and returns parsed bins.

        Args:
            data: mtHeadData bitstring.

        Returns:
            Bins.

        Raise:
            ValueError: If data could not be parsed.
        """
        # Any number of exceptions could occur here if the packet is corrupted,
        # so a catch-all approach is used for safety.
        try:
            # Get the total number of bytes.
            count = data.read(16).uintle
            rospy.logdebug("Byte count is %d", count)

            # The device type should be 0x05 for a Profiling Sonar.
            device_type = data.read(8)
            if device_type.uint != 0x05:
                # Packet is likely corrupted, try again.
                raise ValueError(
                    "Unexpected device type: {}"
                    .format(device_type.hex)
                )

            # Get the head status byte:
            #   Bit 0:  'HdPwrLoss'. Head is in Reset Condition.
            #   Bit 1:  'MotErr'. Motor has lost sync, re-send Parameters.
            #   Bit 2:  'PrfSyncErr'. Always 0.
            #   Bit 3:  'PrfPingErr'. Always 0.
            #   Bit 4:  Whether prf_AGC is enabled.
            #   Bit 5:  RESERVED (ignore).
            #   Bit 6:  RESERVED (ignore).
            #   Bit 7:  Message appended after last packet data reply.
            _head_status = data.read(8)
            rospy.logdebug("Head status byte is %s", _head_status)
            if _head_status[-1]:
                rospy.logerr("Head power loss detected")
            if _head_status[-2]:
                rospy.logerr("Motor lost sync")
                self.set(force=True)

            # Get the sweep code. Its value should correspond to:
            #   0: Scanning normal.
            #   1: Scan at left limit.
            #   2: Scan at right limit.
            #   3: RESERVED (ignore).
            #   4: RESERVED (ignore)
            #   5: Scan at center position.
            sweep = data.read(8).uint
            rospy.logdebug("Sweep code is %d", sweep)
            if sweep == 1:
                rospy.loginfo("Reached left limit")
            elif sweep == 2:
                rospy.loginfo("Reached right limit")

            # Get the HdCtrl bytes to control operation:
            #   Bit 0:  prf_AGC         0: AGC off(def) 1: AGC on
            #   Bit 1:  prf_alt         0: continuous   1: alternate scan
            #   Bit 2:  scanright       0: left         1: right
            #   Bit 3:  invert          0: upright      1: inverted
            #   Bit 4:  motoff          0: on           1: off
            #   Bit 5:  txoff           0: on           1: off (for testing)
            #   Bit 6:  prf_t10         0: default      1: N/A
            #   Bit 7:  chan2           0: default      1: N/A
            #   Bit 8:  prf_first       0: N/A          1: default
            #   Bit 9:  hasmot          0: lol          1: has a motor (always)
            #   Bit 10: prf_PingSync    0: sync off     1: sync on, default
            #   Bit 11: prf_ScanSync    0: default      1: use when Dual Head Profiler
            #   Bit 12: stareLLim       0: default      1: N/A
            #   Bit 13: prf_Master      0: N/A          1: default for single profiler
            #   Bit 14: prf_Mirror        0: default      1: N/A
            #   Bit 15: IgnoreSensor    0: default      1: emergencies
            # Should be the same as what was sent.
            hd_ctrl = data.read(16)
            hd_ctrl.byteswap()  # Little endian please.
            self.inverted, self.scanright, self.prf_alt, self.prf_AGC = (
                hd_ctrl.unpack("pad:12, bool, bool, bool, bool")
            )
            rospy.logdebug("Head control bytes are %s", hd_ctrl.bin)
            rospy.logdebug("PRF Adaptive Gain Control mode %s", self.prf_AGC)
            rospy.logdebug("PRF Alternate Scan mode %s", self.prf_alt)
            rospy.logdebug("Scanning right %s", self.scanright)

            # Range scale.
            # The lower 14 bits are the range scale * 10 units and the higher 2
            # bits are coded units:
            #   0: meters
            #   1: feet
            #   2: fathoms
            #   3: yards
            # Only the metric system is implemented for now, because it is
            # better.
            self.range = data.read(16).uintle / 10.0
            rospy.logdebug("Range scale is %f", self.range)

            # TX/RX transmitter constants: N/A to DST.
            data.read(32)

            # The gain ranges from 0 to 210.
            self.gain = data.read(8).uintle
            rospy.logdebug("Gain is %f", self.gain)

            # Slope setting is N/A to DST.
            data.read(16)

            # ADC analog threshold (1/255 units)
            adc_threshold = data.read(8).uintle
            filt_gain = data.read(8).uintle
            rospy.logdebug("filt Gain is %f", filt_gain)

            # Left/right angles limits are in 1/16th of a gradian.
            self.left_limit = to_radians(data.read(16).uintle)
            self.right_limit = to_radians(data.read(16).uintle)
            rospy.logdebug(
                "Limits are %f to %f",
                self.left_limit, self.right_limit
            )


            # Step angle size.
            self.step = to_radians(data.read(8).uint)
            rospy.logdebug("Step size is %f", self.step)

            ScanTime = data.read(16).uintle
            rospy.logdebug("Scan Time is %d", ScanTime)

            # Dbytes is the number of bytes with data to follow.
            dbytes = data.read(16).uintle
            self.nbins = dbytes
            bin_size = 8
            rospy.logdebug("DBytes is %d", dbytes)

            # Get bins.
            bins = [data.read(bin_size).uint for i in range(self.nbins)]
        except Exception as e:
            # Damn.
            raise ValueError(e)

        return bins

    def scan(self, callback):
        print 'scan'
        """Sends scan command.

        This method is blocking but calls callback at every reply with the
        heading and a new dataset.

        To stop a scan, simply call the preempt() method. Otherwise, the scan
        will run forever.

        The intensity at every bin is an integer value ranging between 0 and
        255.

        Args:
            callback: Callback for feedback.
                Called with args=(sonar, slice)
                where sonar is this sonar instance and slice is a SonarSlice
                instance.

        Raises:
            SonarNotInitialized: Sonar is not initialized.
            SonarNotConfigured: Sonar is not configured for scanning.
        """
        # Verify sonar is ready to scan.
        self.update()
        if self.no_params or not self.has_cfg:
            raise exceptions.SonarNotConfigured(self.no_params, self.has_cfg)

        # Timeout count to keep track of how many failures in a row occured.
        # This will then try to recover by resetting the sonar parameters.
        timeout_count = 0
        MAX_TIMEOUT_COUNT = 5

        # Scan until stopped.
        self.preempted = False
        while not self.preempted:
            # Preempt on ROS shutdown.
            if rospy.is_shutdown():
                self.preempt()
                return

            # Ping the sonar.
            self._ping()

            # Get the scan data.
            try:
                data = self.get(Message.HEAD_DATA, wait=1).payload
                timeout_count = 0
            except exceptions.TimeoutError:
                timeout_count += 1
                rospy.logdebug("Timeout count: %d", timeout_count)
                if timeout_count >= MAX_TIMEOUT_COUNT:
                    # Try to resend parameters.
                    self.set(force=True)
                    timeout_count = 0
                # Try again.
                continue

            try:
                bins = self.__parse_head_data(data)
            except ValueError as e:
                # Try again.
                rospy.logerr("Failed to parse head data: %r", e)
                continue

            # Generate configuration.
            config = {
                key: self.__getattribute__(key)
                for key in (
                    "inverted", "prf_alt", "scanright",
                    "prf_AGC", "gain", "filt_gain", "adc_threshold",
                    "left_limit", "right_limit",
                    "range", "nbins", "step", "lockout"
                )
            }

            # Run callback.
            slice = ScanSlice(self.heading, bins, config)
            callback(self, slice)

    def preempt(self):
        print 'preempt'
        """Preempts a scan in progress."""
        rospy.logwarn("Preempting scan...")
        self.preempted = True

    def reboot(self):
        print 'reboot'
        """Reboots Sonar.

        Raises:
            SonarNotInitialized: Sonar is not initialized.
        """
        rospy.logwarn("Rebooting sonar...")
        self.send(Message.REBOOT)
        self.open()

    def update(self):
        print 'update'
        """Updates Sonar states from mtAlive message.

        Note: This is a blocking function.

        Raises:
            SonarNotInitialized: Sonar is not initialized.
        """
        # Wait until successful no matter what.
        while True:
            try:
                self.get(Message.ALIVE)
                return
            except exceptions.TimeoutError:
                continue

    def __update_state(self, alive):
        print '___update state'
        """Updates Sonar states from mtAlive message.

        Args:
            alive: mtAlive reply.
        """
        payload = alive.payload
        payload.bytepos = 1

        # Get current time and compute up time.
        micros = payload.read(32).uintle * 1000
        self.clock = datetime.timedelta(microseconds=micros)
        if self._time_offset > self.clock:
            self._time_offset = datetime.timedelta(0)
        self.up_time = self.clock - self._time_offset

        # Get heading.
        self.heading = to_radians(payload.read(16).uintle)

        # Decode HeadInf byte.
        head_inf = payload.read(8)
        self.recentering = head_inf[0]
        self.centred = head_inf[1]
        self.motoring = head_inf[2]
        self.motor_on = head_inf[3]
        self.scanright = head_inf[4]
        self.scanning = head_inf[5]
        self.no_params = head_inf[6]
        self.has_cfg = head_inf[7]

        rospy.loginfo("UP TIME:     %s", self.up_time)
        rospy.logdebug("RECENTERING: %s", self.recentering)
        rospy.logdebug("CENTRED:     %s", self.centred)
        rospy.logdebug("MOTORING:    %s", self.motoring)
        rospy.logdebug("MOTOR ON:    %s", self.motor_on)
        rospy.logdebug("CLOCKWISE:   %s", self.scanright)
        rospy.logdebug("SCANNING:    %s", self.scanning)
        rospy.logdebug("NO PARAMS:   %s", self.no_params)
        rospy.logdebug("HAS CFG:     %s", self.has_cfg)


class Resolution(object):

    """Sonar mechanical resolution enumeration.

    The mechanical resolution is the angle the step motor rotates per scan line
    in radians. A higher resolution slows down the scan.

    Set as such:

        with TritechProfiler() as sonar:
            sonar.set(step=Resolution.LOW)

    Other resolutions can also be used, but these are the ones documented by
    Tritech.
    """
    LOWEST = to_radians(255)  # Not recommended.
    LOWER = to_radians(128)  # Not recommended.
    LOWERISH = to_radians(64)  # Not recommended.
    LOW = to_radians(32)
    MEDIUM = to_radians(24)
    HIGH = to_radians(16)
    ULTIMATE = to_radians(8)
