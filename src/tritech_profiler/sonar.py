# -*- coding: utf-8 -*-

"""Tritech Profiler sonar."""

import rospy
import serial
import datetime
import bitstring
import exceptions
from math import pow
from tsocket import Socket
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

        .. data:: agc

            This bit is used to enable/disable the Adaptive Gain Control (AGC) in the sonar receiver. *AGCMax*
            and *SetPoint* determine the levels for this control. *AGC* applies automatic refining of the
            receiver Gain. If false applies 'manual' Gain (*AGC_Max*).

            *AGC* will always act on the maximum received echo, and as a result the ``prf_first`` control will
            de-activate when ``agc`` is enabled.

            **True** if automatic refining of receiver gain, **False** manual gain.

            **Default = True**


        .. data:: prf_alt

            This bit governs whether Profiler scanning should be alternative as per a car windscreen wiper (i.e.
            direction = Scan Alternate), or whether scanning should in one direction only defined by the
            ``scanright`` bit (Scan Right or Scan Left).

            **True** if alternating scan, **False** if scan in one direction.

            **Default = False**


        .. data:: scanright

            This bit determines the scanning direction when ``prf_alt`` (Bit 1) = 0. It is ignored when ``prf_alt`` = 1.

            scanright = 0 = Profiler scans anticlockwise when viewed from top (Scan Left).

            scanright =1 = Clockwise scan rotation (Scan Right).

            **Default = 1**

        .. data:: inverted

            This bit allows the rotation directions to be reversed if the Profiler head is mounted inverted, i.e.
            when the transducer boot is pointing backwards against direction of travel rather than forwards.

            **Default = 0 = Profiler mounted 'upright', transducer boot pointing forward.**

        .. data:: left_limit

            Left limit of sector scan in radians. ``LeftLim`` refers to the Anti-Clockwise scan limit.

            ``scanright`` = True  = 1st ping is at ``left_limit``

            ``scanright`` = False = 1st ping is at ``right_limit``

            The units are in 1/16th Gradian units, in the range [0, 6399]

            The SeaKing direction convention is as follows, with transducer boot forwards:

            Left 90° = 1600

            Ahead = 3200

            Right 90° = 4800

            Astern = 0 (or 6399)

            The ``cont`` bit in ``HdCtrl`` will override these limits, and allow continuous rotation.
            The ``stareLLim`` bit in ``HdCtrl`` will cause the head to be driven to the ``LeftLim`` position and
            "stare" in that direction. If ``LeftLim`` = ``RightLim``, then the head will act as if the ``stareLLim``
            bit is set.

            To Scan a 90° sector ahead, set:

            LeftLim = 2400 (= Left 45°)

            RightLim = 4000 (= Right 45°)

        .. data:: right_limit

            Left limit of sector scan in radians. ``right_limit`` refers to the Clockwise scan limit.

            ``scanright`` = True  = 1st ping is at ``left_limit``

            ``scanright`` = False = 1st ping is at ``right_limit``

            The units are in 1/16th Gradian units, in the range [0, 6399]

            The SeaKing direction convention is as follows, with transducer boot forwards:

            Left 90° = 1600

            Ahead = 3200

            Right 90° = 4800

            Astern = 0 (or 6399)

            The ``cont`` bit in ``HdCtrl`` will override these limits, and allow continuous rotation.
            The ``stareLLim`` bit in ``HdCtrl`` will cause the head to be driven to the ``LeftLim`` position and
            "stare" in that direction. If ``LeftLim`` = ``RightLim``, then the head will act as if the ``stareLLim``
            bit is set.

            To Scan a 90° sector ahead, set:

            LeftLim = 2400 (= Left 45°)

            RightLim = 4000 (= Right 45°)

        .. data:: adc_threshold

            This sets the analogue detect threshold level on the receiver and controls the sensitivity of the Profiler.
            A higher detect threshold will make the device less sensitive.

            **Default = 50**

        .. data:: filt_gain

            This is the level of Adaptive Gain Control (%) that is applied to the receiver.

            **Default = 20**

        .. data:: mo_time

            High speed limit of the motor in units of 10 microseconds.

            **Default = 25 to 40 (for Profiling mode)**

        .. data:: step

            This byte sets the scanning motor step angle between ‘pings’, and is in units of 1/16 Gradians. The
            SeaKing default settings are:

            Low Resolution = 32 (= 2 Gradians = 1.8 °)

            Medium Resolution = 24 (= 1.5 Gradian = 1.35°)

            High Resolution = 16 (= 1 Gradian = 0.9°)

            Ultimate Resolution (= 0.5 Gradian = 0.45°)

        .. data:: lockout

            This value is the time in microseconds at which the Receiver will start sampling after it has transmitted.
            The factory default setting for this is:

            - Lockout = 271 (for 1100/1210 kHz Profiler channel)
            - Lockout = 542 (for 580/600 kHz Profiler channel)

            **Default = 100**

        .. data:: centred

            Whether the sonar motor is centred.

        .. data:: clock

            Sonar time of the day.

        .. data:: conn

            Serial connection.

        .. data:: gain

            Initial gain percentage (0.00-1.00).

        .. data:: has_cfg

            Whether the sonar has acknowledged with mtBBUserData.

        .. data:: heading

            Current sonar heading in radians.

        .. data:: initialized

            Whether the sonar has been initialized with parameters.

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

        .. data:: scanning

            Whether the sonar is scanning.

        .. data:: speed

            Speed of sound in medium.

            **Default = 1500 m/s**


    """

    def __init__(self, port="/dev/ttyUSB0", **kwargs):
        """
        Constructs Sonar object.

        :param port: Serial port (default: /dev/sonar).

        :param kwargs: Key-word arguments to pass to set() on initialization

        :return:

        """
        print 'init'
        # Parameter defaults.
        self.adc_threshold = 50.0
        self.filt_gain = 20.00
        self.agc = False
        self.prf_alt = False
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

        self.port_enabled = True

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
        self.baudrate = 115200
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

        :raises: SonarNotFound: Sonar port could not be opened.

        """
        print 'open'
        if not self.conn:
            try:
                self.conn = Socket(self.port,self.baudrate)
            except OSError as e:
                raise exceptions.SonarNotFound(self.port, e)

        # Update properties.
        rospy.loginfo("Initializing sonar on %s", self.port)
        self.initialized = True

        # Reboot to make sure the sonar is clean.
        self.send(Message.REBOOT)
        self.update()
        self.send(Message.REBOOT)
        #rospy.loginfo('Sending Version Data request...')
        #self.send(Message.SEND_VERSION)
        #self.get(Message.VERSION_DATA, 1)
        #rospy.loginfo('Version Data message received')

        #self.send(Message.SEND_BB_USER)
        #self.get(Message.BB_USER_DATA)

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

    def get(self, message=None, wait=4):
        """
        Sends command and returns reply.

        :param message: Message to expect (default: first to come in).

        :param kwargs: wait: Seconds to wait until received if a specific message is required (default: 2).

        :return: Reply.

        :raises SonarNotInitialized: Attempt reading serial without opening port.

        :raises TimeoutError: Process timed out while waiting for specific message.

        """
        print 'get'
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
                self.port_enabled = True
                #print self.port_enabled
                if self.port_enabled:
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
                #print ('port enabled:',self.port_enabled)
                #rospy.sleep(0.1)
            except exceptions.PacketCorrupted, serial.SerialException:
                # Keep trying.
                continue

        # Timeout.
        rospy.logerr("Timed out before receiving message: %s", expected_name)
        raise exceptions.TimeoutError()

    def send(self, command, payload=None):
        """
        Sends command and returns reply.

        :param command: Command to send.
        :param payload: Fields of the command packet
        :return: Reply

        :raises SonarNotInitialized: Attempt sending command without opening port.
        """
        print 'send'
        if not self.initialized:
            raise exceptions.SonarNotInitialized(command, payload)

        self.conn.send(command, payload)

    def set(self, agc=None, prf_alt=None, scanright=None, step=None,
            filt_gain=None, adc_threshold=None, left_limit=None, right_limit=None,
            mo_time=None, range=None, gain=None, speed=None, lockout = None,
            inverted=None, force=False, port_enabled =True,
            profiler_port_baudrate=115200, profiler_port="/dev/ttyUSB0"):
        """
        Sends Sonar head command with new properties if needed.

        Only the parameters specified will be modified.

        If initialized, arguments are compared to current properties in order
        to see if sending the command is necessary.

        :param filt_gain: Percentage of Adaptive Gain Control applied to receiver.
        :param adc_threshold: Analogue detect threshold level.
        :param agc: True if automatic AGC, otherwise manual AGC.
        :param prf_alt: True if alternate scan, otherwise continuous scan.
        :param gain: Initial gain percentage (0.00-1.00).
        :param inverted: Whether the sonar is mounted upside down.
        :param left_limit: Left limit of sector scan in radians.
        :param mo_time: High speed limit of the motor in units of 10 microseconds.
        :param range: Scan range in meters.
        :param right_limit: Right limit of sector scans in radians.
        :param scanright: Whether the sonar scanning direction is clockwise.
        :param speed: Speed of sound in medium.
        :param step: Mechanical resolution (Resolution enumeration).
        :param force: Whether to force setting the parameters or not.
        :param port_enabled: enables/disables virtual serial port
        :param profiler_port_baudrate: stablishes port baudrate for communication
        :param profiler_port: name of serial port where the profiler is connected

        :raises SonarNotInitialized: Sonar is not initialized.
        """
        print 'set'
        if not self.initialized:
            raise exceptions.SonarNotInitialized()

        self.__set_parameters(
            agc=agc, prf_alt=prf_alt, scanright=scanright,
            step=step, filt_gain=filt_gain, adc_threshold=adc_threshold, left_limit=left_limit,
            right_limit=right_limit, mo_time=mo_time, range=range,
            gain=gain, speed=speed, lockout=lockout, inverted=inverted, force=force, port_enabled=port_enabled,
            profiler_port_baudrate=profiler_port_baudrate, profiler_port = profiler_port
        )


    def __set_parameters(self, force, **kwargs):
        """
        Sends Sonar head command to set sonar properties. This function is called
        by ``set()`` method. See ``set()`` method for more information

        Only the parameters specified will be modified.

        If initialized, arguments are compared to current properties in order
        to see if sending the command is necessary.

        :param force: Whether to force set parameters.
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
        port_enabled = kwargs.get('port_enabled')
        if port_enabled:
            self.port_enabled = True

        else:
            self.port_enabled = False

        #self.port_enabled = True


        self.port = kwargs.get('profiler_port')
        self.baudrate = kwargs.get('profiler_port_baudrate')

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
        rospy.loginfo("PORT ENABLED:         %s", self.port_enabled)
        rospy.loginfo("PORT:                 %s", self.port)
        rospy.loginfo("BAUDRATE:             %s bauds", self.baudrate)

        # This device is not Dual Channel so skip the “V3B” Gain Parameter
        # block: 0x01 for normal, 0x1D for extended V3B Gain Parameters.
        v3b = bitstring.pack("0x1D")

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
        # hd_ctrl = bitstring.pack(
        #     "0b001000110000, bool, bool, bool, bool",
        #     self.inverted, self.scanright, self.prf_alt, self.agc
        # )
        # hd_ctrl = bitstring.pack(
        #     "0b 0010 1111 0000, bool, bool, bool, bool",
        #     self.inverted, self.scanright, self.prf_alt, self.agc
        # )
        hd_ctrl = bitstring.pack(
            "0b 011000111000, bool, bool, bool, bool",
            self.inverted, self.scanright, self.prf_alt, self.agc
        )

        hd_ctrl.byteswap()  # Little endian please.

        # Set the sonar type: 0x05 for Profiler.
        # Profiler allows also Imaging functionality (0x02)
        hd_type = bitstring.pack("0x05")

        # TX/RX transmitter constants:
        # f = transmitter frequency in Hertz

        f_txrx_ch1 = 600000 # in Hz
        f_txrx_ch2 = 1100000 # in Hz

        TxN_ch1 = int(f_txrx_ch1*pow(2,32)/(32*pow(10,6))) # = 147639500

        TxN_ch2 = int(f_txrx_ch2 * pow(2, 32) / (32 * pow(10, 6))) #= 147639500

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
        AGC_SetPoint = 189 # default value = 90, but Seanet sends this value...
        AGC_args = bitstring.pack("uint:8, uint:8",AGC_Max, AGC_SetPoint)
        #

        # Slope setting is according to the sonar frequency
        # The equation is slope = (f*7/1000+11560)/142
        _slope_ch1 = (f_txrx_ch1*7/1000+11560)/142
        _slope_ch2 = (f_txrx_ch2*7/1000+11560)/142
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
        #_interval = 2 * self.range * self.nbins/ self.speed / 10e-3
        #_interval = _interval + 1 if _interval % 2 else _interval
        _interval = 30 # Seanet as well
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

        v3b_adth1     = bitstring.pack('0x32')
        v3b_adth2     = bitstring.pack('0x32')
        v3b_filtgain1 = bitstring.pack('0x01')
        v3b_filtgain2 = bitstring.pack('0x14')
        v3b_agcmax1   = bitstring.pack('0x69')
        v3b_agcmax2   = bitstring.pack('0x69')
        v3b_setpoint1 = bitstring.pack('0xBD')
        v3b_setpoint2 = bitstring.pack('0xBD')
        v3b_slope1    = bitstring.pack('uintle:16', 110)
        v3b_slope2    = bitstring.pack('uintle:16', 150)
        v3b_sloped1   = bitstring.pack('uintle:16', 0)
        v3b_sloped2   = bitstring.pack('uintle:16', 0)

        # Order and construct bitstream.
        bitstream = (
            v3b, hd_ctrl, hd_type, tx_rx, range_scale, left_limit, right_limit,
            adc_threshold, filt_gain, AGC_args, slope, mo_time, step, ScanTime, PrfSp1,
            PrfCtl2, lockout, minor_axis, major_axis, ctl2, scanz, v3b_adth1,
            v3b_adth2, v3b_filtgain1, v3b_filtgain2, v3b_agcmax1, v3b_agcmax2,
            v3b_setpoint1, v3b_setpoint2, v3b_slope1, v3b_slope2, v3b_sloped1,
            v3b_sloped2
        )

        payload = bitstring.BitStream()
        for chunk in bitstream:
            payload.append(chunk)

        if port_enabled:
            self.conn.conn.port = self.port
            self.conn.conn.baudrate = self.baudrate
            self.conn.open()
            self.send(Message.HEAD_COMMAND, payload)
            rospy.logwarn("Parameters are sent")
        else:
            self.conn.close()
        return hd_ctrl,hd_type,TxN_ch1,TxN_ch2,RxN_ch1,RxN_ch2,TxPulseLen,tx_rx,
        range_scale, left_limit,right_limit,adc_threshold,filt_gain,
        AGC_args,slope,mo_time,step,ScanTime,lockout,minor_axis,
        major_axis,payload

    def reverse(self):
        """
        Instantaneously reverses scan direction.
        """
        payload = bitstring.pack("0x0F")
        self.send(Message.HEAD_COMMAND, payload)
        self.scanright = not self.scanright

    def _ping(self):
        """
        Commands the sonar to ping once.
        """
        print '_ping'
        # Get current time in milliseconds.
        now = datetime.datetime.now()
        current_time = datetime.timedelta(
            hours=now.hour, minutes=now.minute,
            seconds=now.second, microseconds=0
        )
        payload = bitstring.pack("uintle:8 , uintle:8, uintle:8, uintle:8, uintle:8, uintle:8, uintle:32",
                                 255, 20, 7, 25, 128, 20,current_time.total_seconds() * 1000
        )

        # Reset offset for up time.
        self._time_offset = current_time - self.up_time

        # Send command.
        self.send(Message.SEND_DATA, payload)

    def __parse_head_data(self, data):
        """
        Parses mtHeadData payload and returns parsed bins.

        :param mtHeadData bitstring.

        :return: Bins.

        :raises ValueError: If data could not be parsed.
        """
        print '_parse_head:data'
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
            bin_size = 16 # antes 8
            rospy.logdebug("DBytes is %d", dbytes)

            # Get bins.
            bins = [data.read(bin_size).uintle for i in range(self.nbins)]
            print bins
        except Exception as e:
            # Damn.
            raise ValueError(e)

        return bins

    def scan(self, callback):
        """
        Sends scan command.

        This method is blocking but calls callback at every reply with the
        heading and a new dataset.

        To stop a scan, simply call the *preempt()* method. Otherwise, the scan
        will run forever.

        The intensity at every bin is an integer value ranging between 0 and
        255.

        :param callback: Callback for feedback.
                Called with args=(sonar, slice)
                where sonar is this sonar instance and slice is a *SonarSlice*
                instance.

        :raises SonarNotInitialized: Sonar is not initialized.
        :raises SonarNotConfigured: Sonar is not configured for scanning.
        """
        print 'scan'
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
        """
        Preempts a scan in progress.
        """
        rospy.logwarn("Preempting scan...")
        self.preempted = True

    def reboot(self):
        """
        Reboots Sonar.

        :raises SonarNotInitialized: Sonar is not initialized.
        """
        rospy.logwarn("Rebooting sonar...")
        self.send(Message.REBOOT)
        self.open()

    def update(self):
        """
        Updates Sonar states from *mtAlive* message.

        Note: This is a blocking function.

        :raises SonarNotInitialized: Sonar is not initialized.
        """
        print 'Update'
        # Wait until successful no matter what.
        while True:
            try:
                self.get(Message.ALIVE)
                return
            except exceptions.TimeoutError:
                continue

    def __update_state(self, alive):
        """
        Updates Sonar states from mtAlive message.

        :param alive: mtAlive reply.
        """
        print '_update_state'
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
    """
    Sonar mechanical resolution enumeration.

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
