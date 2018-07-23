#!/usr/bin/env python
import pytest
import os
import sys
import numpy as np
from sonar import TritechProfiler


class TestPS():
    def test_grad_conversion(self):
        # Set default values of profiler according to datasheet
        tp = TritechProfiler()
        tp.adc_threshold = 50.0,  #Default
        tp.filt_gain = 20.00, # Default
        tp.range = 2.00, # Default
        tp.gain = 1.616, # so agcmax gives default value
        tp.left_limit = 1600, # Default?
        tp.mo_time = 250, #default
        tp.nbins = 11, # Default
        tp.right_limit = 4800, # Default?
        tp.speed = 1500.0,
        tp.step = 16 ,# Default
        tp.lockout = 500, # Default
        tp.port_enabled = False
        tp.has_cfg = True
        tp.no_params = True
        tp.initialized = True

        tp.inverted = False, #Default
        tp.scanright = True, # Default
        tp.prf_alt = False, # Default
        tp.agc = False, # Default
        vars = tp.set(port_enabled = False,profiler_port = 'port',
                            profiler_port_baudrate = 115200)
        print vars
        assert 2 == 2
