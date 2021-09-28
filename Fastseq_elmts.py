# -*- coding: utf-8 -*-
"""
Created on Tue Aug 24 17:22:26 2021

@author: Baptiste
"""

import numpy as np
from FPGA_utils import build_order, uint16_to_float, float_to_uint16

#--------------------------------------------------------------------------------
# Sequence elements for CIR7
#--------------------------------------------------------------------------------
class Panel4():
    """
    Sequence slot updating modified DAC panel (parallel addressing).
    even/odd values are [-5V ; +5V] floats.
    """
    def __init__(self, address=0, clk=False, even_value = 0., odd_value = 0.):
        self.even_value = even_value
        self.odd_value = odd_value
        self.address = address
        self.clk = clk

    def gen_order(self):
        even_code = float_to_uint16(self.even_value)
        odd_code = float_to_uint16(self.odd_value)
        return build_order([(1 if self.clk else 0, 2), (self.address, 6), (even_code, 16), (odd_code, 16)])

    def __str__(self):
        return f'Addr {self.address}, clk {"T" if self.clk else "F"}, Even input {self.even_value:.5f}V, odd input {self.odd_value:.5f}V'

class End():
    """
    Sequence slot stopping the sequence execution.
    """
    def __init__(self):
        pass

    def gen_order(self):
        return build_order([(0x8F, 8), (0, 32)])

    def __str__(self):
        return 'End'

class Wait():
    """
    Sequence slot waiting the indicated duration (in us).
    precision is either '1us', '10us', '100us', '1ms' or 'ticks'.
    if value > (2**16 - 1) * precision, the precision is increased.
    """
    def __init__(self, value, precision = '1us'):
        self.value = value
        self.precision = precision

    def gen_order(self):
        if self.precision not in ['ticks', '1us', '10us', '100us', '1ms']:
            raise ValueError('Time precision not understood')
        else:
            multipliers = {'ticks':1/80., '1us':1., '10us':10., '100us':100., '1ms':1000.}
            if self.value > 0xFFFF * multipliers[self.precision]: # increase precision
                s = int(np.ceil(np.log10(self.value / 0xFFFF))) # lowest power of 10
                if s>3:
                    raise ValueError('Value is too large')
                else:
                    self.precision = ['1us', '10us', '100us', '1ms'][s]
            val = int(np.round(self.value / multipliers[self.precision]))
            s = ['ticks', '1us', '10us', '100us', '1ms'].index(self.precision)
            return build_order([(0x81, 8), (0, 13), (s, 3), (val, 16)])

    def __str__(self):
        return 'Wait {}us [{}]'.format(self.value, self.precision)

class Trig_out():
    """
    Sequence slot updating the 11 trigger_out outputs (addr 0-6, clk, DIO 0-3).
    """
    def __init__(self, address=0, clk=False, trig=[True]*4):
        self.address = address
        self.clk = clk
        self.trig = trig

    def gen_order(self):
        code = sum([b << i for (i, b) in enumerate(self.trig)])
        return build_order([(0x82, 8), (0, 21), (code, 4), (self.clk, 1), (self.address, 6)])

    def __str__(self):
        return f'Addr {self.address}, clk {"T" if self.clk else "F"}, Trig [' + ','.join(['T' if c else 'F' for c in self.trig]) + ']'
        
class Trig_in():
    """
    Sequence slot waiting for a specific trigger status on panel 8.
    monitor indicates which trigger inputs need to correspond.
    status and monitor are lists of 4 booleans, trig n°0 first.
    """
    def __init__(self, status = [True] * 4, monitor = [False] * 4):
        self.status = status
        self.monitor = monitor

    def gen_order(self):
        s = sum([b << i for (i, b) in enumerate(self.status)])
        m = sum([b << i for (i, b) in enumerate(self.monitor)])
        return build_order([(0x83, 8), (0, 24), (m, 4), (s, 4)])

    def __str__(self):
        return 'Trig in [' + ','.join(['X' if not m else 'T' if v else 'F' for m,v in zip(self.monitor, self.status)]) + ']'

class JumpFor():
    """
    Sequence slot jumping to the indicated index.
    if count==0, the jump is never desactivated.
    """
    def __init__(self, target, count = 0):
        self.target = target
        self.count = count

    def gen_order(self):
        return build_order([(0x84, 4), (0, 8), (self.count, 12), (self.target, 12)])

    def __str__(self):
        return 'Jump to {} x {}'.format(self.target, self.count if self.count > 0 else 'inf')

class SPIRead():
    """
    Sequence slot reading Nbytes from the SPI.
    address is the initial address to read from.
    """
    def __init__(self, address, Nbytes):
        self.address = address
        self.Nbytes = Nbytes

    def gen_order(self):
        return build_order([(0x85, 4), (0, 8), (self.address, 8), (self.Nbytes, 16)])

    def __str__(self):
        if self.Nbytes == 1:
            return 'Read SPI addr {:02x}'.format(self.address)
        else:
            return 'Read SPI addr {:02x} to {:02x}'.format(self.address, self.address + self.Nbytes)

class SPIWrite():
    """
    Sequence slot writing to one address of the SPI.
    """
    def __init__(self, address, data):
        self.address = address
        self.data = data

    def gen_order(self):
        return build_order([(0x86, 4), (0, 16), (self.address, 8), (self.data, 8)])

    def __str__(self):
        return 'Write {:02x} to SPI at addr {:02x}'.format(self.data, self.address)

class ADC_get():
    """
    Sequence slot acquiring the current value for one ADC channel [0-15].
    """
    def __init__(self, channel):
        self.channel = channel

    def gen_order(self):
        return build_order([(0x87, 4), (0, 28), (self.channel, 4)])

    def __str__(self):
        return f'Read 1 point from ADC channel {self.channel}'
