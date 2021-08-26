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
    addr and clk are converted to 0V or 1.8V.
    even/odd values are [-5V ; +5V] floats.
    """
    def __init__(self, addr = 0, clk = True, even_value = 0., odd_value = 0.):
        self.addr = addr
        self.clk = clk
        self.even_value = even_value
        self.odd_value = odd_value

    def gen_order(self):
        if self.addr & 8 > 0:
            raise ValueError('Addr[3] must be 0V.')
        else:
            even_code = float_to_uint16(self.even_value)
            odd_code = float_to_uint16(self.odd_value)
            return build_order([(1 if self.clk else 0, 2), (self.addr, 6), (even_code, 16), (odd_code, 16)])

    def __str__(self):
        return f'Addr {self.addr}, clk {self.clk}, even {self.even_value}V, odd {self.odd_value}V'

class End():
    """
    Sequence slot stopping the sequence execution.
    """
    def __init__(self):
        pass

    def gen_order(self):
        return build_order([(0x80, 8), (0, 32)])

    def __str__(self):
        return 'End'

class Wait():
    """
    Sequence slot waiting the indicated duration (in us).
    precision is either '1us', '10us', '100us' or '1ms'.
    if value > (2**16 - 1) * precision, the precision is increased.
    """
    def __init__(self, value, precision = '1us'):
        self.value = value
        self.precision = precision

    def gen_order(self):
        if self.precision not in ['1us', '10us', '100us', '1ms']:
            raise ValueError('Time precision not understood')
        else:
            s = ['1us', '10us', '100us', '1ms'].index(self.precision)
            if self.value > (2**16-1) * 10**s:
                s = int(np.ceil(np.log10(self.value / (2**16-1))))
                if s>3:
                    raise ValueError('Value is too large')
                else:
                    self.precision = ['1us', '10us', '100us', '1ms'][s]
            val = int(np.round(self.value / 10**s))
            return build_order([(0x81, 8), (0, 14), (s, 2), (val, 16)])

    def __str__(self):
        return 'Wait {}us [{}]'.format(self.value, self.precision)

class Trig_out():
    """
    Sequence slot updating the 10 trigger_out outputs (panel 9).
    status is a list of 10 booleans, trig n°0 first.
    """
    def __init__(self, status = [True] * 10):
        self.status = status

    def gen_order(self):
        code = sum([b << i for (i, b) in enumerate(self.status)])
        return build_order([(0x82, 8), (0, 22), (code, 10)])

    def __str__(self):
        return 'Trig out [' + ','.join(['T' if c else 'F' for c in self.status]) + ']'
        
class Trig_in():
    """
    Sequence slot waiting for a specific trigger status on panel 8.
    monitor indicates which trigger inputs need to correspond.
    status and monitor are lists of 10 booleans, trig n°0 first.
    """
    def __init__(self, status = [True] * 10, monitor = [False] * 10):
        self.status = status
        self.monitor = monitor

    def gen_order(self):
        s = sum([b << i for (i, b) in enumerate(self.status)])
        m = sum([b << i for (i, b) in enumerate(self.monitor)])
        return build_order([(0x83, 8), (0, 12), (m, 10), (s, 10)])

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
        return build_order([(0x84, 8), (0, 12), (self.count, 10), (self.target, 10)])

    def __str__(self):
        return 'Jump to {} x {}'.format(self.target, self.count if self.count == 0 else 'inf')

class SPIRead():
    """
    Sequence slot reading Nbytes from the SPI.
    address is the initial address to read from.
    """
    def __init__(self, address, Nbytes):
        self.address = address
        self.Nbytes = Nbytes

    def gen_order(self):
        return build_order([(0x85, 8), (0, 8), (self.address, 8), (self.Nbytes, 16)])

    def __str__(self):
        if self.Nbytes == 1:
            return 'Read SPI addr {:02x}'.format(self.address)
        else:
            return 'Read SPI addr {:02x} to {}'.format(self.address, self.address + self.Nbytes)

class SPIWrite():
    """
    Sequence slot writing to one address of the SPI.
    """
    def __init__(self, address, data):
        self.address = address
        self.data = data

    def gen_order(self):
        return build_order([(0x86, 8), (0, 16), (self.address, 8), (self.data, 8)])

    def __str__(self):
        return 'Write {:02x} to SPI at addr {:02x}'.format(self.data, self.address)
