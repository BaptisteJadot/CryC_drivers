# -*- coding: utf-8 -*-
"""
Created on Mon Dec 14 09:17:20 2020
@author: Baptiste
"""

def build_order(tup):
    """
    Combines a list of tuples (integer, number of bits) to a int64 order.
    MSB to LSB
    e.g.: convert_int_tuples_to_bin([(1,2),(2,4)]) -> 18 ('010010')
    """
    if any([t[0]>2**t[1]-1 for t in tup]) or sum([t[1] for t in tup])>64:
        raise ValueError('Number of bits is incorrect')
    else:
        order = 0
        offset = 0
        for t in tup[::-1]:
            order += int(t[0])<<offset
            offset += t[1]
        return order

def uint16_to_float(code):
    """
    Converts a 16 bit unsigned integer to a floating point number in the DAC range [-5,5].
    e.g.: uint16_to_float(2**16-1) -> +5.0
    """
    if code not in range(0,2**16):
        raise ValueError('Input is not a uint16')
    else:
        return code*10.0/(2**16-1)-5

def float_to_uint16(val):
    """
    Converts a floating point number in the DAC range [-5,5] to a 16 bit unsigned integer into .
    e.g.: float_to_uint16(5) -> 2**16-1
    """
    if val<-5 or val>5:
        raise ValueError('Input not in the DAC range ([-5,5])')
    else:
        return int((val+5)*(2**16-1)/10)