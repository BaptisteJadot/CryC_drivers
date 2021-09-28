# -*- coding: utf-8 -*-
"""
Created on Thu Sep  9 15:51:01 2021

@author: manip.batm
"""

from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
from FPGA_utils import build_order
import Fastseq_elmts as fs
import numpy as np
import time
import matplotlib.pyplot as plt
import h5py as h5


def test_SPI(instr, speed_ticks):
    ## SET SPI SPEED
    instr.send_orders([build_order([(2, 8), (4, 8), (0, 16), (speed_ticks, 32)])]) # SPI speed
    
    ## SEND RANDOM ADDRESSES
    addr = np.random.randint(0, 64, 64)
    instr.SPI_write(0, addr)
    
    ## READ REGISTER CONTENT
    ans = instr.SPI_read(0, 64)
    
    ## RETURN NUMBER OF ERRORS
    Nerr = sum(ans != addr)
    return Nerr
    
## OPEN INSTRUMENT
bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main.lvbitx"""
ip_address = "192.168.1.21"
instr = CIR7Driver(ip_address, bitfile_path, DAC_dict)
# print(instr.FPGA.fpga_vi_state)

instr.reset()

VDDC = 0.6
DAC_val = {}
DAC_val['VBGNC'] = 0.
DAC_val['VBGPC'] = 0.
DAC_val['VBGNM'] = 0.
DAC_val['VBGPM'] = 0.
instr.update_DAC(DAC_val)

## INT CLK
int_clk = False
osc_vco = 0.
instr.set_clk(int_clk, osc_vco, two_cycles=False, add_delay=False)
SPI_settings = instr.SPI_dump_all(False)

## EXT CLK
ext_clk_MHz = 0
instr.stop_seq()

VBG = np.linspace(-0.2,1.2,15)
N = 10
max_speed = np.zeros((len(VBG), N))

for i, VBGi in enumerate(VBG):
    DAC_val['VBGNC'] = +VBGi
    DAC_val['VBGPC'] = -VBGi
    instr.update_DAC(DAC_val)
    
    # FIRST ITERATION : DICHOTOMY
    m = 1
    M = 4000
    while abs(M-m) > 20:
        limit_speed = (m+M)//2
        if test_SPI(instr, limit_speed): # too fast
            m = limit_speed
        else:
            M = limit_speed        
    print(limit_speed)
    
    # NOW WE CAN START
    start = limit_speed + 40
    for k in range(N):
        for speed_ticks in range(start,100,-1):
            Nerr = test_SPI(instr, speed_ticks)
            if Nerr > 0:
                break
        max_speed[i,k] = np.NaN if speed_ticks == start else speed_ticks
        print(f'{VBGi:.1f} V : max speed {40/max_speed[i,k]*1000:.2f} kHz ({max_speed[i,k]} ticks).')

instr.FPGA.close()

## SAVE
# init_move_dt = np.dtype({'names':['name','value'],'formats':['S100','f8']})
# DAC_val_arr = np.array([(key, val) for key, val in DAC_val.items()], dtype=init_move_dt)
with h5.File('''D:\Baptiste\CIR7\\4K\SPI_speed_vs_BG_CD2.h5''', 'a') as f:
    i = 0
    while f'data_{i}' in f.keys():
        i += 1
    grp = f.create_group(f'data_{i}')
    grp.create_dataset('VBGNC', data=VBG)
    grp.create_dataset('VBGPC', data=-VBG)
    grp.create_dataset('max_speed', data=max_speed)
    grp.attrs.create('VDDC', VDDC)
    grp.attrs.create('SPI_settings', SPI_settings)
    grp.attrs.create('OSC_VCO', osc_vco)
    grp.attrs.create('int_clk', int_clk)
    grp.attrs.create('ext_clk_MHz', ext_clk_MHz)
    f.close()
