# -*- coding: utf-8 -*-
"""
Created on Thu Sep  9 15:51:01 2021

@author: manip.batm
"""

from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
from FPGA_utils import build_order
import numpy as np
import time
    
## OPEN INSTRUMENT
bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main.lvbitx"""
ip_address = "192.168.1.21"
instr = CIR7Driver(ip_address, bitfile_path, DAC_dict)
# print(instr.FPGA.fpga_vi_state)

DAC_val = {}
DAC_val['VBGNC'] = 0.5
DAC_val['VBGPC'] = -0.5
DAC_val['VBGNM'] = 0.
DAC_val['VBGPM'] = 0.
instr.update_DAC(DAC_val)

for i in range(10):
    for speed_ticks in range(800,300,-1):
        ## SET SPI SPEED
        instr.send_orders([build_order([(2, 8), (4, 8), (0, 16), (speed_ticks, 32)])]) # SPI speed
        
        ## SEND RANDOM ADDRESSES
        addr = np.random.randint(0, 64, 64)
        instr.SPI_write(0, addr)
        
        ## READ REGISTER CONTENT
        ans = instr.SPI_read(0, 64)
        if all(ans==addr):
            print(f'Success @ {80/speed_ticks*1000:.2f} kHz ({speed_ticks} ticks).')
        else:
            print(f'{np.sum(ans!=addr)} errors detected @ {80/speed_ticks*1000:.2f} kHz ({speed_ticks} ticks).')
            break

instr.FPGA.close()