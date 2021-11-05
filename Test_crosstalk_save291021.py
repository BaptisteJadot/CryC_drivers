# -*- coding: utf-8 -*-
"""
Created on Tue Oct 26 12:46:42 2021

@author: manip.batm
"""

from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
import Fastseq_elmts as fs
import numpy as np
import matplotlib.pyplot as plt
import time
  
## OPEN INSTRUMENT
bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main_analog_addr.lvbitx"""
ip_address = "192.168.1.21"
instr = CIR7Driver(ip_address, bitfile_path, DAC_dict)
print(instr.FPGA.fpga_vi_state)

## RESET CIR7
# instr.reset()

## DACS
DAC_val = {}
# DAC_val['MODE_ROT'] = 0. # auto-set by following functions
# DAC_val['SEL_MAT'] = 1.8
# DAC_val['OSC_VCO'] = 0.
# DAC_val['RESETN'] = 1.8

DAC_val['VBGPA'] = -2.
DAC_val['VBGNA'] = 2.
DAC_val['VBGNC'] = 0.
DAC_val['VBGPC'] = 0.
DAC_val['VBGNM'] = 0.
DAC_val['VBGPM'] = 0.

DAC_val['CMD_R0'] = 0.
DAC_val['CMD_R1'] = 0.
DAC_val['CMD_R2'] = 0.
DAC_val['CMD_R3'] = 0.
DAC_val['CMD_R4'] = 0.
DAC_val['CMD_R5'] = 0.
DAC_val['CMD_R6'] = 0.
instr.update_DAC(DAC_val)

## SPI
# instr.set_mode('addr')
# instr.set_mode('counter')
# instr.set_mode('lmt_counter', start=0, stop=63)
instr.set_mode('register', values=list(range(64)))
# instr.set_mode('sram', values=list(range(64))))
# instr.set_mode('direct')

instr.set_clk(int_clk=False, osc_vco=0., two_cycles=False, add_delay=False)
instr.set_output(mux_mat=False, line0=14//2, line1=None, column0=None, column1=None)

## INCREMENT CLOCK UNTIL ADDRESS 0
i0 = instr.SPI_read(0xA1, 1)[0]
print(i0)
while i0 != 0:
    seq = []
    seq += [fs.Panel4(address=0, clk=False, even_value=0.5, odd_value=0.5)]
    seq += [fs.Panel4(address=0, clk=True, even_value=0.5, odd_value=0.5)]
    seq += [fs.Panel4(address=0, clk=False, even_value=0.5, odd_value=0.5)]
    seq += [fs.End()]
    instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                        us_per_DAC=10, 
                        trig_reset_states=[True]*10, 
                        start_after=True, 
                        start_index=0)
    time.sleep(0.05)
    i0 = instr.SPI_read(0xA1, 1)[0]
    print(i0)
SPI_state = instr.SPI_dump_all(output_to_console=True)

## RUN
v = [0.5]*64
# v[13] = 0.
v[14] = 1.
seq = []
seq += [fs.Trig_out(trig=[True]*4)]
for i in range(32+16): # write value to all + loop to addr 13
    seq += [fs.Panel4(clk=False, even_value=v[(2*i)%64], odd_value=v[(2*i-1)%64])]
    seq += [fs.Panel4(clk=True, even_value=v[(2*i)%64], odd_value=v[(2*i-1)%64])]
    seq += [fs.Panel4(clk=False, even_value=v[(2*i)%64], odd_value=v[(2*i+1)%64])]
    seq += [fs.Panel4(clk=True, even_value=v[(2*i)%64], odd_value=v[(2*i+1)%64])]

seq += [fs.Panel4(clk=False, even_value=0.5, odd_value=0.5)]
seq += [fs.Panel4(clk=True, even_value=0.5, odd_value=0.5)]
print((2*i+2)%64)
seq += [fs.Trig_out(trig=[False]*4)]
seq += [fs.Panel4(clk=False, even_value=0.8, odd_value=0.5)]
seq += [fs.Wait(value=15, precision='1us')]
seq += [fs.Panel4(clk=False, even_value=1.2, odd_value=0.5)]
seq += [fs.Wait(value=15, precision='1us')]
seq += [fs.JumpFor(target=len(seq)-4, count=100)] # keep on changing V15
seq += [fs.End()]

instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=10, 
                    trig_reset_states=[True]*10, 
                    start_after=True, 
                    start_index=0)
time.sleep(2)
    
# CLOSE INSTRUMENT
instr.stop_seq()
instr.FPGA.close()

# CTALK_02 : 14 @ 1V, 15 @ 0.8/1.2V
# CTALK_03 : 14 @ 1V, 15 @ 0.5V, 16 @ 0.5V, 17 @ 0.8/1.2V