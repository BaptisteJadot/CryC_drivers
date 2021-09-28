# -*- coding: utf-8 -*-
"""
Created on Thu Aug 26 14:06:45 2021

@author: Baptiste
"""

from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
import Fastseq_elmts as fs
import numpy as np
import time

## OPEN INSTRUMENT
bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main.lvbitx"""
ip_address = "192.168.1.21"
instr = CIR7Driver(ip_address, bitfile_path, DAC_dict)
print(instr.FPGA.fpga_vi_state)

## RESET CIR7
instr.reset()

## DACS
DAC_val = {}
# DAC_val['MODE_ROT'] = 0. # auto-set by following functions
# DAC_val['SEL_MAT'] = 1.8
# DAC_val['OSC_VCO'] = 0.
# DAC_val['RESETN'] = 1.8

DAC_val['VBGPA'] = 0.
DAC_val['VBGNA'] = 0.
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
instr.set_mode('addr')
# instr.set_mode('counter')
# instr.set_mode('lmt_counter', start=0, stop=3)
# instr.set_mode('register', values=list(range(64)))
# instr.set_mode('register', values=[14]*64)
# instr.set_mode('sram', values=list(range(64))))
# instr.set_mode('direct')

# instr.set_output(mux_mat=False, line0=14//2, line1=15//2, column0=16//2, column1=17//2)
instr.set_output(mux_mat=True, line0=0//2, line1=1//2, column0=None, column1=None)

instr.set_clk(int_clk=True, osc_vco=0.48, two_cycles=False, add_delay=False)

## FAST SEQUENCE
v = [-0.2, 0.2, 0.6, 0.6]
seq = [fs.Trig_out(address=3, clk=True, trig=[False]*4)]
seq += [fs.Panel4(even_value=v[0], odd_value=v[1])]
seq += [fs.Trig_out(address=3, clk=True, trig=[False]*4)]
seq += [fs.Panel4(even_value=v[0], odd_value=v[1])]
seq += [fs.Trig_out(address=3, clk=True, trig=[False]*4)]
seq += [fs.Panel4(even_value=v[2], odd_value=v[1])]
seq += [fs.Trig_out(address=3, clk=True, trig=[False]*4)]
seq += [fs.Panel4(even_value=v[2], odd_value=v[1])]
seq += [fs.Trig_out(address=3, clk=True, trig=[False]*4)]
seq += [fs.Panel4(even_value=v[2], odd_value=v[3])]
seq += [fs.Trig_out(address=3, clk=True, trig=[False]*4)]
seq += [fs.Panel4(even_value=v[2], odd_value=v[3])]
seq += [fs.Trig_out(address=3, clk=True, trig=[False]*4)]
seq += [fs.Panel4(even_value=v[0], odd_value=v[3])]
seq += [fs.Trig_out(address=3, clk=True, trig=[False]*4)]
seq += [fs.Panel4(even_value=v[0], odd_value=v[3])]
# seq += [fs.Wait(value=1, precision='1us')]
seq += [fs.JumpFor(target=0, count=0)] # play infinitely
seq += [fs.End()]

for s in seq:
    print(s)    
    
instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=6, 
                    trig_reset_states=[True]*10, 
                    start_after=False, 
                    start_index=0)

## READ CURRENT STATE
instr.stop_seq()
# i0 = instr.SPI_read(0xA0, 1)[0]
# print(i0)
# print((i0*4),seq[4*i0])
# instr.start_seq(start_ind=((i0+1)%4)*4)
instr.start_seq(start_ind=0)

## CLOSE INSTRUMENT
instr.FPGA.close()

