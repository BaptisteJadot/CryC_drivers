# -*- coding: utf-8 -*-
"""
Created on Mon Sep 27 14:52:03 2021

@author: manip.batm
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
_ = instr.SPI_empty_buffer()

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
# instr.set_mode('addr')
# instr.set_mode('counter')
# instr.set_mode('lmt_counter', start=0, stop=63)
# instr.set_mode('register', values=list(range(64)))
# instr.set_mode('register', values=[0x20]*64)
# instr.set_mode('sram', values=list(range(64)))
instr.set_mode('direct')
# 

instr.set_output(mux_mat=False, line0=None,  line1=None, column0=None, column1=None)

instr.set_clk(int_clk=True, osc_vco=0., two_cycles=False, add_delay=False)

# code = instr.SPI_read(0xFE, 1)[0]
# instr.SPI_write(0xFE, code & ~0x04)
# instr.SPI_write(0xFE, code | 0x04)

_ = instr.SPI_dump_all(True)

## FAST SEQUENCE
addr = 5
if addr & 8 > 0:
    print('Invalid address')
    addr = 0
seq = []
seq += [fs.Trig_out(address=addr, clk=True, trig=[True]*4)]
seq += [fs.End()]
instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=100, 
                    trig_reset_states=[True]*10, 
                    start_after=True, 
                    start_index=0)

## READ CURRENT STATE
str_out = 'A0, A1, A2, selected\n'
str_out += '-' * 42 + '\n'
for k in range(20):
    # read current indexes
    SPI_output = instr.SPI_read(0xA0, 3)
    for i, data in enumerate(SPI_output):
        str_out += f'{data:02X}' + ', '
    SPI_output = instr.SPI_read(0x90, 8)
    sel = []
    for i, data in enumerate(SPI_output):
        sel += [i*8+j for j in range(8) if (data>>j)&1==1]
    str_out += '[' + ', '.join([f'{s:02X}' for s in sel]) + ']\n'
    # run clk for a while
    instr.update_DAC({'OSC_VCO':0.46}) # 100kHz
    instr.update_DAC({'OSC_VCO':0.}) # disable
print(str_out)

# ## READ CURRENT STATE
# str_out = 'A0, A1, A2, 90, 91, 92, 93, 94, 95, 96, 97\n'
# str_out += '-' * 42 + '\n'
# for k in range(20):
#     # read current indexes
#     SPI_output = instr.SPI_read(0xA0, 3)
#     SPI_output += instr.SPI_read(0x90, 8)
#     for i, data in enumerate(SPI_output):
#         str_out += f'{data:02X}' + ('\n' if i == 10 else ', ')
#     # run clk for a while
#     instr.update_DAC({'OSC_VCO':0.46}) # 100kHz
#     instr.update_DAC({'OSC_VCO':0.}) # disable
# print(str_out)

## SRAM
# code = instr.SPI_read(0xFE, 1)[0]
# # instr.SPI_write(0xFE, code & ~0x04)
# instr.SPI_write(0xFE, code | 0x04)
# time.sleep(0.1)
# instr.SPI_write(0xFE, code & ~0x04)
# time.sleep(0.1)
# instr.SPI_dump_all()
# instr.SPI_sram_write(0x00, [0x11]*64, sel_mem='ctl')
# time.sleep(0.1)
# # instr.SPI_write(0xFE, 0b00001000)
# ans = instr.SPI_sram_read(0x00, 64, sel_mem='both')
# print(ans[0])
# print(ans[1])

# CLOSE INSTRUMENT
# instr.stop_seq()
instr.FPGA.close()

