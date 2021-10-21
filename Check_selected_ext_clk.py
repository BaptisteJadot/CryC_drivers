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
bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main_analog_addr.lvbitx"""
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
addr = 0b010111
# addr = 32
if addr & 8 > 0:
    print('Invalid address')
    addr = 0
# instr.set_mode('addr')
# instr.set_mode('counter')
instr.set_mode('lmt_counter', start=0, stop=63)
# instr.set_mode('register', values=list(range(64)))
# instr.set_mode('register', values=[addr]*64)
# instr.set_mode('sram', values=[addr]*64)
# instr.set_mode('direct')
# 

instr.set_output(mux_mat=False, line0=None,  line1=None, column0=None, column1=None)

instr.set_clk(int_clk=False, osc_vco=0., two_cycles=False, add_delay=False)

# code = instr.SPI_read(0xFE, 1)[0]
# instr.SPI_write(0xFE, code & ~0x04)
# instr.SPI_write(0xFE, code | 0x04)

_ = instr.SPI_dump_all(True)

## FAST SEQUENCE
seq = []
# seq += [fs.Trig_out(trig=[False]*4)]
seq += [fs.Panel4(address=addr, clk=False, even_value = 0., odd_value = 0.)]
seq += [fs.Panel4(address=addr, clk=True, even_value = 0., odd_value = 0.)]
seq += [fs.JumpFor(target=addr, count=1)] # 2 addresses
seq += [fs.Panel4(address=0, clk=False, even_value = 0., odd_value = 0.)]
# seq += [fs.Panel4(address=addr, clk=True, even_value = 0., odd_value = 0.)]
# seq += [fs.Trig_out(trig=[True]*4)]
# seq += [fs.SPIRead(address=0xA0, Nbytes=3)]
# seq += [fs.SPIRead(address=0x90, Nbytes=8)]
# # seq += [fs.Wait(value=15000, precision='1us')]
# seq += [fs.JumpFor(target=0, count=20)] # 20 times
seq += [fs.End()]
instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=100, 
                    trig_reset_states=[True]*10, 
                    start_after=False, 
                    start_index=0)

# ## READ CURRENT STATE
# SPI_output = []
# start = time.time()
# while len(SPI_output) < 11*20 and time.time()-start < 2:
#     SPI_output += instr.SPI_empty_buffer()
#     time.sleep(0.1)

# str_out = 'A0, A1, A2, selected\n'
# str_out += '-' * 42 + '\n'
# for k in range(20):
#     if [addr for (addr, val) in SPI_output[:11]] != [0xA0, 0xA1, 0xA2] + list(range(0x90, 0x98)):
#         print ('Error parsing answer')
#     else:
#         for (_, data) in SPI_output[:3]:
#             str_out += f'{data:02X}' + ', '
#         sel = []
#         for (addr, data) in SPI_output[3:11]:
#             sel += [(addr-0x90)*8+j for j in range(8) if (data>>j)&1==1]
#         str_out += '[' + ', '.join([f'{s:02X}' for s in sel]) + ']\n'
# print(str_out)

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
    instr.start_seq(start_ind=0)
    time.sleep(0.1)
    instr.stop_seq()
print(str_out)


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

