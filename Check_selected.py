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
instr.set_clk(int_clk=False, osc_vco=0., two_cycles=False, add_delay=False)

# instr.set_mode('addr')
# instr.set_mode('counter')
# instr.set_mode('lmt_counter', start=0, stop=63)
instr.set_mode('register', values=list(range(64)))
# instr.set_mode('register', values=[0x20]*64)
# instr.set_mode('sram', values=list(range(64)))
# instr.set_mode('direct')
# 

instr.set_output(mux_mat=False, line0=None,  line1=None, column0=None, column1=None)

# code = instr.SPI_read(0xFE, 1)[0]
# instr.SPI_write(0xFE, code & ~0x04)
# instr.SPI_write(0xFE, code | 0x04)

_ = instr.SPI_dump_all(True)

## FAST SEQUENCE
seq = []
# for i in range(20):
seq += [fs.Trig_out(address=32, clk=True, trig=[True]*4)]
seq += [fs.Wait(value=5, precision='1us')]
seq += [fs.Trig_out(address=32, clk=False, trig=[False]*4)]
seq += [fs.Wait(value=5, precision='1us')]
seq += [fs.JumpFor(target=0, count=100)] # 100 times
seq += [fs.SPIRead(address=0xA0, Nbytes=3)]
seq += [fs.SPIRead(address=0x90, Nbytes=8)]
seq += [fs.Wait(value=15000, precision='1us')]
seq += [fs.JumpFor(target=0, count=20)] # 20 times
seq += [fs.End()]

instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=100, 
                    trig_reset_states=[True]*10, 
                    start_after=True, 
                    start_index=0)

## READ CURRENT STATE
str_out = 'A0, A1, A2, 90, 91, 92, 93, 94, 95, 96, 97\n'
str_out += '-' * 42 + '\n'
for k in range(100):
    time.sleep(0.05)
    SPI_output = instr.SPI_empty_buffer()
    # SPI_output = instr.SPI_read(0xA0, 3)
    # SPI_output += instr.SPI_read(0x90, 8)
    if len(SPI_output)==0:
        break
    # print(len(SPI_output))
    for (addr, data) in SPI_output:
        str_out += f'{data:02X}' + ('\n' if addr == 0x97 else ', ')
    # for i, data in enumerate(SPI_output):
    #     str_out += f'{data:02X}' + ('\n' if i == 10 else ', ')
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

