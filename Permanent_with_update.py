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

def update_perm(seq, addr, new_val):
    ind = [(2 * addr + k) % 128 + 1 for k in [0,2]]
    new_slots = {}
    for i in ind:
        if addr % 2 == 0: 
            seq[i].even_value = new_val
        else:
            seq[i].odd_value = new_val
        new_slots[i] = seq[i]
    instr.update_slots(new_slots, stop_before=False, start_after=False)
    
## OPEN INSTRUMENT
bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main_analog_addr.lvbitx"""
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
# instr.set_mode('addr')
# instr.set_mode('counter')
# instr.set_mode('lmt_counter', start=0, stop=63)
instr.set_mode('register', values=list(range(64)))
# instr.set_mode('sram', values=list(range(64)))
# instr.set_mode('direct')

instr.set_clk(int_clk=False, osc_vco=0., two_cycles=False, add_delay=False)
ind = 14
v = [0.]*64
v[14] = 0.
v[17] = 0.

# v = np.linspace(0,0.63*2,64)+0.2
# v[0] = 1.5
# v[1::2] = 1.

instr.set_output(mux_mat=False, line0=None, line1=None, column0=None, column1=None)
# instr.set_output(mux_mat=False, line0=14//2, line1=17//2, column0=None, column1=None)

## FAST SEQUENCE
seq = [fs.Trig_out(trig=[False]*4)]
for i in range(32):
    seq += [fs.Panel4(address=0, clk=False, even_value=v[(2*i)%64], odd_value=v[(2*i-1)%64])]
    seq += [fs.Panel4(address=0, clk=True, even_value=v[(2*i)%64], odd_value=v[(2*i-1)%64])]
    seq += [fs.Panel4(address=0, clk=False, even_value=v[(2*i)%64], odd_value=v[(2*i+1)%64])]
    seq += [fs.Panel4(address=0, clk=True, even_value=v[(2*i)%64], odd_value=v[(2*i+1)%64])]
# seq += [fs.Panel4(address=0, clk=False, even_value=v[62], odd_value=v[63])]
# seq = [fs.Trig_out(trig=[True]*4)]
# seq += [fs.Wait(value=1000000, precision='1ms')]
seq += [fs.JumpFor(target=1, count=0)] # play infinitely
seq += [fs.End()]

instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=5, 
                    trig_reset_states=[True]*10, 
                    start_after=False, 
                    start_index=0)

## READ CURRENT STATE
SPI_state = instr.SPI_dump_all(output_to_console=True)
instr.stop_seq()
i0 = instr.SPI_read(0xA1, 1)[0]+0
print(i0)
print(((i0)%64)*2+1,seq[((i0)%64)*2+1])
instr.start_seq(start_ind=((i0)%64)*2+1)
time.sleep(2)

# for i in range(2,64,2):
#     if i>2:
#         update_perm(seq, i-2, 0.)
#     update_perm(seq, i, 0.5)
#     time.sleep(0.2)

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

