# -*- coding: utf-8 -*-
"""
Created on Mon Oct 18 09:23:38 2021

@author: Baptiste
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
instr.set_mode('register', values=[14,17]*32)
# instr.set_mode('sram', values=list(range(64))))
# instr.set_mode('direct')

instr.set_clk(int_clk=False, osc_vco=0., two_cycles=False, add_delay=True)
instr.set_output(mux_mat=False, line0=14, line1=17, column0=None, column1=None)

## INCREMENT CLOCK UNTIL ADDRESS 14
i0 = instr.SPI_read(0xA1, 1)[0]
print(i0)
while i0 != 14:
    seq = []
    seq += [fs.Panel4(address=0, clk=False, even_value=0., odd_value=0.)]
    seq += [fs.Panel4(address=0, clk=True, even_value=0., odd_value=0.)]
    seq += [fs.Panel4(address=0, clk=False, even_value=0., odd_value=0.)]
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

## FAST SEQUENCE
seq = []
seq += [fs.Trig_out(trig=[False]*4)]

for i in range(50):
    seq += [fs.Panel4(clk=False, even_value=0.3, odd_value=0.8)]
    seq += [fs.Panel4(clk=True, even_value=0.3, odd_value=0.8)] # 14
    
    seq += [fs.Panel4(clk=False, even_value=0.3, odd_value=0.6)]
    seq += [fs.Panel4(clk=True, even_value=0.3, odd_value=0.6)] # 17
    
    seq += [fs.Panel4(clk=False, even_value=0.5, odd_value=0.6)]
    seq += [fs.Panel4(clk=True, even_value=0.5, odd_value=0.6)] # 14
    
    seq += [fs.Panel4(clk=False, even_value=0.5, odd_value=0.8)]
    seq += [fs.Panel4(clk=True, even_value=0.5, odd_value=0.8)] # 17
    
seq += [fs.End()]

instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=5, 
                    trig_reset_states=[True]*10, 
                    start_after=True, 
                    start_index=0)

# CLOSE INSTRUMENT
instr.stop_seq()
instr.FPGA.close()

