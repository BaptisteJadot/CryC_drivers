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
DAC_val['VE0N'] = 0
DAC_val['VE1N'] = 0.
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

DAC_val['CMD_R0'] = 1.8
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
# instr.set_mode('sram', values=list(range(64))))
# instr.set_mode('direct')

instr.set_clk(int_clk=False, osc_vco=0.0, two_cycles=False, add_delay=False)
instr.set_output(mux_mat=True, line0=None, line1=None, column0=0, column1=0)

## FAST SEQUENCE
seq = []
seq = [fs.Trig_out(status=[False] * 4 + [True] * 6)]
seq += [fs.ADC_get(channel=2)]
seq += [fs.Wait(value=100)]
seq += [fs.JumpFor(target=1, count=10)]
seq += [fs.Trig_out(status=[True] * 10)]
seq += [fs.End()]

instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=10, 
                    trig_reset_states=[True]*10, 
                    start_after=True, 
                    start_index=0)

## READ DATA
out_data = instr.get_ADC_data(Npts=None)
out_data = np.array(out_data,dtype=np.float)
# out_data = out_data.reshape((len(out_data)//2,2))
print(out_data)

## CLOSE INSTRUMENT
instr.FPGA.close()

