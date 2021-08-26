# -*- coding: utf-8 -*-
"""
Created on Thu Aug 26 09:44:45 2021

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
with CIR7Driver(ip_address, bitfile_path, DAC_dict) as instr:
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
    # instr.set_mode('addr')
    # instr.set_mode('counter')
    instr.set_mode('lmt_counter', start=0, stop=63)
    # instr.set_mode('register', values=list(range(64)))
    # instr.set_mode('sram', values=list(range(64))))
    # instr.set_mode('direct', values=list(range(64))))

    instr.set_clk(int_clk=False)
    instr.set_output(mux_mat=True, line0=0, line1=1, column0=32, column1=33)

    ## FAST SEQUENCE
    seq = [fs.Trig_out(status=[False] * 4 + [True] * 6)]
    seq += [fs.Panel4(addr=0, clk=False, even_value=0., odd_value=0.)]
    seq += [fs.Panel4(addr=0, clk=True, even_value=0., odd_value=0.)]
    seq += [fs.SPIRead(addr=0xA0, Nbytes=3)]
    seq += [fs.Wait(value=1000)] # wait for data recovery
    seq += [fs.SPIRead(addr=0x90, Nbytes=8)]
    seq += [fs.Wait(value=1000)] # wait for data recovery
    seq += [fs.JumpFor(target=1, count=10)] # play 10 times
    seq += [fs.Trig_out(status=[True] * 10)]
    
    instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                        us_per_DAC=10, 
                        trig_reset_states=[True]*10, 
                        start_after=False, 
                        start_index=0)

    ## RUN
    instr.start_seq()
    time.sleep(0.5)

    ## PRINT RESULT
    SPI_output = instr.SPI_empty_buffer
    for (addr, data) in SPI_output:
        print('{addr:02x} : {data:02x}')
    

