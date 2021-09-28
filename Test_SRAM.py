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
# instr.reset()

## DACS
DAC_val = {}
DAC_val['VBGPA'] = -0.
DAC_val['VBGNA'] = 0.
DAC_val['VBGNC'] = 0.
DAC_val['VBGPC'] = -0.
DAC_val['VBGNM'] = 0.
DAC_val['VBGPM'] = -0.
instr.update_DAC(DAC_val)

## SPI
# instr.set_mode('addr')
# instr.set_mode('counter')
instr.set_mode('lmt_counter', start=0, stop=63)
# instr.set_mode('register', values=list(range(64))[::2])
# instr.set_mode('sram', values=list(range(64))))
# instr.set_mode('direct')

instr.set_clk(int_clk=True, osc_vco=0.3, two_cycles=False, add_delay=False) # 0.85
instr.set_output(mux_mat=True, line0=None, line1=None, column0=None, column1=None)

## FAST SEQUENCE
seq = []
seq += [fs.Trig_out(address=3, clk=False, trig=[False]*4)]
seq += [fs.Wait(value=10, precision='1us')]
seq += [fs.Trig_out(address=3, clk=True, trig=[False]*4)]
seq += [fs.Wait(value=10, precision='1us')]
seq += [fs.JumpFor(target=0, count=0)] # play infinitely
seq += [fs.End()]

instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=10, 
                    trig_reset_states=[True]*10, 
                    start_after=False, 
                    start_index=0)

# instr.start_seq()
# time.sleep(0.2)


## SRAM
code = instr.SPI_read(0xFE, 1)[0]
# instr.SPI_write(0xFE, code & ~0x04)
# instr.SPI_write(0xFE, code | 0x04)
# time.sleep(0.1)
# instr.SPI_write(0xFE, code & ~0x04)
# time.sleep(0.1)
# instr.SPI_dump_all()
instr.SPI_sram_write(0x00, list(range(64))[::-1], sel_mem='ctl')
time.sleep(0.1)
# instr.SPI_write(0xFE, 0b00001000)
ans = instr.SPI_sram_read(0x00, 64, sel_mem='both')
print(ans[0])
print(ans[1])

## CLOSE INSTRUMENT
# instr.stop_seq()
instr.FPGA.close()

