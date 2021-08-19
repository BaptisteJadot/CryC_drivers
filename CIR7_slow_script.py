# -*- coding: utf-8 -*-
"""
Created on Tue Aug 17 16:51:53 2021

@author: manip.batm
"""

import numpy as np
from CIR7_driver import CIR7_driver
from CIR7_default_config import SPI_content, DAC_dict
from FPGA_utils import build_order,uint16_to_float,float_to_uint16
import time
    
# open instrument
bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main.lvbitx"""
ip_address = "192.168.1.21"
instr = CIR7_driver(ip_address,bitfile_path,DAC_dict)
print(instr.FPGA.fpga_vi_state)

# write all to SPI
for addr,val in SPI_content.items():
    instr.SPI_write(addr,[val])
time.sleep(0.1)
    
# read all to check
SPI_readable = list(SPI_content.keys())
SPI_readable += [0x60,0x61,0x80,0x81,0xA0,0xA1,0xA2]
SPI_readable += list(range(0x90,0x98))
SPI_output = instr.SPI_read(0x00,256)
print([hex(add)+':'+hex(val) for (add,val) in SPI_output if add in SPI_readable])

# set all DC DACs
DAC_val = {}
DAC_val['VE0N'] = 0
DAC_val['VE1N'] = 0.
DAC_val['MODE_ROT'] = 0.
DAC_val['SEL_MAT'] = 1.8
DAC_val['OSC_VCO'] = 0.

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

# set all CIR7 targets
# instr.panel4_config(start_ind=8,stop_ind=32,ticks_per_address=8000,ticks_per_bit=10)
# values_dict = {i:0. for i in range(63)}
# values_dict = {}
# values_dict[16] = 1.5
# values_dict[17] = 0.5
# values_dict[2] = 0.
# values_dict[3] = 1.5
# instr.panel4_set_value(values_dict)
# instr.panel4_stop()
# instr.panel4_start()

# read all DACs & CIR7 targets
DC_val, CIR7_val = instr.read_current_values()
print(DC_val)
# print(CIR7_val)


# set fast_seq
instr.config_seq(slots={},us_per_DAC=10,trig_reset_states=[True]*10,start_after=False,start_index=0)
seq = []
seq += [build_order([(5,8),(4,8),(len(seq),8),(0x82,8),(0,28),(0,4)])] # All trigger to False
seq += [build_order([(5,8),(4,8),(len(seq),8),(1,2),(0,6),(float_to_uint16(0.),16),(float_to_uint16(0.),16)])] # DAC output
seq += [build_order([(5,8),(4,8),(len(seq),8),(0x81,8),(0,16),(100,16)])] # Wait 100 us
Epair = [0.8,1.5]
Eimpair = [1.6,1.]



# for i in range(0,4):
#     addr = 1
#     seq += [build_order([(5,8),(4,8),(len(seq),8),(1,2),(addr,6),(float_to_uint16(Epair[int(i/2)]),16),(float_to_uint16(Eimpair[int(i/2)]),16)])] # DAC output
#     seq += [build_order([(5,8),(4,8),(len(seq),8),(0,2),(addr,6),(float_to_uint16(Epair[int(i/2)]),16),(float_to_uint16(Eimpair[int(i/2)]),16)])] # DAC output
#     seq += [build_order([(5,8),(4,8),(len(seq),8),(1,2),(addr,6),(float_to_uint16(Epair[int(i/2)]),16),(float_to_uint16(Eimpair[int(i/2)]),16)])] # DAC output
seq += [build_order([(5,8),(4,8),(len(seq),8),(1,2),(2,6),(float_to_uint16(0.8),16),(float_to_uint16(1.8),16)])] # DAC output
seq += [build_order([(5,8),(4,8),(len(seq),8),(1,2),(2,6),(float_to_uint16(1),16),(float_to_uint16(1.8),16)])] # DAC output
seq += [build_order([(5,8),(4,8),(len(seq),8),(1,2),(2,6),(float_to_uint16(1.2),16),(float_to_uint16(1.8),16)])] # DAC output
seq += [build_order([(5,8),(4,8),(len(seq),8),(1,2),(2,6),(float_to_uint16(1.2),16),(float_to_uint16(1.8),16)])] # DAC output
seq += [build_order([(5,8),(4,8),(len(seq),8),(1,2),(2,6),(float_to_uint16(1.),16),(float_to_uint16(1.8),16)])] # DAC output
seq += [build_order([(5,8),(4,8),(len(seq),8),(0,2),(1,6),(float_to_uint16(0.8),16),(float_to_uint16(1.8),16)])] # DAC output
seq += [build_order([(5,8),(4,8),(len(seq),8),(0,2),(1,6),(float_to_uint16(0.8),16),(float_to_uint16(1.8),16)])] # DAC output
seq += [build_order([(5,8),(4,8),(len(seq),8),(0,2),(1,6),(float_to_uint16(1),16),(float_to_uint16(1.8),16)])] # DAC output
seq += [build_order([(5,8),(4,8),(len(seq),8),(1,2),(2,6),(float_to_uint16(1.2),16),(float_to_uint16(1.8),16)])] # DAC output
# seq += [build_order([(5,8),(4,8),(len(seq),8),(0,2),(16,6),(float_to_uint16(0.8),16),(float_to_uint16(1.8),16)])] # DAC output
# seq += [build_order([(5,8),(4,8),(len(seq),8),(0,2),(16,6),(float_to_uint16(0.5),16),(float_to_uint16(1.5),16)])] # DAC output
# seq += [build_order([(5,8),(4,8),(len(seq),8),(1,2),(16,6),(float_to_uint16(0.5),16),(float_to_uint16(1.5),16)])] # DAC output
seq += [build_order([(5,8),(4,8),(len(seq),8),(0x82,8),(0,28),(15,4)])] # All trigger to True
seq += [build_order([(5,8),(4,8),(len(seq),8),(0x81,8),(0,16),(1,16)])] # Wait 1 us
# seq += [build_order([(5,8),(4,8),(len(seq),8),(0x84,8),(0,12),(5,10),(1,10)])] # Jump 5 times
seq += [build_order([(5,8),(4,8),(len(seq),8),(0x80,8),(0,32)])] # End
instr.update_slots_lowlevel(seq,start_after=False,start_index=0)

time.sleep(0.5)
instr.start_seq()

# close instr comm
instr.FPGA.close()
    