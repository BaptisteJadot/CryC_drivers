# -*- coding: utf-8 -*-
"""
Created on Thu Nov 18 13:08:08 2021

@author: manip.batm
"""

from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
import Fastseq_elmts as fs
import numpy as np
import matplotlib.pyplot as plt
import time
import h5py as h5
        
def build_waveforms(values=[0.9]*64, current_index=63, current_val=0.):
    # build lists
    CLK = [0., 1.8] * len(values)
    V0 = np.array([[v] * 4 for v in values[::2]]).flatten().tolist()
    V1 = np.array([[v] * 4 for v in values[1::2]]).flatten().tolist()
    
    # correct for even / odd number of values
    if len(values) % 2 == 0:
        V0 += [V0[-1]] * 2
        V1 = [current_val] * 2 + V1
    else:
        V1 = [current_val] * 2 + V1 + [V1[-1]] * 2
        
    # correct for starting index parity
    if current_index % 2 != 0:
        Veven = np.array(V0)
        Vodd = np.array(V1)
    else:
        Vodd = np.array(V1)
        Veven = np.array(V0)

    return Veven, Vodd, CLK

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
instr.set_mode('register', values=list(range(64)))
# instr.set_mode('sram', values=list(range(64))))
# instr.set_mode('direct')

instr.set_clk(int_clk=False, osc_vco=0., two_cycles=False, add_delay=False)
instr.set_output(mux_mat=False, line0=None, line1=None, column0=None, column1=17//2)

## INCREMENT CLOCK UNTIL ADDRESS 0
i0 = instr.SPI_read(0xA1, 1)[0]
print(i0)
while i0 != 0:
    seq = []
    seq += [fs.Panel4(address=0, clk=False, even_value=0.9, odd_value=0.9)]
    seq += [fs.Panel4(address=0, clk=True, even_value=0.9, odd_value=0.9)]
    seq += [fs.Panel4(address=0, clk=False, even_value=0.9, odd_value=0.9)]
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

## PREPARE SEQUENCE
Vsweep = np.linspace(0.6, 1.2, 61)
N0 = 50
seq = []
for i in range(65):
    seq += [fs.Panel4(clk=False, even_value=0.9, odd_value=0.9)]
    seq += [fs.Panel4(clk=True, even_value=0.9, odd_value=0.9)]

seq += [fs.Panel4(clk=True, even_value=Vsweep[0], odd_value=0.9)]
seq += [fs.Wait(value=100, precision='1us')]

for Vi in Vsweep:
    seq += [fs.Panel4(clk=True, even_value=Vi, odd_value=0.9)]
    seq += [fs.ADC_get(channel=3)]
    seq += [fs.JumpFor(target=len(seq)-1, count=N0-1)] # read 50 times
    
seq += [fs.Panel4(clk=True, even_value=0.9, odd_value=Vsweep[0])]
seq += [fs.Wait(value=100, precision='1us')]

for Vi in Vsweep:
    seq += [fs.Panel4(clk=True, even_value=0.9, odd_value=Vi)]
    seq += [fs.ADC_get(channel=3)]
    seq += [fs.JumpFor(target=len(seq)-1, count=N0-1)] # read 50 times

seq += [fs.End()]

instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=10, 
                    trig_reset_states=[True]*10, 
                    start_after=False, 
                    start_index=0)
time.sleep(2)
_ = instr.get_ADC_data(Npts=None)

## RUN
data = np.zeros((N0, len(Vsweep), 2, 64))
for i in range(64):
    addr = instr.SPI_read(0xA1, 1)[0]
    instr.start_seq()
    time.sleep(0.1)
    # d = instr.get_ADC_data(Npts=None)
    d = np.array(instr.get_ADC_data(Npts=2*N0*len(Vsweep)), dtype=np.float)
    data[:, :, :, i] = d.reshape((N0, len(Vsweep), 2), order='F')
    print(addr, d.shape)
   
## CLOSE INSTRUMENT
instr.stop_seq()
instr.FPGA.close()

## SAVE DATA
fname = '''D:\Baptiste\CIR7\\4K\\Crosstalk_V15_vs_all_2.h5'''
with h5.File(fname, 'a') as f:
    if 'data' in f.keys():
        f.close()
        raise ValueError("File already exists")
    else:
        f.create_dataset('data', data=data)
        f.create_dataset('Vsweep', data=Vsweep)
        f.create_dataset('addr', data=list(range(64)))
        grp = f.create_group("Settings")
        grp.attrs.create("SPI_settings", SPI_state)
        grp.attrs.create("i0", i0)
        grp.attrs.create("fast_seq", "\n".join([s.__str__() for s in seq]))
        f.close()