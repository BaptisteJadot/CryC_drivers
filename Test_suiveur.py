# -*- coding: utf-8 -*-
"""
Created on Mon Sep  6 09:48:04 2021

@author: manip.batm
"""

from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
import Fastseq_elmts as fs
import numpy as np
import time
import matplotlib.pyplot as plt
import h5py as h5

## OPEN INSTRUMENT
bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main.lvbitx"""
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

DAC_val['VBGPA'] = -2
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
instr.set_mode('addr')
# instr.set_mode('counter')
# instr.set_mode('lmt_counter', start=0, stop=3)
# instr.set_mode('register', values=list(range(64)))
# instr.set_mode('sram', values=list(range(64))))
# instr.set_mode('direct')

instr.set_clk(int_clk=True, osc_vco=1., two_cycles=False, add_delay=False)

addr = 15
V = np.linspace(0.,1.8,91)
avg = 10
# if addr % 2 == 0:
#     instr.set_output(mux_mat=True, line0=None, line1=None, column0=addr//2, column1=None)
# else:
#     instr.set_output(mux_mat=True, line0=None, line1=None, column0=None, column1=addr//2)
instr.set_output(mux_mat=False, line0=0//2, line1=1//2, column0=2//2, column1=3//2)
## FAST SEQUENCE
seq = []
seq += [fs.Trig_out(address=addr, clk=False, trig=[False]*4)]
seq += [fs.Wait(value=1, precision='1us')]
seq += [fs.Trig_out(address=addr, clk=True, trig=[False]*4)]
seq += [fs.Wait(value=1, precision='1us')]
seq += [fs.JumpFor(target=0, count=50)]
for vi in V:
    seq += [fs.Panel4(even_value=vi, odd_value=vi)]
    seq += [fs.ADC_get(channel=2 if addr%2 == 0 else 3)]
    seq += [fs.JumpFor(target=len(seq)-2, count=avg-1)]
seq += [fs.Trig_out(address=addr, clk=False, trig=[True]*4)]
# seq += [fs.JumpFor(target=1, count=avg-1)]
seq += [fs.End()]

instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=10, 
                    trig_reset_states=[True]*10, 
                    start_after=False, 
                    start_index=0)

## COPY STATE
SPI_state = instr.SPI_dump_all(output_to_console=False)
DAC_values, _ = instr.read_current_values()
fastseq_str = '\n'.join([s.__str__() for s in seq])

## RUN SEQUENCE
instr.start_seq()
print('Starting')

## READ DATA
out_data = []
for i in range(avg):
    out_data += instr.get_ADC_data(Npts=len(V))
out_data = np.array(out_data, dtype=np.float)
print(out_data)

## CLOSE INSTRUMENT
instr.FPGA.close()

## PLOT
out_data = out_data.reshape((len(V), avg))
plt.figure(200) ; plt.ion()
Vmean = np.mean(out_data[:,2:], axis=1)
Vstd = np.std(out_data[:,2:], axis=1)
plt.errorbar(V.T, Vmean, Vstd, fmt='o')

## SAVE
# init_move_dt = np.dtype({'names':['name','value'],'formats':['S100','f8']})
# DAC_val_arr = np.array([(key, val) for key, val in DAC_values.items()], dtype=init_move_dt)
# with h5.File('''D:\Baptiste\CIR7\\4K\Suiveur_vs_BG.h5''', 'a') as f:
#     try:
#         grp = f['Settings']
#     except:
#         grp = f.create_group('Settings')
#         grp.attrs.create('SPI', SPI_state)
#         grp.attrs.create('DAC_values', DAC_val_arr, dtype=init_move_dt)
#         grp.attrs.create('Fastseq', fastseq_str)
    
#     print (list(f.keys()))
#     i = 0
#     while f'data_{i}' in f.keys():
#         i += 1
#     grp = f.create_group(f'data_{i}')
#     grp.create_dataset('Vin', data=V)
#     grp.create_dataset('Vout', data=out_data)
#     grp.attrs.create('VBGPA', DAC_val['VBGPA'])
#     grp.attrs.create('VBGNA', DAC_val['VBGNA'])
#     grp.attrs.create('Ipol', 100e-6)
    
#     f.close()

