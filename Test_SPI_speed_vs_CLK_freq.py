# -*- coding: utf-8 -*-
"""
Created on Thu Sep  9 15:51:01 2021

@author: manip.batm
"""

from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
from FPGA_utils import build_order
import Fastseq_elmts as fs
import numpy as np
import time
import matplotlib.pyplot as plt
import h5py as h5


def test_SPI(instr, speed_ticks):
    ## SET SPI SPEED
    instr.send_orders([build_order([(2, 8), (4, 8), (0, 16), (speed_ticks, 32)])]) # SPI speed
    
    ## SEND RANDOM ADDRESSES
    addr = np.random.randint(0, 64, 64)
    instr.SPI_write(0, addr)
    
    ## READ REGISTER CONTENT
    ans = instr.SPI_read(0, 64)
    
    ## RETURN NUMBER OF ERRORS
    Nerr = sum(ans != addr)
    return Nerr
    
## OPEN INSTRUMENT
bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main.lvbitx"""
ip_address = "192.168.1.21"
instr = CIR7Driver(ip_address, bitfile_path, DAC_dict)
# print(instr.FPGA.fpga_vi_state)

instr.reset()

VDDC = 1.
VBG = 0.
DAC_val = {}
DAC_val['VBGNC'] = VBG
DAC_val['VBGPC'] = -VBG
DAC_val['VBGNM'] = 0.
DAC_val['VBGPM'] = 0.
instr.update_DAC(DAC_val)

## INT CLK
int_clk = False
osc_vco = 0.
instr.set_clk(int_clk, osc_vco, two_cycles=False, add_delay=False)
SPI_settings = instr.SPI_dump_all(False)

## EXT CLK
ext_clk_MHz = 0.001
ext_clk_ticks = max(np.round((40/ext_clk_MHz)-21), 0)
seq = []
seq += [fs.Trig_out(address=0, clk=True, trig=[True]*4)]
seq += [fs.Wait(value=ext_clk_ticks/80, precision='ticks')]
seq += [fs.Trig_out(address=0, clk=False, trig=[False]*4)]
seq += [fs.Wait(value=ext_clk_ticks/80, precision='ticks')]
seq += [fs.JumpFor(target=0, count=0)] # play infinitely
seq += [fs.End()]

if not int_clk:
    instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                        us_per_DAC=10, 
                        trig_reset_states=[True]*10, 
                        start_after=True, 
                        start_index=0)
else:
    instr.stop_seq()

speed_ticks = np.arange(200,2300,1)
N = 10
error_count = np.zeros((len(speed_ticks), N))

for i, s in enumerate(speed_ticks):
    for k in range(N):
        error_count[i,k] = test_SPI(instr, s)
    print(f'{40/s*1000:.2f} kHz ({s} ticks) : {np.mean(error_count[i,:])} errors.')

instr.FPGA.close()

# ## PLOT
# # plt.figure(352) ; plt.ion()
# # m = np.mean(40000/max_speed, axis=1)
# # s = np.std(40000/max_speed, axis=1)
# # plt.errorbar(VBG, m, s, fmt='o')
# # plt.xlabel('VBGN = -VBGP (V)')
# # plt.ylabel('max SPI clock (kHz)')
# # plt.grid()

## SAVE
# init_move_dt = np.dtype({'names':['name','value'],'formats':['S100','f8']})
# DAC_val_arr = np.array([(key, val) for key, val in DAC_val.items()], dtype=init_move_dt)
with h5.File('''D:\Baptiste\CIR7\\RT\SPI_speed_vs_CLK_EXT.h5''', 'a') as f:
    i = 0
    while f'data_{i}' in f.keys():
        i += 1
    grp = f.create_group(f'data_{i}')
    grp.create_dataset('speed_ticks', data=speed_ticks)
    grp.create_dataset('error_count', data=error_count)
    grp.attrs.create('VDDC', VDDC)
    grp.attrs.create('VBGNC', VBG)
    grp.attrs.create('N', N)
    grp.attrs.create('VBGPC', -VBG)
    grp.attrs.create('SPI_settings', SPI_settings)
    grp.attrs.create('OSC_VCO', osc_vco)
    grp.attrs.create('int_clk', int_clk)
    grp.attrs.create('ext_clk_MHz', ext_clk_MHz)
    f.close()
