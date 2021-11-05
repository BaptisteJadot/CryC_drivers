# -*- coding: utf-8 -*-
"""
Created on Thu Oct 21 15:44:39 2021

@author: manip.batm
"""

from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
import Fastseq_elmts as fs
import numpy as np
import time
import h5py as h5
   
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

DAC_val['dummy'] = 1.2
instr.update_DAC(DAC_val)

## SPI
# instr.set_mode('addr')
# instr.set_mode('counter')
# instr.set_mode('lmt_counter', start=0, stop=63)
instr.set_mode('register', values=list(range(64)))
# instr.set_mode('sram', values=list(range(64))))
# instr.set_mode('direct')

instr.set_clk(int_clk=False, osc_vco=0., two_cycles=False, add_delay=False)

## INCREMENT CLOCK UNTIL ADDRESS 0
i0 = instr.SPI_read(0xA1, 1)[0]
print(i0)
while i0 != 0:
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
print(instr.get_ADC_data())

## PREPARE SEQ
v = [0.5]*64
# v[14] = 1.
# v = np.linspace(0,0.63,64)+0.2
v[16] = 1.2
instr.set_output(mux_mat=False, line0=None, line1=None, column0=16//2, column1=None)

seq = [fs.Trig_out(trig=[False]*4)]
for i in range(32):
    seq += [fs.Panel4(address=0, clk=False, even_value=v[(2*i)%64], odd_value=v[(2*i-1)%64])]
    seq += [fs.Panel4(address=0, clk=True, even_value=v[(2*i)%64], odd_value=v[(2*i-1)%64])]
    seq += [fs.Panel4(address=0, clk=False, even_value=v[(2*i)%64], odd_value=v[(2*i+1)%64])]
    seq += [fs.Panel4(address=0, clk=True, even_value=v[(2*i)%64], odd_value=v[(2*i+1)%64])]
# seq += [fs.Panel4(address=0, clk=False, even_value=v[62], odd_value=v[63])]
# seq = [fs.Trig_out(trig=[True]*4)]
# seq += [fs.Wait(value=1000000, precision='1ms')]
seq += [fs.JumpFor(target=1, count=99)] # set 100 times
seq += [fs.End()]
seq += [fs.ADC_get(channel=0)]
seq += [fs.JumpFor(target=len(seq)-1, count=99)] # read 100 times
seq += [fs.ADC_get(channel=1)]
seq += [fs.JumpFor(target=len(seq)-1, count=99)] # read 100 times
seq += [fs.ADC_get(channel=2)]
seq += [fs.JumpFor(target=len(seq)-1, count=99)] # read 100 times
seq += [fs.ADC_get(channel=3)]
seq += [fs.JumpFor(target=len(seq)-1, count=99)] # read 100 times
seq += [fs.End()]

instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=10, 
                    trig_reset_states=[True]*10, 
                    start_after=False, 
                    start_index=0)
instr.start_seq(start_ind=0)
time.sleep(2.)
_ = instr.get_ADC_data(Npts=None)
        
## RUN
t_tot = 60*3600
# t_tot = 120
dt_read = 1
dt_save = 60
Ntot = t_tot//dt_read
k = 0
out_data = np.zeros((Ntot, 5), dtype=np.float)
t0 = time.time()
last_read = 0
last_save = 0
while k < Ntot:
    t = time.time()
    if t - last_read > dt_read:
        instr.start_seq(start_ind=131)
        time.sleep(0.1)
        out_data[k, 0] = time.time()
        d = np.array(instr.get_ADC_data(Npts=400), dtype=np.float).reshape((100,4), order='F')
        out_data[k, 1:] = np.mean(d, axis=0)
        print(out_data[k, 1:])
        k += 1
        last_read = t
    if t - last_save > dt_save or k >= Ntot:
        h = int((t - t0)//3600)
        fname = f'''D:\Baptiste\CIR7\\4K\\Remanence_weekend\\remanence_weekend_12V_{h}.h5'''
        with h5.File(fname, 'a') as f:
            if 'data' in f.keys():
                del f['data']
            f.create_dataset('data', data=out_data)
            f.close()
        last_save = t
    time.sleep(0.1)
        
   
# CLOSE INSTRUMENT
instr.stop_seq()
instr.FPGA.close()
