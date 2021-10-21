# -*- coding: utf-8 -*-
"""
Created on Mon Oct 18 09:23:38 2021

@author: Baptiste
"""

from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
import Fastseq_elmts as fs
import numpy as np
import matplotlib.pyplot as plt
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
instr.set_mode('register', values=[16,15,14,17]*32)
# instr.set_mode('sram', values=list(range(64))))
# instr.set_mode('direct')

instr.set_clk(int_clk=False, osc_vco=0., two_cycles=False, add_delay=False)
instr.set_output(mux_mat=False, line0=None, line1=None, column0=16, column1=15)

## INCREMENT CLOCK UNTIL ADDRESS 16
i0 = instr.SPI_read(0xA1, 1)[0]
print(i0)
while i0 != 16:
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

seq += [fs.Panel4(clk=False, even_value=0.3, odd_value=0.8)]
seq += [fs.Panel4(clk=True, even_value=0.3, odd_value=0.8)] # 16

seq += [fs.Panel4(clk=False, even_value=0.3, odd_value=0.6)]
seq += [fs.Panel4(clk=True, even_value=0.3, odd_value=0.6)] # 15

seq += [fs.Panel4(clk=False, even_value=0.5, odd_value=0.6)]
seq += [fs.Panel4(clk=True, even_value=0.5, odd_value=0.6)] # 14

seq += [fs.Panel4(clk=False, even_value=0.5, odd_value=0.8)]
seq += [fs.Panel4(clk=True, even_value=0.5, odd_value=0.8)] # 17

avg = 10000
avg1 = avg//100
avg2 = 100
seq += [fs.ADC_get(channel=2)]
seq += [fs.ADC_get(channel=3)]
seq += [fs.Wait(value=990, precision='1us')]
seq += [fs.JumpFor(target=len(seq)-3, count=avg1-1)]
seq += [fs.JumpFor(target=len(seq)-4, count=avg2-1)]
    
seq += [fs.Panel4(clk=False, even_value=0.3, odd_value=0.8)]
seq += [fs.Panel4(clk=True, even_value=0.3, odd_value=0.8)] # 16

seq += [fs.Panel4(clk=False, even_value=0.3, odd_value=0.6)]
seq += [fs.Panel4(clk=True, even_value=0.3, odd_value=0.6)] # 15

seq += [fs.Panel4(clk=False, even_value=0.5, odd_value=0.6)]
seq += [fs.Panel4(clk=True, even_value=0.5, odd_value=0.6)] # 14

seq += [fs.Panel4(clk=False, even_value=0.5, odd_value=0.8)]
seq += [fs.Panel4(clk=True, even_value=0.5, odd_value=0.8)] # 17
    
seq += [fs.End()]

instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=5, 
                    trig_reset_states=[True]*10, 
                    start_after=False, 
                    start_index=0)
## RUN
print(instr.get_ADC_data())
instr.start_seq()
out_data = []
for i in range(avg2):
    out_data += instr.get_ADC_data(Npts=2*avg1)
out_data = np.array(out_data, dtype=np.float).reshape((avg1*avg2,2))
# print(out_data)

# CLOSE INSTRUMENT
instr.stop_seq()
instr.FPGA.close()

## PLOT
plt.figure(225) ; plt.clf(); plt.ion()
t = np.arange(avg)*1.
dV16 = out_data[:,0] - np.mean(out_data[:,0])
dV15 = out_data[:,1] - np.mean(out_data[:,1])

plt.plot(t, dV16*1000)
plt.plot(t, dV15*1000)
plt.xlabel('t (ms)')
plt.ylabel('dV (mV)')
# plt.legend(['MUX_mat OFF', 'MUX_mat ON'])
# plt.title('SO<3> (addr 15) vs Vodd')
plt.grid()



