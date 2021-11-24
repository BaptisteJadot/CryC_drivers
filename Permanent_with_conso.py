# -*- coding: utf-8 -*-
"""
Created on Tue Nov 23 13:31:37 2021

@author: manip.batm
"""


from CIR7_driver import CIR7Driver
from CIR7_config import DAC_dict
import Fastseq_elmts as fs
import numpy as np
import time
import pyvisa
import matplotlib.pyplot as plt
import h5py as h5

def read_IV(smu):
    Ia = float(smu.query("print(smua.measure.i())"))
    Va = float(smu.query("print(smua.measure.v())"))
    Ib = float(smu.query("print(smub.measure.i())"))
    Vb = float(smu.query("print(smub.measure.v())"))
    return Ia, Va, Ib, Vb


## OPEN COMM WITH SMU
rm = pyvisa.ResourceManager()
smu = rm.open_resource("GPIB1::26::INSTR", write_termination="\n", read_termination="\n")
smu.write("beeper.beep(0.2, 880)")

## OPEN INSTRUMENT
bitfile_path = """C:/Users/manip.batm/Documents/FPGA_Batch_2_0_5_CryoC009/Labview_2019/FPGA Bitfiles/FPGA_CIR7_main_analog_addr.lvbitx"""
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

DAC_val['VBGPA'] = 0.
DAC_val['VBGNA'] = 0.
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

DAC_val['ADDR_0'] = 1.8
DAC_val['ADDR_1'] = 0.
DAC_val['ADDR_2'] = 0.
DAC_val['ADDR_3'] = 0.
DAC_val['ADDR_4'] = 0.
DAC_val['ADDR_5'] = 0.
instr.update_DAC(DAC_val)

## SPI
# instr.set_mode('addr')
# instr.set_mode('counter')
# instr.set_mode('lmt_counter', start=0, stop=63)
instr.set_mode('register', values=list(range(64)))
# instr.set_mode('sram', values=list(range(64)))
# instr.set_mode('direct')

instr.set_clk(int_clk=False, osc_vco=0., two_cycles=False, add_delay=False)
v = [0.]*64
v[14] = 0.
v[17] = 0.
f_kHz = 100

instr.set_output(mux_mat=False, line0=None, line1=None, column0=None, column1=None)
# instr.set_output(mux_mat=False, line0=14//2, line1=17//2, column0=None, column1=None)

## FAST SEQUENCE
seq = [fs.Trig_out(trig=[False]*4)]
for i in range(32):
    seq += [fs.Panel4(address=0, clk=False, even_value=v[(2*i)%64], odd_value=v[(2*i-1)%64])]
    seq += [fs.Panel4(address=0, clk=True, even_value=v[(2*i)%64], odd_value=v[(2*i-1)%64])]
    seq += [fs.Panel4(address=0, clk=False, even_value=v[(2*i)%64], odd_value=v[(2*i+1)%64])]
    seq += [fs.Panel4(address=0, clk=True, even_value=v[(2*i)%64], odd_value=v[(2*i+1)%64])]
# seq += [fs.Panel4(address=0, clk=False, even_value=v[62], odd_value=v[63])]
# seq = [fs.Trig_out(trig=[True]*4)]
# seq += [fs.Wait(value=1000000, precision='1ms')]
seq += [fs.JumpFor(target=1, count=0)] # play infinitely
seq += [fs.End()]

instr.config_seq(slots={i:s for (i,s) in enumerate(seq)}, 
                    us_per_DAC=int(1000//(2*f_kHz)), 
                    trig_reset_states=[True]*10, 
                    start_after=False, 
                    start_index=0)

## READ CURRENT STATE
SPI_state = instr.SPI_dump_all(output_to_console=True)
DAC_values, _ = instr.read_current_values()
instr.stop_seq()
i0 = instr.SPI_read(0xA1, 1)[0]+0
print(i0)
print(((i0)%64)*2+1,seq[((i0)%64)*2+1])
time.sleep(0.1)

## RAMP ADDRESS AND RECORD CONSO
addr = "VBGNM"
if addr != "ADDR_5":
    DAC_values["ADDR_5"] = "floating"
Vsweep = np.linspace(0., 3., 301)
V18A = np.zeros((len(Vsweep), 2))
V18N = np.zeros((len(Vsweep), 2))
I18A = np.zeros((len(Vsweep), 2))
I18N = np.zeros((len(Vsweep), 2))

# first with sequence OFF
for i, Vi in enumerate(Vsweep):
    instr.update_DAC({addr:Vi})
    time.sleep(0.02)
    tup = read_IV(smu)
    print(f"CLK OFF {i} \t {tup[0]*tup[1]*1e6:.2f} \t {tup[2]*tup[3]*1e6:.2f}")
    I18A[i, 0], V18A[i, 0], I18N[i, 0], V18N[i, 0] = tup

# next with sequence ON
instr.start_seq(start_ind=((i0)%64)*2+1)
for i, Vi in enumerate(Vsweep):
    instr.update_DAC({addr:Vi})
    time.sleep(0.02)
    tup = read_IV(smu)
    print(f"CLK ON {i} \t {tup[0]*tup[1]*1e6:.2f} \t {tup[2]*tup[3]*1e6:.2f}")
    I18A[i, 1], V18A[i, 1], I18N[i, 1], V18N[i, 1] = tup

## CLOSE INSTRUMENT
instr.stop_seq()
instr.FPGA.close()

## SAVE
fname = "D:/Baptiste/CIR7/RT/Conso_vs_BG.h5"
with h5.File(fname, 'a') as f:
    grp = f.create_group(addr)
    grp.create_dataset('V18A', data=V18A)
    grp.create_dataset('I18A', data=I18A)
    grp.create_dataset('V18N', data=V18N)
    grp.create_dataset('I18N', data=I18N)
    grp.create_dataset('Vsweep', data=Vsweep)
    grp.create_dataset('CLK_freq_kHz', data=np.array([0., f_kHz]))
    grp.attrs.create("SPI_settings", SPI_state)
    grp.attrs.create("i0", i0)
    grp.attrs.create("fast_seq", "\n".join([s.__str__() for s in seq]))
    grp.attrs.create("Comment", "ADDR0 & ADDR5 floating, " + DAC_values.__str__())


## PLOT
runfile('D:/Baptiste/CIR7/RT/CIR7_plot_conso_vs_ADDR.py', wdir='D:/Baptiste/CIR7/RT')
# plt.figure(492);plt.ion()
# plt.plot(Vsweep, I18A[:, 0]*V18A[:, 0]*1e6, label=f"PDD18A vs {addr}")

# plt.figure(493);plt.ion()
# plt.plot(Vsweep, I18N[:, 0]*V18N[:, 0]*1e6, label=f"PDD18N vs {addr}")

