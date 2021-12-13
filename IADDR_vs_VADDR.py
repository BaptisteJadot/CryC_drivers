# -*- coding: utf-8 -*-
"""
Created on Thu Nov 25 13:03:07 2021

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

def set_V(smu, channel="A", volts=0.):
    if channel in ["a", "A", 0]:
        smu.write(f"smua.source.levelv = {volts}")
    elif channel in ["b", "B", 1]:
        smu.write(f"smub.source.levelv = {volts}")
    else:
        raise ValueError(f"Incorrect channel {channel}")

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

## READ CURRENT STATE
SPI_state = instr.SPI_dump_all(output_to_console=True)
DAC_values, _ = instr.read_current_values()
instr.stop_seq()
i0 = instr.SPI_read(0xA1, 1)[0]+0
print(i0)
time.sleep(0.1)

## RAMP ADDRESS AND RECORD CURRENT
addr = "CMD_R0"
DAC_values[addr] = "smu"
if addr != "ADDR_5":
    DAC_values["ADDR_5"] = "floating"
Vsweep = np.linspace(0., 1.8, 181)
Iin = np.zeros_like(Vsweep)
for i, Vi in enumerate(Vsweep):
    set_V(smu, channel="A", volts=Vi)
    time.sleep(0.1)
    Imeas, *_ = read_IV(smu)
    print(f"Vin = {Vi:.2f} \t Iin = {Imeas*1e6:.2f}")
    Iin[i] = Imeas
set_V(smu, channel="A", volts=0.)

## CLOSE INSTRUMENT
instr.stop_seq()
instr.FPGA.close()

## SAVE
fname = "D:/Baptiste/CIR7/RT/Input_current_vs voltage.h5"
with h5.File(fname, 'a') as f:
    grp = f.create_group(addr)
    grp.create_dataset('Iin', data=Iin)
    grp.create_dataset('Vsweep', data=Vsweep)
    grp.attrs.create("SPI_settings", SPI_state)
    grp.attrs.create("Comment", DAC_values.__str__())


## PLOT
# runfile('D:/Baptiste/CIR7/RT/CIR7_plot_conso_vs_ADDR.py', wdir='D:/Baptiste/CIR7/RT')
plt.figure(492);plt.ion()
plt.plot(Vsweep, Iin*1e6, label=f"{addr}")
plt.legend()

